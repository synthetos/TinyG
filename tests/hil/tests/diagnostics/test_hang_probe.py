"""Overnight probe: commands a careless or AI-driven host might send that could leave the machine stuck.

Each case is sent alone (some follow a setting with a move; settings are restored). Then the board must answer
{stat:null} within 2 s; if it is busy, paused or stuck, a recovery ladder runs and the step it needed is recorded:
  1 '!' then '%' (feedhold, flush)    2 ^D (job kill)    3 ^X (reset)
A HANG is a case that needed ^X or left the board unanswering: that fails the test. Cases meant to wait (M0, long
dwells, a probe that never touches) pass when step 1 frees the machine. Not sent: defa, boot, baud, text mode.
An "input lost" exception (alarm 13) after an accepted line also fails: the host waited for the reply and the
board still dropped the next line (persisted SET then a move; the self-test then hold and flush).
"""
import json
import os
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(3600)]

POLL = float(os.environ.get("HANG_PROBE_POLL", "0.5"))   # 0.1 reproduces the ring overflow during the self-test

LINES = [
    "G4 P999999", "G4 P-1", "M0", "M1", "M60", "G28.2 X0", "G28.2 Z0", "G38.2 Z-10 F10",
    "G1 X1000000 F1", "G91 G1 X10 F0.001", "G90", "G0 X1e9", "G2 X0 Y0 I1000000 J0 F100", "G2 X0 Y0 I0.0001 J0 F100",
    "G3 X5 Y5 R-5 F500", "G1 A9999999 F100", "G2 X0 Y0 I5 J0 P5 F2000", "G90.1", "G91.1", "G64 P0.1",
    '{"gc":"M0"}', '{"gc":"G4 P99"}', '{"txt":"{\\"gc\\":\\"G4 P99\\"}"}', '{"txt":"$$$"}',
    '{"rt":{"hb":1,"hb":0}}', '{"rt":{"fi":60}}', '{"sr":{}}', '{"sr":{"posx":true,"bogus":true}}', '{"qv":9}',
    '{"a":' * 30 + "1" + "}" * 30, '{"xvm":1' + " " * 230 + "}", "G1 X1\x00 Y1", "\x01\x02\x7f\xfe garbage",
    "!!!~~~", "%%%%", "~", "M2", "M30", "T1 M6", '{"test":1}',
]
SETTING_THEN_MOVE = [  # (setting, value, move): settings a host might set wrongly, then a move under them
    ("xvm", 0, "G0 X5"), ("xfr", 0, "G1 X5 F500"), ("xjm", 0, "G1 X5 F500"), ("ja", 0, "G1 X5 Y5 F500"),
    ("1mi", 0, "G1 X5 F500"), ("1sa", 0, "G1 X5 F500"), ("1tr", 0, "G1 X5 F500"), ("si", 0, "G1 X5 F500"),
    ("si", 99999999, "G1 X5 F500"), ("ct", 0, "G2 X0 Y0 I2 J0 F500"), ("xjd", 0, "G1 X5 Y5 F500"),
]


def _answers(tg, timeout=2.0):
    """Any reply means the board is alive (a homing error rides on the next reply as status 247)."""
    try:
        tg.send_line('{"stat":null}', timeout=timeout)
        return True
    except Exception:
        return False


def _settled(tg, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            s = tg.stat()
            if s in tg.dialect.stats("READY", "PROGRAM_STOP", "PROGRAM_END", "ALARM") and tg.get("qr") == 32:
                return True
        except Exception:
            pass
        time.sleep(POLL)    # slow poll by default: while {"test":n} reads from flash, host input only queues in
                            # the 248-byte RX ring, and ten polls a second overflow it (a true "input lost")
    return False


def _recover(tg):
    """The recovery step the board needed: 0 none, 1 hold+flush, 2 job kill, 3 reset, 4 still unanswering."""
    if _answers(tg) and _settled(tg, 3):
        return 0
    tg.send_raw(b"!"); time.sleep(0.8); tg.send_raw(b"%")
    if _settled(tg, 5) and _answers(tg):
        return 1
    tg.send_raw(b"\x04")
    if _settled(tg, 5) and _answers(tg):
        return 2
    try:
        tg.reset()
    except Exception:
        return 4
    return 3 if _answers(tg, 5) else 4


def test_hang_probe(tg, keep, record):
    keep(*sorted({s for s, _, _ in SETTING_THEN_MOVE}))
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0 A0", check=True, allow="declare the position")
    results, hangs, lost = [], [], []
    cases = [("line", l, None, None) for l in LINES] + [("setting", m, s, v) for s, v, m in SETTING_THEN_MOVE]
    cases = cases[int(os.environ.get("HANG_PROBE_FROM", "0")):]     # bisect: start at a later case (N35)
    for kind, line, tok, val in cases:
        tg.guard.check_gcode("G1 X1", allow="hang probe")
        status, set_status, original = None, None, None
        mark = len(tg.transport.history)
        try:
            if tg.stat() == tg.dialect.stat("ALARM"):         # left over from the previous case: clear, go on
                tg.clear_alarm()
            if kind == "setting":
                original = tg.get(tok)
                set_status = tg.send_line(json.dumps({tok: val}), timeout=3.0).status
            try:
                status = tg.send_line(line, timeout=3.0).status
            except Exception as e:
                status = f"no reply: {type(e).__name__}"
            time.sleep(0.5)
            step = _recover(tg)
        finally:
            if kind == "setting" and original is not None:
                try:
                    if tg.stat() == tg.dialect.stat("ALARM"):
                        tg.clear_alarm()
                    tg.set(tok, original, allow="restore after the hang probe")
                except Exception:
                    pass
        exc = [e.raw[:120] for e in tg.transport.history[mark:] if e.kind == "exception"]
        r = {"case": (f"{tok}={val} then " if tok else "") + repr(line)[:70], "set_status": set_status,
             "status": status, "recovery": ["none", "hold+flush", "job kill", "reset", "UNANSWERING"][step],
             "exceptions": exc}
        results.append(r)
        if step >= 3:
            hangs.append(r)
        if any("input lost" in x for x in exc):              # a line dropped after an accepted one (N35, N36)
            lost.append(r)
        try:
            if tg.stat() == tg.dialect.stat("ALARM"):
                tg.clear_alarm()
            tg.send_gcode("M5 M9 G21 G90 G17 G94 G92.1 G54", allow="reset modes")
            tg.send_line("G91.1")                        # default arc-centre mode (the probe sends G90.1)
            tg.send_gcode("G28.3 X0 Y0 Z0 A0", allow="declare the position")
        except Exception:
            break
    record("hang_probe", {"results": results, "hangs": hangs, "input_lost": lost, "cases": len(cases), "ran": len(results)})
    assert len(results) == len(cases), f"only {len(results)} of {len(cases)} cases ran (cleanup failed): last {results[-1:]}"
    assert not hangs, "needed a reset or left the board unanswering: " + json.dumps(hangs[:6])
    assert not lost, "input lost (alarm 13) after an accepted command: " + json.dumps(lost[:6])
