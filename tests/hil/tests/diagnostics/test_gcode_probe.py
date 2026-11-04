"""Overnight probe: how the board answers unsupported, malformed and conflicting G-code, one line at a time.

Each line is sent alone at a known position; the reply status, any motion, any alarm and the board's health
afterwards are recorded. Fails on: a panic or shutdown, a MUST_REFUSE line answered with status 0, or a
refused line (status != 0) that moved the machine. UNSUPPORTED lines are recorded for review only (NIST says
unsupported codes should be errors; some may be ignored by design).
"""
import json
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.dialect("d449"), pytest.mark.timeout(1800)]

MUST_REFUSE = [
    "G28.9", "G28 X", "G38.9", "G92.5", "G1 X1X2", "G1 Xnan", "G1 X1e40", "G1 X--1", "G1 X1.2.3", "N-1 G1 X0",
    "G1 X1 G0 Y1", "G90 G91 X1", "G2 X1 Y1", "G1 F-5 X1", "Q5", "G1 X", "G1 X1 Y", "G99", "M99", "G9999",
    "G1 X[1+2]", "#1=5", "G1 X#1", "G0 X1 *55", "G1 X1 F0", "G4 P-1", "G2 X1 Y1 R0", "G10 L99 P1 X1",
    "G1 X1;" + "x" * 300,
]
UNSUPPORTED = [
    "G41 D1", "G42 D1", "G43 H1", "G49", "G81 R1 Z-1", "G83 R1 Z-1 Q1", "G80", "G33 Z-1 K1", "G76", "M98 P100",
    "O100 sub", "G5.1 X1 Y1", "G7", "G8", "G12", "G15", "G16", "G50", "G51", "G68 R45", "G95", "M6 T2", "M48",
    "M49", "M50", "M60", "G18 G2 X1 Z1 I1", "G19 G2 Y1 Z1 J1", "M3 M4", "G20 G21", "G61 G64",
]


def _pos(tg):
    return tuple(round(tg.get(f"mpo{a}"), 3) for a in "xyza")


def test_gcode_probe(tg, record):
    tg.send_gcode("G21 G90 G17 G94 G64", check=True)
    results, bugs, review = [], [], []
    for group, lines in (("must_refuse", MUST_REFUSE), ("unsupported", UNSUPPORTED)):
        for line in lines:
            tg.send_gcode("G21 G90 G17 G94", allow="reset modes")
            before = _pos(tg)
            tg.guard.check_gcode("G1 X1", allow="probe lines are short or refused")
            resp = tg.send_line(line, timeout=5.0)
            _settle(tg, 30)
            stat = tg.stat()
            after = _pos(tg)
            alarm = stat == tg.dialect.stat("ALARM")
            if alarm:
                tg.clear_alarm()
            moved = before != after
            r = {"group": group, "line": line[:60], "status": resp.status, "moved": moved, "alarm": alarm, "stat": stat}
            results.append(r)
            assert stat not in (tg.dialect.stat("PANIC"), tg.dialect.stat("SHUTDOWN")), f"{line!r} -> stat {stat}"
            if group == "must_refuse" and resp.status == 0:
                bugs.append(r)
            if resp.status != 0 and moved:
                bugs.append(r)
            if group == "unsupported" and (resp.status == 0 or alarm):
                review.append(r)
            if moved:                                    # put the machine back for the next probe
                tg.send_gcode(f"G53 G0 X{before[0]} Y{before[1]} Z{before[2]} A{before[3]}", allow="back")
                _settle(tg, 30)
            time.sleep(0.05)
    tg.send_gcode("M5 M9 G21 G90 G17 G94 G64 G92.1", allow="cleanup")
    record("probe", {"results": results, "bugs": bugs, "review": review})
    assert not bugs, json.dumps(bugs)

def _settle(tg, timeout):
    """Wait until idle, or return at once when the board is in ALARM (it never becomes idle then)."""
    end = time.monotonic() + timeout
    alarm, idle = tg.dialect.stat("ALARM"), tg.dialect.stats("READY", "PROGRAM_STOP", "PROGRAM_END")
    while time.monotonic() < end:
        s = tg.stat()
        if s == alarm or (s in idle and tg.get("qr") == 32):
            return s
        time.sleep(0.1)
    raise AssertionError(f"not idle and no alarm after {timeout} s (stat {tg.stat()})")
