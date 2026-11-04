"""Diagnostics for source-review candidates (N09-N12): each reproduces one claim on a board and records what happened.

N11 hold ignored while output is blocked: with XON/XOFF (ex=1) the host sends XOFF, asks for enough output to fill the
    board's TX ring, then sends '!' during a long move. The ICE shows whether the hold ever happens before XON.
N09 negative ja: {"ja":-1} accepted? and does the planner still slow at a 90-degree corner (status-report velocity)?
N12 range gaps: 1sa tiny, ara=0 in radius mode, G93 with a minute feed, G4 P4295: the reply status and, through the
    ICE, whether the derived values are finite / the dwell ticks sane.
N12 step rate: a geometry asking for 80 000 steps/s at F30 (above the 50 kHz step tick): pulses vs commanded steps.
Needs --ice. Every setting is restored; the board is reset after the hold case.
"""
import math
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.resets_board,
              pytest.mark.dialect("d449"), pytest.mark.timeout(900)]

XOFF, XON = b"\x13", b"\x11"


def _finite(v):
    try:
        return math.isfinite(float(v))
    except Exception:
        return False


def test_n11_hold_ignored_while_tx_blocked(tg, keep, record, ice):
    keep("ex", "rxm", "ej", "si", "sv")
    for tok, val in (("ej", 1), ("rxm", 1), ("ex", 1), ("si", 100), ("sv", 1)):
        tg.set(tok, val, allow="xon/xoff for the blocked-output test")
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    info = {}
    tr = tg.transport
    try:
        tg.send_line("G91 G1 X40 F300")                        # 8 s move
        time.sleep(0.5)
        tr.write(XOFF, drain_timeout=None)                       # host says stop talking
        time.sleep(0.2)
        for _ in range(12):                                     # fill the board's TX ring behind the XOFF
            tr.write(b'{"x":null}\n', drain_timeout=None)
            time.sleep(0.05)
        time.sleep(0.5)
        tr.write(b"!", drain_timeout=None)
        time.sleep(1.5)
        info["after_hold_request"] = ice.peek_many(["cm.hold_state", "cm.motion_state", "mr.segment_velocity",
                                                     "us[0].fc_state_tx", "us[0].tx_buf_head", "us[0].tx_buf_tail"])
        info["pc"] = sorted({ice.where() for _ in range(4)})
        tr.write(b"\x04", drain_timeout=None)                    # job kill: also only honoured by the main loop
        time.sleep(1.0)
        info["after_job_kill"] = ice.peek_many(["cm.hold_state", "cm.motion_state", "mr.segment_velocity"])
        tr.write(XON, drain_timeout=None)                        # host lets the board talk again
        time.sleep(1.5)
        info["after_xon"] = ice.peek_many(["cm.hold_state", "cm.motion_state", "mr.segment_velocity"])
    finally:
        tr.write(XON, drain_timeout=None)
        time.sleep(0.3)
        tg.reset()
        tg.send_gcode("G21 G90 G54", allow="recover")
    record("n11", info)
    held_before_xon = info["after_hold_request"].get("cm.motion_state") in (0, 2) or info["after_hold_request"].get("cm.hold_state", 0) >= 6
    assert held_before_xon, f"'!' had no effect while output was blocked (hold waited for XON): {info}"


def test_n09_negative_ja(tg, keep, record):
    keep("ja", "xjm", "yjm")
    tg.send_gcode("G21 G90 G17 G94 G64 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    info = {}

    def corner_min_vel(ja):
        resp = tg.send_line(f'{{"ja":{ja}}}', timeout=3.0)
        info[f"set_ja_{ja}_status"] = resp.status
        tg.send_gcode("G28.3 X0 Y0", check=True, allow="declare the position")
        samples = []
        start = time.monotonic()
        tg.send_line("G91 G1 X10 F800")
        tg.send_line("G1 Y10")
        while time.monotonic() - start < 6:
            try:
                samples.append(float(tg.get("vel", timeout=0.5)))
            except Exception:
                pass
            time.sleep(0.05)
        tg.wait_idle(timeout=30)
        tg.send_gcode("G90", check=True)
        mid = samples[len(samples) // 4: 3 * len(samples) // 4] or samples
        return min(mid) if mid else None

    info["min_vel_ja_100000"] = corner_min_vel(100000)
    info["min_vel_ja_-1"] = corner_min_vel(-1)
    record("n09", info)
    assert info["set_ja_-1_status"] != 0, f"ja=-1 was accepted: {info}"


def _probe_case(tg, ice, record, name, steps):
    """Run one range-gap case; record progressively; recover from PANIC/ALARM with a reset."""
    info = {}
    try:
        for label, fn in steps:
            info[label] = fn()
            time.sleep(0.3)
            st = tg.stat()
            info[label + "_stat"] = st
            if st in (tg.dialect.stat("PANIC"), tg.dialect.stat("SHUTDOWN")):
                info["pc"] = sorted({ice.where() for _ in range(3)})
                break
    finally:
        record(name, info)
        try:
            tg.reset()
        except Exception:  # noqa: BLE001
            pass
        tg.send_gcode("G21 G90 G94 G54", allow="recover")
    return info


def test_n12_tiny_step_angle(tg, keep, record, ice):
    keep("1sa")
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0 A0", check=True, allow="declare the position")
    info = _probe_case(tg, ice, record, "n12_sa", [
        ("set_1sa_1e-38", lambda: tg.send_line('{"1sa":1e-38}', timeout=3.0).status),
        ("steps_per_unit", lambda: repr(ice.peek("st_cfg.mot[0].steps_per_unit"))),
        ("move", lambda: tg.send_line("G91 G1 X0.1 F300", timeout=5.0).status),
    ])
    assert info.get("set_1sa_1e-38") != 0 or _finite(info.get("steps_per_unit", "nan").strip("'")), f"non-finite steps/unit accepted: {info}"
    assert info.get("move_stat") not in (tg.dialect.stat("PANIC"), tg.dialect.stat("SHUTDOWN")), f"PANIC: {info}"


def test_n12_radius_zero(tg, keep, record, ice):
    keep("aam", "ara")
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0 A0", check=True, allow="declare the position")
    info = _probe_case(tg, ice, record, "n12_ara", [
        ("set_aam_3", lambda: tg.send_line('{"aam":3}', timeout=3.0).status),
        ("set_ara_0", lambda: tg.send_line('{"ara":0}', timeout=3.0).status),
        ("move", lambda: tg.send_line("G91 G1 A1 F100", timeout=5.0).status),
    ])
    assert info.get("set_ara_0") != 0, f"ara=0 accepted: {info}"
    assert info.get("move_stat") not in (tg.dialect.stat("PANIC"), tg.dialect.stat("SHUTDOWN")), f"PANIC: {info}"


def test_n12_inverse_time_tiny_feed(tg, keep, record, ice):
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    info = _probe_case(tg, ice, record, "n12_g93", [
        ("move", lambda: tg.send_line("G91 G93 G1 X0.01 F0.0000001", timeout=5.0).status),
        ("runtime", lambda: ice.peek_many(["mr.segments", "mr.segment_count", "mr.segment_time", "cm.motion_state"])),
    ])
    assert info.get("move_stat") not in (tg.dialect.stat("PANIC"), tg.dialect.stat("SHUTDOWN")), f"PANIC: {info}"


def test_n12_long_dwell(tg, keep, record, ice):
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    info = _probe_case(tg, ice, record, "n12_g4", [
        ("dwell", lambda: tg.send_line("G4 P4295", timeout=5.0).status),
        ("ticks", lambda: ice.peek_many(["st_pre.dda_ticks", "st_run.dda_ticks_downcount", "cm.motion_state"])),
    ])
    assert info.get("dwell_stat") not in (tg.dialect.stat("PANIC"), tg.dialect.stat("SHUTDOWN")), f"PANIC: {info}"


def test_n12_step_rate_overrun(tg, keep, record, ice):
    """A geometry asking for far more than the 50 kHz step tick can give at F30: steps the firmware counts vs commanded."""
    keep("1sa", "1mi", "1tr", "xvm", "xfr", "1ma")
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    for tok, val in (("1ma", 0), ("1sa", 1.8), ("1mi", 8), ("1tr", 0.01), ("xfr", 60), ("xvm", 60)):
        tg.set(tok, val, allow="step-rate overrun geometry")
    spu = ice.peek("st_cfg.mot[0].steps_per_unit")
    before = ice.peek("en.en[0].encoder_steps")
    tg.send_line("G91 G1 X1 F30")
    tg.wait_idle(timeout=60)
    after = ice.peek("en.en[0].encoder_steps")
    info = {"steps_per_unit": spu, "commanded_steps": spu * 1.0, "firmware_counted": after - before,
            "posx": tg.get("posx")}
    tg.send_gcode("G90", check=True)
    record("n12_rate", info)
    assert abs(info["firmware_counted"] - info["commanded_steps"]) <= 2, f"step count mismatch: {info}"
