"""Diagnostic: a timed sequence of single-axis moves and spindle/coolant commands, for mapping instrument channels.

An external capture (logic analyzer or MSO) records while this runs; each step's wall-clock time is recorded so the
capture's edges can be assigned to the step that caused them. Bench bring-up only (bench 3 has no motors).
SEQ=motors (default) moves X, Y, Z, A by +2 then -2 one at a time; SEQ=outputs runs M3/M4/M5 at two speeds and
M7/M8/M9. START_DELAY seconds of idle first, GAP seconds between steps.
"""
import os
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.dialect("d449"), pytest.mark.timeout(300)]

SEQ = os.environ.get("SEQ", "motors")
START_DELAY = float(os.environ.get("START_DELAY", "1.5"))
GAP = float(os.environ.get("GAP", "1.5"))


def _steps():
    if SEQ == "pwm":
        return ["M3 S1000", "M3 S1500", "M3 S2000", "M4 S1500", "M5"]
    if SEQ == "outputs":
        return ["M3 S1000", "M3 S6000", "M5", "M4 S3000", "M5", "M7", "M9", "M8", "M9"]
    out = []
    for a in "XYZA":
        out += [f"G1 {a}2 F600", f"G1 {a}-2 F600"]
    return out


def test_bench_signal_sequence(tg, keep, record):
    keep("1pm", "2pm", "3pm", "4pm")
    tg.send_gcode("G21 G91 G94", check=True)
    log = []
    time.sleep(START_DELAY)
    for line in _steps():
        tg.guard.check_gcode(line, allow="bench bring-up signal sequence, no motors fitted")
        t = time.time()
        status = tg.send_line(line, timeout=5.0).status
        log.append({"wall": round(t, 4), "line": line, "status": status})
        time.sleep(GAP)
    tg.wait_idle(timeout=20)
    tg.send_gcode("G90 M5 M9", allow="leave spindle and coolant off")
    record("signal_sequence", {"seq": SEQ, "steps": log})
    print("STEPS", log)
