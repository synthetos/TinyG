"""E096: ^D (EOT, job kill) is ignored, so CNCjs 'Stop (force)' leaves the queued job running.

The USB RX ISR traps only ^X, '!', '%', '~' and ENQ (xio/xio_usb.c:142-161); in line mode
_post_buffer strips 0x04 as leading whitespace (xio.c:531), so "\\x04\\n" is a blank line.
CNCjs sends it for any fb >= 101 (TinyGController.js:1414-1419). Master has the same gap.
"""
import time

import pytest


@pytest.mark.motion
@pytest.mark.bug("E096", on=["d440", "d449"])
def test_ctrl_d_kills_job(tg):
    start = tg.get("posx")
    tg.send_gcode("G91 G1 X5 F120", check=True, allow="5 mm move at 2 mm/s")
    try:
        time.sleep(0.8)
        tg.send_raw(b"\x04\n")      # what CNCjs writes
        time.sleep(0.8)             # decel from 2 mm/s at default jerk takes ~0.3 s
        a = tg.get("posx")
        time.sleep(0.5)
        b = tg.get("posx")
        assert b == pytest.approx(a, abs=0.001), "still moving after ^D"
        assert b - start < 4.9, "job ran to completion"
        assert tg.get("qr") == tg.planner_buffers
    finally:
        tg.wait_idle(timeout=10)
        if tg.stat() in (tg.dialect.stat("ALARM"), tg.dialect.stat("SHUTDOWN")):
            tg.clear_alarm()
        tg.send_gcode(f"G90 G1 X{start:.3f} F300", check=True, allow="return to start")
        tg.wait_idle()
