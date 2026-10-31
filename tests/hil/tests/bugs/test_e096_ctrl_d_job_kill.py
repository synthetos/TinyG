"""E096: ^D (EOT, job kill) is ignored, so CNCjs 'Stop (force)' leaves the queued job running.

The USB RX ISR traps only ^X, '!', '%', '~' and ENQ (xio/xio_usb.c:142-161); in line mode
_post_buffer strips 0x04 as leading whitespace (xio.c:531), so "\\x04\\n" is a blank line.
CNCjs sends it for any fb >= 101 (TinyGController.js:1414-1419). Master has the same gap.

Spec (g2core wiki Job-Exception-Handling, Feedhold): ^d is trapped by the serial layer and raises an
ALARM (status 207, kill job): motion and spindle stop, the planner is flushed, and commands are
rejected until {clear:n}, $clear, M2 or M30; it works in or out of a feedhold.
"""
import time

import pytest

pytestmark = [pytest.mark.motion]


def _killed(tg, start, mark):
    a = tg.get("posx")
    time.sleep(0.5)
    b = tg.get("posx")
    assert b == pytest.approx(a, abs=0.001), "still moving after ^D"
    assert b - start < 4.9, "job ran to completion"
    assert tg.get("qr") == tg.planner_buffers, "planner not flushed"
    assert tg.stat() == tg.dialect.stat("ALARM")
    kill = tg.dialect.status("KILL_JOB")
    assert any(e.kind == "exception" and e.data["er"].get("st") == kill for e in tg.transport.history[mark:]), \
        "no kill-job exception reported"
    # the next command is rejected and does not move
    assert tg.send_gcode("G1 X1 F300").status == tg.dialect.status("COMMAND_REJECTED_BY_ALARM")
    time.sleep(0.3)
    assert tg.get("posx") == pytest.approx(b, abs=0.001)


def _restore(tg, start):
    tg.wait_idle(timeout=10)
    if tg.stat() in (tg.dialect.stat("ALARM"), tg.dialect.stat("SHUTDOWN")):
        tg.clear_alarm()
    tg.send_gcode(f"G90 G1 X{start:.3f} F300", check=True, allow="return to start")
    tg.wait_idle()


@pytest.mark.bug("E096", on=["d440"])
def test_ctrl_d_kills_job(tg):
    start = tg.get("posx")
    tg.send_gcode("G91 G1 X5 F120", check=True, allow="5 mm move at 2 mm/s")
    tg.send_gcode("G1 X5", check=True, allow="queued behind it")
    mark = len(tg.transport.history)
    try:
        time.sleep(0.8)
        tg.send_raw(b"\x04\n")      # what CNCjs writes
        time.sleep(0.8)             # decel from 2 mm/s at default jerk takes ~0.3 s
        _killed(tg, start, mark)
    finally:
        _restore(tg, start)


@pytest.mark.bug("E096", on=["d440"])
def test_ctrl_d_kills_job_in_feedhold(tg):
    start = tg.get("posx")
    tg.send_gcode("G91 G1 X5 F120", check=True, allow="5 mm move at 2 mm/s")
    try:
        time.sleep(0.8)
        tg.send_raw(b"!")
        time.sleep(0.8)
        assert tg.get("hold") == tg.dialect.hold("HOLD")
        mark = len(tg.transport.history)
        tg.send_raw(b"\x04")        # bare, no newline
        time.sleep(0.5)
        _killed(tg, start, mark)
        assert tg.get("hold") == tg.dialect.hold("OFF"), "the flush ends the hold"
    finally:
        _restore(tg, start)
