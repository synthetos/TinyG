"""E144: after an alarm stops a move, a '%' flush and a clear leave the cycle running.

E044 latches a feedhold when an alarm fires. '%' flushes the planner but (by design) leaves the
hold for '~'; clearing the alarm only changed machine_state. So cycle_state stayed MACHINING
(cycs 1, stat 3) and every persisted SET failed with 'Can't save when moving' until a reset.
"""
import time

import pytest

pytestmark = pytest.mark.dialect("d449")

MOVE = "5 mm X move (3 s) stopped by an alarm, then back"


@pytest.mark.motion
@pytest.mark.persists
@pytest.mark.bug("E144", on=[])
def test_clear_after_alarm_and_flush_ends_the_cycle(tg, keep):
    keep("zsn")
    alarm = tg.dialect.stat("ALARM")
    tg.send_gcode("G21 G91", check=True)
    try:
        tg.send_gcode("G1 X5 F100", check=True, allow=MOVE)
        time.sleep(0.5)
        tg.send_json({"alarm": None})
        end = time.monotonic() + 5
        while tg.get("vel") > 0 and time.monotonic() < end:
            time.sleep(0.1)
        assert tg.stat() == alarm
        tg.transport.write(b"%\n")
        time.sleep(0.5)
        tg.drain()
        tg.clear_alarm()
        assert tg.get("cycs") == 0, "cycle still running after the alarm was cleared"
        zsn = tg.get("zsn")
        assert tg.send_json({"zsn": zsn}).status == 0     # a persisted SET must be accepted again
    finally:
        if tg.get("cycs") != 0:
            tg.reset()
        tg.send_gcode("G90 G1 X0 F600", allow=MOVE)
        tg.wait_idle()
