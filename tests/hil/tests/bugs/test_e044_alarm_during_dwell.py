"""E044: an ALARM raised while nothing is moving (during a dwell) neither holds nor flushes; queued moves then run.

edge cm_alarm (canonical_machine.c:769-771) relies on cm_request_feedhold, which needs
`cm.motion_state != MOTION_STOP` (:1554), and cm_request_queue_flush, which needs `cm.hold_state != FEEDHOLD_OFF`
(:1568). Only ALINE blocks set MOTION_RUN (plan_exec.c:62-65). A second dwell between the running dwell and
the move keeps the move from being prepped early (stepper.c:967-968, :1106), so motion is still STOP when the
alarm arrives and the G1 runs in ALARM after the dwells. The 'alarm' command does not exist on master.
"""
import time

import pytest

MOVE = "1 mm +X at F300 queued behind dwells; must not run after an alarm"


@pytest.mark.motion
@pytest.mark.dialect("d449")
@pytest.mark.bug("E044", on=["d449"])
def test_alarm_during_dwell_cancels_queued_move(tg):
    start = tg.get("posx")
    tg.send_gcode("G4 P2", check=True)
    tg.send_gcode("G4 P0.5", check=True)
    tg.send_gcode("G91 G1 X1 F300", check=True, allow=MOVE)
    tg.send_json({"alarm": None})
    try:
        assert tg.stat() == tg.dialect.stat("ALARM")
        vels = []
        end = time.monotonic() + 4.0         # the dwells end at ~2.5 s, the move would take ~0.3 s
        while time.monotonic() < end:
            vels.append(tg.get("vel"))
        assert max(vels) == 0, "queued move ran in ALARM"
        assert tg.get("qr") == tg.planner_buffers
    finally:
        tg.clear_alarm()
        tg.wait_idle(timeout=10)
        back = start - tg.get("posx")
        if abs(back) > 0.0005:
            tg.send_gcode(f"G91 G1 X{back:.4f} F600", allow=MOVE)
            tg.wait_idle()
