"""E030: resuming from a feedhold re-runs the held block's old trapezoid from the hold point, overshooting.

edge plan_exec.c:236-238 only patches `bf->block_state`, `bf->length` and `bf->entry_vmax` at the hold point;
mp_calculate_trapezoid never runs again (bf->replannable=false at :183, _plan_block_list stops at :439 of
plan_line.c, mp_plan_hold_callback is declared in planner.h:298 but not defined). On resume :189-205 reload the
stale head/body/tail lengths and build waypoints from the hold point, so the move travels its full original
length again. Master replans in mp_plan_hold_callback (controller.c:165, plan_line.c:641-747).

At rest the board reports the model position (canonical_machine.c:145), which hides the overshoot, so the
runtime position is sampled while the resumed move runs.
"""
import time

import pytest

MOVE = "4 mm +X at F120 with a feedhold and resume, then back"


def _wait_hold(tg, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if tg.get("hold") == tg.dialect.hold("HOLD") and tg.get("vel") == 0:
            return
    raise TimeoutError("feedhold did not reach HOLD")


@pytest.mark.motion
@pytest.mark.resets_board
@pytest.mark.bug("E030", on=["d449"])
def test_resume_stops_at_endpoint(tg):
    start = tg.get("posx")
    tg.send_gcode("G91 G1 X4 F120", check=True, allow=MOVE)
    peak = start
    try:
        time.sleep(0.8)
        tg.send_raw(b"!")
        _wait_hold(tg)
        held = tg.get("posx")
        assert start < held < start + 4
        tg.send_raw(b"~")
        end = time.monotonic() + 6
        while time.monotonic() < end:
            peak = max(peak, tg.get("posx"))
            if tg.get("qr") == tg.planner_buffers and tg.get("vel") == 0:
                break
        assert peak <= start + 4 + 0.01, f"resumed move overshot to {peak:.3f} (endpoint {start + 4:.3f})"
    finally:
        if peak > start + 4 + 0.01:
            tg.wait_idle()
            tg.reset()                       # runtime and model positions now disagree
        else:
            tg.send_raw(b"~")
            tg.wait_idle()
            tg.send_gcode("G91 G1 X-4 F600", allow=MOVE)
            tg.wait_idle()
