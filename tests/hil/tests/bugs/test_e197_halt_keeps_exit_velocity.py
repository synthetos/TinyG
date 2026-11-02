"""E197: halting the runtime keeps the halted block's exit velocity, so the next move starts at speed.

mp_halt_runtime() (planner.c, called by cm_halt_motion() from cm_shutdown() and cm_panic()) stops the steppers
and resets the planner queues, but planner_reset() leaves mr.exit_velocity alone. After {clear} leaves
SHUTDOWN without a reset (E045), the next move's entry comes from the runtime (plan_line.c
_exit_of_previous(): mr.exit_velocity when no block is queued before it), so it is planned to start at the
halted block's exit speed while the machine stands still.

The board test shuts down in the middle of a chain of collinear moves at F1500, whose running block plans to
leave at speed, and reads mr.exit_velocity with the ICE: the machine is at rest, so it must be 0.
"""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.ice, pytest.mark.dialect("d449"),
              pytest.mark.bug("E197", on=[])]

MACHINE_SHUTDOWN = 7


def test_shutdown_leaves_no_exit_velocity(tg, ice, record):
    x0 = tg.get("mpox")
    tg.send_gcode("G91", check=True)
    try:
        for _ in range(12):                                  # 12 mm of 1 mm collinear moves
            tg.send_gcode("G1 X1 F1500", check=True, allow="12 mm chain at F1500")
        time.sleep(0.25)
        tg.send_json({"shutd": None}, check=True)            # a GET of shutd performs it (E138)
        time.sleep(0.05)
        snap = ice.peek_many(["cm.machine_state", "mr.exit_velocity"])
        record("after shutdown", snap)
        state, exit_velocity = snap["cm.machine_state"], snap["mr.exit_velocity"]
        assert state == MACHINE_SHUTDOWN
        assert exit_velocity == 0, f"halted runtime keeps exit velocity {exit_velocity} mm/min"
    finally:
        tg.send_json({"clear": None})                        # {clear} leaves SHUTDOWN (E045)
        if tg.stat() == tg.dialect.stat("SHUTDOWN"):
            tg.reset()                                       # positions are not kept: no return move
        else:
            tg.send_gcode("G90", check=True)
            tg.send_gcode(f"G53 G1 X{x0:.3f} F1500", check=True, allow="return to start")
            tg.wait_idle(timeout=20)
