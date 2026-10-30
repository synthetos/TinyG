"""E020: M0/M1/M60 do not pause; the next queued move starts without a cycle start ('~').

edge canonical_machine.c:1792-1796 cm_program_stop() only queues _exec_program_finalize(MACHINE_PROGRAM_STOP),
the same idle state cm_cycle_end uses, and that clears the hold (:1749-1755). plan_exec.c:62-65 then sets
MOTION_RUN for the next ALINE, so nothing waits for '~'. Master does the same (canonical_machine.c:1396-1400,
plan_exec.c mp_exec_move).

The at-rest position is the model's (canonical_machine.c:145), which already includes the queued move, so the
test watches the planner queue instead: after M0 the second move must still be waiting.
"""
import time

import pytest

MOVE = "two 1 mm +X moves separated by M0, then back"


@pytest.mark.motion
@pytest.mark.bug("E020", on=["d440", "d449"])
@pytest.mark.parametrize("stop", ["M0", "M60"])     # M1 may legitimately depend on an optional-stop switch
def test_program_stop_waits_for_cycle_start(tg, stop):
    tg.send_gcode("G91 G1 X1 F300", check=True, allow=MOVE)
    tg.send_gcode(stop, check=True)
    tg.send_gcode("G1 X1", check=True, allow=MOVE)
    try:
        time.sleep(2.0)                      # the first move takes ~0.3 s
        assert tg.get("qr") < tg.planner_buffers, "second move ran without a cycle start"
    finally:
        tg.send_raw(b"~")
        tg.wait_idle(timeout=10)
        tg.send_gcode("G91 G1 X-2 F600", allow=MOVE)
        tg.wait_idle()
