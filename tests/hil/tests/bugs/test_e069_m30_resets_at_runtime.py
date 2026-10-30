"""E069: M2/M30 program-end resets run at runtime and overwrite modes set by blocks parsed after the M30.

edge canonical_machine.c:1804-1808 cm_program_end() only queues _exec_program_finalize, which runs from the
stepper loader ISR (stepper.c:961-962) and its END branch (:1759-1771) rewrites the model:
cm_set_coord_system(cm.default_coord_system), cm_set_distance_mode(cm.default_distance_mode), ... So a G56 or
G91 parsed after the M30, while an earlier move is still running, is undone when the M30 executes.
Master has the same pattern (canonical_machine.c:1360-1371, 1408-1411).
"""
import pytest

MOVE = "1 mm +X at F60 before M30, then back"


@pytest.mark.motion
@pytest.mark.bug("E069", on=["d440", "d449"])
def test_modes_after_m30_survive(tg):
    # pick modes that differ from the $gco/$gdi defaults M30 restores
    coord, coord_word = (4, "G57") if tg.get("gco") == 3 else (3, "G56")
    dist, dist_word = (0, "G90") if tg.get("gdi") == 1 else (1, "G91")
    tg.send_gcode("G55", check=True)
    tg.send_gcode("G91 G1 X1 F60", check=True, allow=MOVE)      # ~1 s, so the M30 is still queued
    tg.send_gcode("M30", check=True)
    tg.send_gcode(f"{coord_word} {dist_word}", check=True)
    try:
        tg.wait_idle()
        assert (tg.get("coor"), tg.get("dist")) == (coord, dist)
    finally:
        tg.send_gcode("G91 G1 X-1 F600", allow=MOVE)
        tg.wait_idle()
