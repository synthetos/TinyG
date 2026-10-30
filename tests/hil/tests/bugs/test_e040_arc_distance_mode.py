"""E040: G90.1/G91.1 (arc distance mode) are parsed but never executed, and absolute IJK would ignore work offsets.

edge gcode_parser.c:353/:361 only set gn/gf.arc_distance_mode; _execute_gcode_block (:508-510) has no
EXEC_FUNC(cm_set_arc_distance_mode, ...), so cm.gm.arc_distance_mode stays INCREMENTAL (canonical_machine.c:589).
If it were applied, plan_arc.c:231-234 subtracts the machine position from work-coordinate I/J/K.
Master has no admo token; the arc test is limited to 449 with it.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")

MOVE = "absolute-IJK arc test (2 mm semicircle from a G28.3 origin, returns to it)"


@pytest.mark.bug("E040", on=["d449"])
def test_g90_1_sets_absolute_arc_mode(tg):
    try:
        tg.send_gcode("G90.1", check=True)
        assert tg.get("admo") == 0
    finally:
        tg.send_gcode("G91.1", check=True)     # M2 does not reset the arc distance mode


def test_g91_1_reports_incremental(tg):
    tg.send_gcode("G91.1", check=True)
    assert tg.get("admo") == 1


@pytest.mark.motion
@pytest.mark.bug("E040", on=["d449"])
def test_absolute_ijk_uses_work_coordinates(tg):
    # Work X10 = machine X0; centre at work X11, end at work X12 (machine 2). Both the current
    # firmware (incremental centre at work 21) and a fix without offsets reject the arc as an
    # impossible centre, so neither moves.
    try:
        tg.send_gcode("G21 G90 G17", check=True)
        tg.send_gcode("G28.3 X0 Y0", check=True)
        tg.send_gcode("G92 X10 Y0", check=True)
        assert tg.send_gcode("G90.1 G2 X12 Y0 I11 J0 F600", allow=MOVE).status == 0
        tg.wait_idle()
        assert tg.get("mpox") == pytest.approx(2, abs=0.01)
        assert tg.get("mpoy") == pytest.approx(0, abs=0.01)
    finally:
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
        tg.send_gcode("G91.1", check=True)
        tg.send_gcode("G92.1", check=True)
        tg.send_gcode("G90 G53 G0 X0 Y0", allow=MOVE)    # also re-seeds a stale target (E054)
        tg.wait_idle()
