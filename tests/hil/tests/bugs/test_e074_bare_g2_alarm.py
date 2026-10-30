"""E074: G2/G3 with only the motion word (e.g. "G2 F100") raises ALARM instead of being a no-op.

edge plan_arc.c:133-137 only traps blocks without a G2/G3 word; "G2 F100" falls through to
full_circle=true (:168-173) with zero offsets and alarms at :240 (STAT_ARC_OFFSETS_MISSING_FOR_SELECTED_PLANE).
Master reaches a zero-radius full circle and returns STAT_MINIMUM_LENGTH_MOVE (201) as an error
(master plan_arc.c:161-167). No axis or offset words, so nothing moves on either build.
"""
import pytest


@pytest.mark.bug("E074", on=["d440"])
def test_bare_g2_is_a_noop(tg):
    tg.send_gcode("G21 G90 G17", check=True)
    assert tg.send_gcode("G2 F100").status == 0
    assert tg.stat() != tg.dialect.stat("ALARM")


MOVE = "half-circle G2/G3 of 2 mm in X, then back"
G1, G2, G3 = 1, 2, 3     # momo values (cmMotionMode)


@pytest.mark.bug("E074", on=["d449"])
def test_bare_arc_word_selects_the_motion_mode(tg):
    tg.send_gcode("G21 G90 G17 G1 F100", check=True)
    tg.send_gcode("G2 F100", check=True)
    assert tg.get("momo") == G2
    tg.send_gcode("G3", check=True)
    assert tg.get("momo") == G3


@pytest.mark.motion
@pytest.mark.bug("E074", on=["d449"])
def test_offset_only_arc_uses_the_bare_mode(tg):
    tg.send_gcode("G21 G17 G91 G1 F600", check=True)
    tg.send_gcode("G3", check=True)                    # select CCW without moving
    try:
        assert tg.send_gcode("X2 Y0 I1 J0", allow=MOVE).status == 0   # no motion word: uses the mode
        tg.wait_idle()
        assert tg.get("momo") == G3
    finally:
        tg.send_gcode("G1 X-2 Y0 F600", allow=MOVE)
        tg.wait_idle()
