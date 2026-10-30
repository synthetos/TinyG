"""E074: G2/G3 with only the motion word (e.g. "G2 F100") raises ALARM instead of being a no-op.

edge plan_arc.c:133-137 only traps blocks without a G2/G3 word; "G2 F100" falls through to
full_circle=true (:168-173) with zero offsets and alarms at :240 (STAT_ARC_OFFSETS_MISSING_FOR_SELECTED_PLANE).
Master reaches a zero-radius full circle and returns STAT_MINIMUM_LENGTH_MOVE (201) as an error
(master plan_arc.c:161-167). No axis or offset words, so nothing moves on either build.
"""
import pytest


@pytest.mark.bug("E074", on=["d440", "d449"])
def test_bare_g2_is_a_noop(tg):
    tg.send_gcode("G21 G90 G17", check=True)
    assert tg.send_gcode("G2 F100").status == 0
    assert tg.stat() != tg.dialect.stat("ALARM")
