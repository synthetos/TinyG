"""E155: in G20, G28.3, G92 and G43.1 scale A/B/C values by 25.4 as if they were inches.

canonical_machine.c cm_set_absolute_origin(), cm_set_origin_offsets() and cm_tool_offset_set() call
_to_millimeters() for every axis; cm_set_coord_offsets() (G10) already passes rotary values through.
Rotary words are degrees in either units mode. Positions are read back in G21 so reporting units
cannot hide the error.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")


def _a_enabled(tg):
    if tg.get("aam") == 0:
        pytest.skip("A axis disabled")


@pytest.mark.bug("E155", on=["d449"])
def test_g20_rotary_origin_and_offsets_are_degrees(tg):
    _a_enabled(tg)
    try:
        tg.send_gcode("G20 G28.3 A10", check=True)         # machine A = 10 degrees
        tg.send_gcode("G21", check=True)
        assert tg.get("mpoa") == pytest.approx(10, abs=1e-3), "G28.3 A10 in G20 was scaled"
        tg.send_gcode("G20 G92 A4", check=True)            # work A = 4 degrees here
        tg.send_gcode("G21", check=True)
        assert tg.get("posa") == pytest.approx(4, abs=1e-3), "G92 A4 in G20 was scaled"
    finally:
        tg.send_gcode("G21 G92.1")
        tg.send_gcode("G28.3 A0")
