"""E038: G43.1 or G49 anywhere in a block returns early and silently drops G54-G59, G90/G91, motion and M2 on that line.

edge gcode_parser.c:505-506 `if (cm.gf.tool_offset_set) { return(...); } if (cm.gf.tool_offset_cancel) { return(...); }`
runs before coordinate system, path control and distance mode (:508-510), next_action and motion.
Master ignores G49 (master gcode_parser.c:298 `case 49: break;`).
"""
import pytest


@pytest.mark.bug("E038", on=["d449"])
def test_g49_keeps_g90(tg):
    tg.send_gcode("G91", check=True)
    tg.send_gcode("G17 G21 G40 G49 G80 G90", check=True)
    assert tg.get("dist") == 0


@pytest.mark.bug("E038", on=["d449"])
def test_g49_keeps_coordinate_system(tg):
    tg.send_gcode("G55", check=True)
    tg.send_gcode("G54 G49", check=True)
    assert tg.get("coor") == 1


def test_g90_alone(tg):
    tg.send_gcode("G91", check=True)
    tg.send_gcode("G17 G21 G40 G80 G90", check=True)
    assert tg.get("dist") == 0
