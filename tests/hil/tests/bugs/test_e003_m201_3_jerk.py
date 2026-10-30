"""E003: M201.3 (set per-axis jerk transiently from Gcode) is missing on edge.

gcode_parser.c:384-400 has no `case 201`, so M201.3 returns STAT_MCODE_COMMAND_UNSUPPORTED (132).
Edge already has cm_set_axis_jerk (canonical_machine.c:2173-2177); master 440.21 implements
M201.3 as jerk[mm/s^3] * 216000 / 1e6, clamped to at least 1.
"""
import pytest


# The axis words set jerk, they do not move; the stock firmware rejects the block before executing it
# (gcode_parser.c:430-432). Marked motion anyway because the line carries axis words.
@pytest.mark.motion
def test_m201_3_sets_jerk(tg, keep):
    keep("xjm", "yjm")
    assert tg.send_gcode("M201.3 X1000", allow="M201.3 sets jerk only").status == 0
    assert tg.get("xjm") == pytest.approx(216, abs=0.01)       # 1000 mm/s^3 -> 216 x 1e6 mm/min^3
    assert tg.send_gcode("M201.3 Y0", allow="M201.3 sets jerk only").status == 0
    assert tg.get("yjm") == pytest.approx(1, abs=0.01)         # clamped to the minimum


@pytest.mark.motion
def test_m201_3_inches_preserve_rotary_units(tg, keep):
    keep("xjm", "ajm", "yjm")
    yjm = tg.get("yjm")
    x0 = tg.get("mpox")
    try:
        tg.send_gcode("G20", check=True)
        tg.send_gcode("M201.3 X1000 A1000", check=True, allow="M201.3 sets jerk only")
    finally:
        tg.send_gcode("G21", check=True)
    # xjm is serialized with zero decimal places; allow half a reporting unit.
    assert tg.get("xjm") == pytest.approx(5486.4, abs=0.5)
    assert tg.get("ajm") == pytest.approx(216, abs=0.01)
    assert tg.get("yjm") == pytest.approx(yjm, abs=0.01)
    assert tg.get("mpox") == pytest.approx(x0, abs=0.001)
