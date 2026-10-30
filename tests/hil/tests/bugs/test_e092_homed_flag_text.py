"""E092: text-mode $homx..$homc print the homed flag as an axis position.

config_app.c:151-156 binds homx..homc to cm_print_pos, so $homx prints "X position: 0.000 mm"
(edge: get_ui8 fills value_int but _print_pos prints value_flt; master prints 1.000 for homed).
The homing_state half (cycle_homing.c:209-212 sets HOMED after any single axis) needs homing
switches and is not tested here.
"""
import pytest


@pytest.mark.bug("E092", on=["d440", "d449"])
def test_homx_text_is_not_a_position(tg):
    text = " ".join(line for line in tg.send_text("$homx") if not line.startswith("tinyg ["))
    assert "position" not in text


def test_homx_json(tg):
    assert tg.get("homx") in (0, 1)
