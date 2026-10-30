"""E092: text-mode $homx..$homc print the homed flag as an axis position.

config_app.c:151-156 binds homx..homc to cm_print_pos, so $homx prints "X position: 0.000 mm"
(edge: get_ui8 fills value_int but _print_pos prints value_flt; master prints 1.000 for homed).
The homing_state half (cycle_homing.c:209-212 sets HOMED after any single axis) uses a software
X-min switch (test-hooks build, --ice) with Z configured as a homing axis.
"""
import pytest


@pytest.mark.bug("E092", on=["d440"])
def test_homx_text_is_not_a_position(tg):
    text = " ".join(line for line in tg.send_text("$homx") if not line.startswith("tinyg ["))
    assert "position" not in text


def test_homx_json(tg):
    assert tg.get("homx") in (0, 1)


@pytest.mark.ice
@pytest.mark.motion
@pytest.mark.profile("switches-no", "homing-x")
@pytest.mark.settings(zsn=1, zsx=0)
@pytest.mark.bug("E092", on=["d440"])
def test_homing_one_axis_is_not_homed_while_z_is_unhomed(tg, switches):
    """cycle_homing.c:209-212 sets HOMED when the requested axes finish, whichever they were."""
    switches.mount("x", "min", at=-3.0)
    try:
        tg.send_gcode("G28.2 X0", check=True, allow="home X onto a software switch 3 mm away")
        tg.wait_idle(timeout=60)
        assert tg.get("homx") == 1 and tg.get("homz") == 0
        assert tg.get("home") == 0, "machine reports Homed while Z (a homing axis) is not homed"
    finally:
        switches.unmount_all()
