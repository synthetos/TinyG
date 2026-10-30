

@pytest.mark.ice
@pytest.mark.motion
@pytest.mark.profile("switches-no", "homing-x")
@pytest.mark.settings(zsn=1, zsx=0)
@pytest.mark.bug("E092", on=["d440", "d449"])
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
