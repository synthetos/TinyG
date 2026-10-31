"""E075: a cyclic rotary axis (AXIS_RADIUS with tn == tm) searches only the latch backoff when homing.

edge cycle_homing.c:231-234 sets the search travel to |tm - tn| + lb, so with edge's own defaults for
A (radius mode, tn == tm meaning infinite travel) the search covers only lb and homing fails unless the
switch happens to be that close. g2core 0112c81 searches one full rotation; in radius mode the homing
distances are linear (converted to degrees through ara), so one rotation is 2*pi*ara.

Bench: motor 4 drives A; A is in radius mode with ara=10 (lb=5 mm of surface = 28.6 degrees), and a
software A-min switch is mounted 90 degrees away (test-hooks build, --ice).
"""
import pytest

pytestmark = [pytest.mark.ice, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449")]


@pytest.mark.profile("switches-no", "rotary-a-radius")
@pytest.mark.settings(**{"4ma": 3, "asn": 1, "asx": 0, "atn": 0, "atm": 0, "alb": 5, "azb": 1,
                         "asv": 600, "alv": 100})
@pytest.mark.bug("E075", on=[])
def test_cyclic_rotary_axis_homes_within_one_rotation(tg, switches, record):
    switches.mount("a", "min", at=-90.0)
    try:
        tg.send_gcode("G28.2 A0", check=True, allow="home A onto a software switch 90 degrees away")
        tg.wait_idle(timeout=90)
        record("mount", switches.mount_status("a", "min"))
        assert tg.get("homa") == 1, "A did not reach a switch 90 degrees away"
    finally:
        switches.unmount_all()
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
