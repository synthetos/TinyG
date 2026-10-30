"""E055: G28.4 rewrites the machine position from the work position.

edge cycle_homing.c:318-320 ends a G28.4 (home without setting coordinates) with
`cm_set_position(axis, cm_get_work_position(RUNTIME, axis))`: the machine position becomes the work
position, wrong whenever a G92 or tool offset is active. The other half of the finding (the clear
move reads sw.s[-1] when the opposite switch is disabled, :270-283) is harmless with this link
layout and is not observable over serial.

Bench: a software X-min switch mounted 3 mm below the start (test-hooks build). G28.2 defines machine
zero 1 mm (zb) above it. After G0 X5 and G92 X2 (origin offset 3), G28.4 homes to the same point;
the machine position there must still read 0, not the work position -3.
"""
import pytest

pytestmark = [pytest.mark.ice, pytest.mark.motion]


@pytest.mark.profile("switches-no", "homing-x")
@pytest.mark.bug("E055", on=["d440"])
def test_g28_4_keeps_machine_coordinates(tg, switches, record):
    switches.mount("x", "min", at=-3.0)
    try:
        tg.send_gcode("G28.2 X0", check=True, allow="home X onto a software switch 3 mm away")
        tg.wait_idle(timeout=60)
        assert tg.get("homx") == 1
        homed_at = tg.get("mpox")
        tg.send_gcode("G90 G0 X5", check=True, allow="5 mm X move")
        tg.wait_idle()
        tg.send_gcode("G92 X2", check=True)
        tg.send_gcode("G28.4 X0", check=True, allow="re-home X without setting coordinates")
        tg.wait_idle(timeout=60)
        record("positions", {"after_g28_2": homed_at, "after_g28_4": tg.get("mpox"),
                             "physical": switches.physical_position("x"),
                             "mount": switches.mount_status("x", "min")})
        assert tg.get("mpox") == pytest.approx(0.0, abs=0.05)
    finally:
        tg.send_gcode("G92.1", check=True)
        switches.unmount_all()
