"""E061: G38.2 silently turns the spindle off and never restores it.

edge cycle_probing.c:176 _probing_init() calls cm_spindle_control(SPINDLE_OFF), and the probe's exit
never restores it: a pick-and-place vacuum (wired to the spindle output) drops its part, and a CNC
user's spindle stops without being told. g2core (cycle_probing.cpp: 'Spindle and coolant are not
affected during probing') and the markmaker PnP fork leave the spindle alone.

Bench: Z-min is the probe, a software switch mounted 0.5 mm down (test-hooks build, --ice), so the
probe succeeds (a missed probe alarms, and an alarm stops the spindle by design). The spindle
enable output is read from the pin.
"""
import time

import pytest

pytestmark = [pytest.mark.ice, pytest.mark.motion, pytest.mark.dialect("d449")]


@pytest.mark.profile("switches-no", "probe-z")
@pytest.mark.bug("E061", on=[])
def test_successful_probe_leaves_spindle_on(tg, ice, switches):
    tg.send_gcode("M3 S1000", check=True, allow="spindle output on a bare bench")
    tg.wait_idle()
    on = ice.output("spindle_enable")
    switches.mount("z", "min", at=-0.5)
    try:
        tg.send_gcode("G38.2 Z-2 F60", check=True, allow="probe 0.5 mm down onto a software probe")
        prb = tg.wait_for(lambda e: e.kind in ("probe", "response") and isinstance(e.body, dict) and "prb" in e.body,
                          15).body["prb"]
        tg.wait_idle()
        assert prb["e"] == 1, "probe did not trip on the mounted switch"
        assert ice.output("spindle_enable") == on, "probing turned the spindle off"
    finally:
        switches.unmount_all()
        tg.send_gcode("M5", check=True)
        tg.send_gcode("G91 G0 Z0.5", allow="back up")
        tg.wait_idle()
        tg.send_gcode("G90", check=True)
