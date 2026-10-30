"""E031: a limit switch hit is a soft jerk-limited feedhold raised late from the main loop.

edge switch.c:223-225 only flags the limit (controller_assert_limit_condition); controller.c:376-394
_limit_switch_handler later calls cm_alarm(), which requests an ordinary feedhold, so the machine
brakes at the move's own jerk and overruns the switch by the full normal stopping distance, and the
axes stay marked homed. Agreed behaviour (maintainer, 2026-10-30): a fast stop that keeps position -
hold requested from the switch ISR, braking at the homing jerk - plus ALARM and the axes unhomed.

Bench: a software X-max limit switch mounted 8 mm out (test-hooks build, --ice); stop distance is
measured from the firmware's physical step count. This checks the planned stopping distance; the
real-time latency of the stop is timing-pending (scope/Saleae).
"""
import time

import pytest

pytestmark = [pytest.mark.ice, pytest.mark.motion, pytest.mark.dialect("d449")]

FEED = 600.0            # mm/min: cruise is reached long before the switch at these jerks


def _stopped(switches, timeout=10.0):
    last, end = None, time.monotonic() + timeout
    while time.monotonic() < end:
        pos = switches.physical_position("x")
        if last is not None and abs(pos - last) < 1e-4:
            return pos
        last = pos
        time.sleep(0.15)
    raise AssertionError("X did not stop")


@pytest.mark.profile("switches-no")
@pytest.mark.settings(xsx=2, xsn=0)
@pytest.mark.bug("E031", on=[])
def test_limit_hit_stops_fast_and_unhomes(tg, switches, keep, record):
    keep("xjh")
    xjm = tg.get("xjm")
    tg.set("xjh", xjm * 9)                          # fast stop brakes in 1/3 of the normal distance
    tg.send_gcode("G28.3 X0", check=True)           # mark X homed
    assert tg.get("homx") == 1
    switches.mount("x", "max", at=8.0)
    seen = []
    try:
        tg.send_gcode(f"G91 G1 X20 F{FEED:.0f}", check=True, allow="X runs into a software limit switch at 8 mm")
        stop = _stopped(switches)
        seen += tg.drain(quiet=0.3)
        status = switches.mount_status("x", "max")
        v = FEED / 60.0                              # mm/s
        jerk = xjm * 1e6 / 216000.0                  # xjm is mm/min^3 / 1e6 -> mm/s^3
        normal = v * (v / jerk) ** 0.5               # jerk-limited stopping distance at the move's jerk
        overrun = stop - status["trip_at"]
        record("stop", {"trip_at": status["trip_at"], "stop": stop, "overrun": overrun, "normal": normal,
                        "xjm": xjm})
        limit = tg.dialect.status("LIMIT_SWITCH_HIT")
        assert any(e.kind == "exception" and e.data["er"].get("st") == limit for e in seen)
        assert tg.stat() == tg.dialect.stat("ALARM")
        assert tg.get("homx") == 0, "axes stay homed after a limit hit"
        assert overrun < 0.6 * normal, f"overran {overrun:.3f} mm; a normal stop is {normal:.3f} mm"
    finally:
        tg.clear_alarm()
        switches.unmount_all()
        tg.send_gcode("G91 G1 X-20 F1200", allow="back off the limit")
        tg.wait_idle(timeout=30)
        tg.send_gcode("G90", check=True)
