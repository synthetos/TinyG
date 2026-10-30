"""Step counts and pulse widths measured by the chip's own timer (pass --ice), across motor geometries."""
import time

import pytest

pytestmark = [pytest.mark.ice, pytest.mark.motion]
MOVE = "short X move for step counting"


def _x_motor(machine):
    motors = machine.motors_for("x")
    if not motors:
        pytest.skip("no motor mapped to X")
    return motors[0]


@pytest.mark.matrix("motor-geometry")
def test_steps_issued_match_steps_per_unit(tg, machine, steps):
    """1 mm of X must produce exactly 360/sa * mi / tr pulses on the X motor's step pin."""
    motor = _x_motor(machine)
    expected = round(1.0 * machine.steps_per_unit("x"))
    if expected > 60000:
        pytest.skip(f"{expected} steps would overflow the 16-bit counter")
    fw_before = steps.firmware_steps(motor)
    steps.arm(motor)
    tg.send_gcode("G91 G1 X1 F120", check=True, allow=MOVE)
    tg.wait_idle()
    time.sleep(0.1)
    counted = steps.count()
    fw_delta = steps.firmware_steps(motor) - fw_before
    tg.send_gcode("G91 G1 X-1 F600", allow=MOVE)
    tg.wait_idle()
    assert counted == expected, f"{machine.label}: {counted} pulses on the pin, expected {expected}"
    assert abs(fw_delta) == expected, f"{machine.label}: firmware counted {fw_delta}, expected {expected}"


@pytest.mark.parametrize("motor", [1, 2, 3, 4])
def test_step_pulse_width_meets_driver_minimum(tg, machine, steps, motor):
    """Every motor's step pulse must stay above the driver's minimum high time (1 us for DRV8818)."""
    axis = "xyzabc"[int(machine.get(f"{motor}ma"))]
    if tg.get(f"{axis}am") == 0:
        pytest.skip(f"motor {motor} drives {axis.upper()}, which is disabled ({axis}am=0)")
    steps.arm_pulse_width(motor)
    tg.send_gcode(f"G91 G1 {axis.upper()}0.5 F300", check=True, allow=MOVE)
    tg.wait_idle()
    width = steps.last_pulse_width_us()
    tg.send_gcode(f"G91 G1 {axis.upper()}-0.5 F600", allow=MOVE)
    tg.wait_idle()
    assert width > 0, "no pulse captured"
    assert width >= 1.0, f"motor {motor} step pulse {width:.2f} us is below 1 us"
