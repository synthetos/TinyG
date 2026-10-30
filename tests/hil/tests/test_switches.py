"""Software switches through the Atmel-ICE (pass --ice): the firmware must see the simulated inputs."""
import time

import pytest

pytestmark = pytest.mark.ice


def _input(tg, n):
    """Firmware input state via text mode ($inN), which is valid even where the JSON form is not (E081)."""
    for line in tg.send_text(f"$in{n}"):
        if line.startswith(f"[in{n}]"):
            return int(float(line.split()[-1]))
    raise AssertionError(f"no $in{n} line")


def test_close_and_open_x_min_is_seen_by_firmware(tg, switches):
    before = _input(tg, 1)
    switches.close("x", "min")
    time.sleep(0.1)                         # past the switch lockout
    during = _input(tg, 1)
    switches.open("x", "min")
    time.sleep(0.1)
    after = _input(tg, 1)
    assert (before, during, after) == (0, 1, 0)


def test_mounted_switch_follows_position(tg, switches):
    switches.mount("x", "max", at=0.5)
    tg.send_gcode("G91 G1 X1 F120", check=True, allow="1 mm X move across a software switch")
    tg.wait_idle()
    tripped = [t for t in switches.trips if t[1:4] == ("x", "max", True)]
    assert tripped, "mounted switch never closed"
    assert tripped[0][4] == pytest.approx(0.5, abs=0.1)
    tg.send_gcode("G91 G1 X-1 F600", allow="move back")
    tg.wait_idle()
