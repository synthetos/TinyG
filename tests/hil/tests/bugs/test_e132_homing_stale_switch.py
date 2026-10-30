"""E132: homing accepts a switch that never changes state.

With NC switches ($st=1, the edge default) and pull-ups, an unconnected input reads active
(switch.c:177). The state is sampled once in reset_switches (switch.c:133) and afterwards changes only
on ISR edges. The clear move (cycle_homing.c:281-282) backs off without re-reading the input, the
search runs its full travel, and the only arrival check is the stale level at cycle_homing.c:300,
so cycle_homing.c:315-317 sets the position to 0 and marks the axis homed.

Bench: nothing is connected to X-min. Travel and backoffs are shrunk so the whole cycle moves ~5 mm.
"""
import time

import pytest

from tinyg_hil.driver import NoResponse

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449")]

SMALL_HOMING = {"xsx": 0, "xtn": 0, "xtm": 2, "xlb": 1, "xzb": 0.5, "xsv": 300, "xlv": 100}


def _body(ev):
    return ev.body if isinstance(ev.body, dict) else {}


def get(tg, token, timeout=2.0):
    """GET that skips unrelated {"r":...} lines such as a homing error message."""
    tg.transport.write(('{"%s":null}\n' % token).encode())
    return tg.wait_for(lambda e: e.kind == "response" and token in _body(e), timeout).body[token]


def home_x(tg, keep, switch_type):
    keep("st", "xsn", *SMALL_HOMING)
    for token, value in {"st": switch_type, "xsn": 1, **SMALL_HOMING}.items():
        tg.set(token, value)
    tg.send_gcode("G28.2 X0", check=True, allow="homing X over ~5 mm")
    end = time.monotonic() + 30
    while get(tg, "cycs") != 0:
        if time.monotonic() > end:
            raise NoResponse("homing cycle did not end")
        time.sleep(0.2)
    tg.drain()


@pytest.mark.bug("E132", on=["d449"])
def test_open_nc_switch_does_not_home(tg, keep):
    home_x(tg, keep, switch_type=1)
    assert get(tg, "homx") == 0


def test_open_no_switch_does_not_home(tg, keep):
    home_x(tg, keep, switch_type=0)
    assert get(tg, "homx") == 0
