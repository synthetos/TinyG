"""E070: G38.2 with the probe already tripped reports success at the current position.

edge cycle_probing.c:187-189 only moves `if (read_switch(pb.probe_switch) == SW_INACTIVE)`; when the
probe is already closed, _probing_finish (:199-205) records PROBE_SUCCEEDED and the current position,
so a stuck or shorted probe (or any NC probe) reports {"prb":{"e":1}} with no motion and the host
zeroes Z in the air. g2 exits with STAT_PROBE_IS_ALREADY_TRIPPED.

Bench: Z-min is the probe (profile probe-z) and is closed through the Atmel-ICE. Probe inputs are
forced normally-open (switch.c:127-128), so closed means driven low whatever $st says.
"""
import time

import pytest

from tinyg_hil.driver import NoResponse

pytestmark = [pytest.mark.ice, pytest.mark.motion, pytest.mark.dialect("d449")]


def _body(ev):
    return ev.body if isinstance(ev.body, dict) else {}


def get(tg, token, timeout=2.0):
    """GET that skips unrelated {"r":...} lines such as the asynchronous {"prb":...} report."""
    tg.transport.write(('{"%s":null}\n' % token).encode())
    return tg.wait_for(lambda e: e.kind == "response" and token in _body(e), timeout).body[token]


def idle(tg, timeout=15.0):
    end = time.monotonic() + timeout
    while get(tg, "cycs") != 0 or get(tg, "qr") != tg.planner_buffers:
        if time.monotonic() > end:
            raise NoResponse(f"not idle after {timeout}s")
        time.sleep(0.1)


@pytest.mark.profile("probe-z")
@pytest.mark.bug("E070", on=[])
def test_g38_2_with_probe_already_closed_is_an_error(tg, ice, switches):
    ice.set_switch_level("z", "min", 0)             # probe contact closed before the cycle starts
    time.sleep(0.1)
    tg.send_gcode("G92 Z0", check=True)
    seen = []

    def prb_report(ev):
        seen.append(ev)
        return ev.kind in ("probe", "response") and "prb" in _body(ev)   # top-level since E101
    try:
        tg.send_gcode("G38.2 Z-1 F50", check=True, allow="probe with the contact already closed")
        assert tg.wait_for(prb_report, 10).body["prb"]["e"] == 0, "already-tripped probe reported success"
        seen += tg.drain(quiet=0.5)
        reported = any(e.kind == "exception" for e in seen)
        assert reported or get(tg, "stat") == tg.dialect.stat("ALARM")
    finally:
        ice.set_switch_level("z", "min", 1)
        idle(tg)
        tg.drain()
        tg.clear_alarm()
        tg.send_gcode("G92.1", check=True)
