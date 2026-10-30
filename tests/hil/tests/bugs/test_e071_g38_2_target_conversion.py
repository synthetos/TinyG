"""E071: the G38.2 target is not resolved at command time.

cycle_probing.c:93 stores the raw word values (`copy_vector(pb.target, target)`), then
cycle_probing.c:174-175 forces ABSOLUTE_MODE before cm_straight_feed(pb.target) at :188, so
`G91 G38.2 Z-2` probes to work Z=-2 instead of 2 mm down. The minimum-travel check at :99-107 sums
all of X, Y and Z against targets of 0 for unflagged axes, so it is only right at work X=Y=0.

Bench: the probe is Z-min with nothing connected; edge forces probe inputs to NO
(switch.c:127-128), so the open input reads untripped and each probe runs its full travel.
Positions are compared as machine positions (mpo*), which are always mm.
"""
import time

import pytest

from tinyg_hil.driver import NoResponse

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449")]


def _body(ev):
    return ev.body if isinstance(ev.body, dict) else {}


def get(tg, token, timeout=2.0):
    """GET that skips unrelated {"r":...} lines such as the asynchronous {"prb":...} report."""
    tg.transport.write(('{"%s":null}\n' % token).encode())
    return tg.wait_for(lambda e: e.kind == "response" and token in _body(e), timeout).body[token]


def wait_prb(tg, timeout=10.0):
    return tg.wait_for(lambda e: e.kind == "response" and "prb" in _body(e), timeout).body["prb"]


def idle(tg, timeout=15.0):
    end = time.monotonic() + timeout
    while get(tg, "cycs") != 0 or get(tg, "qr") != tg.planner_buffers:
        if time.monotonic() > end:
            raise NoResponse(f"not idle after {timeout}s")
        time.sleep(0.1)


def use_zmin_probe(tg, keep):
    """Make Z-min the only probe input (find_probe_switch() errors on two)."""
    for token in ("xsn", "xsx", "ysn", "ysx", "zsx", "asn", "asx"):
        if tg.get(token) == 4:
            keep(token)
            tg.set(token, 0)
    keep("zsn")
    tg.set("zsn", 4)


@pytest.fixture
def probe_zero(tg, keep):
    use_zmin_probe(tg, keep)
    tg.send_gcode("G92 X0 Y0 Z0", check=True)     # work zero here; M2 in the next test clears it
    yield
    idle(tg)
    tg.drain()
    if tg.stat() == tg.dialect.stat("ALARM"):       # a probe in free air misses, which alarms since E072
        tg.clear_alarm()
    tg.send_gcode("G90 G1 X0 Y0 Z0 F600", check=True, allow="return to the G92 zero")
    idle(tg)


@pytest.mark.bug("E071", on=["d440"])
def test_incremental_probe_is_relative(tg, probe_zero):
    tg.send_gcode("G1 Z3 F600", check=True, allow="3 mm up")
    idle(tg)
    z0 = get(tg, "mpoz")
    tg.send_gcode("G91 G38.2 Z-2 F200", check=True, allow="2 mm probe down in free air")
    assert wait_prb(tg)["e"] == 0
    idle(tg)
    assert get(tg, "mpoz") == pytest.approx(z0 - 2, abs=0.01)     # buggy: z0 - 5 (work Z=-2)


@pytest.mark.bug("E071", on=[])
def test_travel_check_ignores_unflagged_axes(tg, probe_zero):
    tg.send_gcode("G92 X3", check=True)           # work X=3 without moving
    resp = tg.send_gcode("G38.2 Z-0.1 F100", allow="0.1 mm probe if the travel check misses")
    assert resp.status == tg.dialect.status("PROBE_TRAVEL_TOO_SMALL")


def test_travel_check_at_xy_zero(tg, probe_zero):
    resp = tg.send_gcode("G38.2 Z-0.1 F100", allow="rejected: below MINIMUM_PROBE_TRAVEL")
    assert resp.status == tg.dialect.status("PROBE_TRAVEL_TOO_SMALL")


@pytest.mark.bug("E071", on=["d440"])
def test_g38_2_in_inverse_time_mode_is_an_error(tg, keep):
    """RS274NGC v3 3.5.9.1: it is an error if G38.2 is used in inverse time feed rate mode."""
    use_zmin_probe(tg, keep)
    try:
        r = tg.send_gcode("G93 G38.2 Z-1 F10", allow="rejected before any motion")
        assert r.status == tg.dialect.status("GCODE_INVERSE_TIME_MODE_CANNOT_BE_USED")
    finally:
        tg.send_gcode("G94", check=True)


@pytest.mark.bug("E071", on=[])
def test_g20_short_probe_above_minimum_is_accepted(tg, probe_zero):
    """RS274NGC 3.5.9.1: error only if the move is < 0.254 mm or 0.01 inch. The check compared a
    current-unit distance (inches in G20) against 0.254, so 0.02 inch (0.508 mm) was rejected."""
    try:
        r = tg.send_gcode("G20 G91 G38.2 Z-0.02 F1", allow="0.5 mm probe in free air")
        assert r.status != tg.dialect.status("PROBE_TRAVEL_TOO_SMALL")
        wait_prb(tg, 60)
    finally:
        idle(tg, 60)
        tg.drain()
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
        tg.send_gcode("G21 G90", check=True)
