"""E025: G38.2 rejects a modal F, and the probe cycle starts before the previous move has finished.

- cycle_probing.c:71 `if (!cm.gf.feed_rate)` rejects G38.2 unless F is on the same line
  (gf is cleared per block); master only rejects a zero feed rate.
- cycle_probing.c:136-141 waits on cm_get_runtime_busy(), which is false while cycle_state is
  CYCLE_OFF (plan_line.c:69). G0/G1 never call cm_cycle_start() (canonical_machine.c:1253-1257),
  so _probing_init (cycle_probing.c:167, CYCLE_PROBE + homing jerk) runs while an earlier G1 is
  still moving, and switch.c:219-222 turns limit hits during that move into plain feedholds.

The stuck-CYCLE_PROBE path (cycle_probing.c:188, ritorno on STAT_MINIMUM_TIME_MOVE) depends on jerk
settings and float rounding, so it is not tested here.

Bench: the probe is Z-min with nothing connected. Edge forces probe inputs to NO (switch.c:127-128),
so the open input reads untripped and each G38.2 runs its full (1 mm) travel.
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


def back_to_start(tg):
    idle(tg)
    tg.drain()
    tg.send_gcode("G90 G1 X0 Y0 Z0 F600", check=True, allow="return to the G92 zero")
    idle(tg)


@pytest.fixture
def probe_zero(tg, keep):
    use_zmin_probe(tg, keep)
    tg.send_gcode("G92 X0 Y0 Z0", check=True)     # work zero here; M2 in the next test clears it
    yield
    back_to_start(tg)


@pytest.mark.bug("E025", on=["d449"])
def test_g38_2_uses_modal_feed_rate(tg, probe_zero):
    tg.send_gcode("F100", check=True)
    resp = tg.send_gcode("G38.2 Z-1", allow="1 mm probe in free air")
    assert resp.status == 0, tg.status_name(resp.status)
    assert wait_prb(tg)["e"] == 0


def test_g38_2_with_feed_rate_on_line(tg, probe_zero):
    tg.send_gcode("G38.2 Z-1 F100", check=True, allow="1 mm probe in free air")
    assert wait_prb(tg)["e"] == 0


@pytest.mark.bug("E025", on=["d449"])
def test_probe_cycle_waits_for_previous_move(tg, probe_zero):
    x0 = get(tg, "mpox")
    tg.send_gcode("G1 X5 F100", check=True, allow="5 mm move (3 s) ahead of the probe")
    tg.send_gcode("G38.2 Z-1 F100", check=True, allow="1 mm probe in free air")
    cycs = get(tg, "cycs")
    assert get(tg, "mpox") < x0 + 4.5       # the G1 is still running
    assert cycs != 3                        # CYCLE_PROBE must not start yet
    wait_prb(tg, timeout=15)
