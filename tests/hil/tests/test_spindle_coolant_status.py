"""Spindle and coolant state readouts, named as in g2core: spe, spd, sps, cof, com.

g2core (wiki: edge-branch changelog, "Spindle Changes"/"Coolant Changes") reports spindle enable {spe:n},
direction {spd:n} (0 = CW, 1 = CCW), speed {sps:n}, and flood/mist coolant {cof:n} / {com:n}, all usable in
status reports. edge-next reports the same names, read-only, so a host written for g2core reads TinyG the
same way. They show the state the machine is in now: M3/M4/M5, S and M7/M8/M9 are queued with the moves
and take effect when the planner reaches them, so a command behind a move is not reported until it runs.
sps is the speed in use, after the p1csl..p1csh (CW) or p1wsl..p1wsh (CCW) clamp. TinyG has no spindle
speed sensor: sps is the commanded speed, not a measurement.
"""
import pytest

pytestmark = [pytest.mark.dialect("d449"), pytest.mark.motion]


def state(tg):
    return {k: tg.get(k) for k in ("spe", "spd", "sps", "cof", "com")}


def clamp(tg, s, ccw=False):
    lo, hi = (tg.get("p1wsl"), tg.get("p1wsh")) if ccw else (tg.get("p1csl"), tg.get("p1csh"))
    return min(max(s, lo), hi)


@pytest.fixture
def outputs_off(tg):
    yield
    tg.send_gcode("M5 M9", allow="spindle and coolant off")
    tg.wait_idle()


def test_spindle_and_coolant_state(tg, outputs_off):
    tg.send_gcode("M5 M9", check=True, allow="known start")
    tg.wait_idle()
    st = state(tg)
    assert (st["spe"], st["cof"], st["com"]) == (0, 0, 0), st

    tg.send_gcode("M3 S5000", check=True, allow="spindle CW")
    tg.wait_idle()
    st = state(tg)
    assert (st["spe"], st["spd"]) == (1, 0), st
    assert st["sps"] == pytest.approx(clamp(tg, 5000), abs=0.5), st

    tg.send_gcode("M4 S7000", check=True, allow="spindle CCW")
    tg.wait_idle()
    st = state(tg)
    assert (st["spe"], st["spd"]) == (1, 1), st
    assert st["sps"] == pytest.approx(clamp(tg, 7000, ccw=True), abs=0.5), st

    tg.send_gcode("M8", check=True, allow="flood")
    tg.wait_idle()
    assert (tg.get("cof"), tg.get("com")) == (1, 0)
    tg.send_gcode("M7", check=True, allow="mist")
    tg.wait_idle()
    assert (tg.get("cof"), tg.get("com")) == (1, 1)

    tg.send_gcode("M5 M9", check=True, allow="off")
    tg.wait_idle()
    st = state(tg)
    assert (st["spe"], st["cof"], st["com"]) == (0, 0, 0), st


def test_state_follows_execution_not_the_queue(tg, outputs_off):
    """M3 behind a move is reported only once the move ends and the M3 runs."""
    tg.send_gcode("M5", check=True, allow="known start")
    tg.send_gcode("G90", check=True)
    tg.send_gcode("G0 X0", check=True, allow="known start")
    tg.wait_idle()
    try:
        tg.send_gcode("G1 X4 F240", check=True, allow="1 s move")
        tg.send_gcode("M3 S5000", check=True, allow="queued behind the move")
        assert tg.get("spe") == 0, "M3 reported before it ran"
        tg.wait_idle()
        assert tg.get("spe") == 1
    finally:
        tg.send_gcode("M5", allow="off")
        tg.send_gcode("G0 X0", allow="return to start")
        tg.wait_idle()


def test_readouts_are_read_only(tg):
    for key in ("spe", "spd", "sps", "cof", "com"):
        before = tg.get(key)
        resp = tg.send_line('{"%s":1}' % key)
        assert resp.status != 0, f"{key} accepted a write: {resp.raw}"
        assert tg.get(key) == before
