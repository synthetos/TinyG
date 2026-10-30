"""E029: G18 (XZ) arcs turn the wrong way compared with RS274NGC, LinuxCNC and grbl.

edge plan_arc.c:154-157 uses plane_axis_0=X, plane_axis_1=Z. With theta=atan2(-off0,-off1) (:305) and
sin/cos on axis 0/1 (:91-92), CW rotates about X x Z = -Y, i.e. clockwise seen from -Y. The standard
order is (Z, X). Master (plan_arc.c:109-112, :274) gives the same path (issues #141, #201).

For `G2 X2 Z0 I1 K0.75` (centre above the chord) the standard short arc dips to Z-0.5;
edge runs the long arc over the top and peaks at Z+2.0.
"""
import time

import pytest

MOVE = "G18 arc direction test (2 mm X, <=2 mm Z, returns to start)"


def _mpo(tg):
    return {a: tg.get(f"mpo{a}") for a in "xyz"}


def _return_to(tg, start):
    if tg.stat() == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    tg.send_gcode("G90 G53 G0 " + " ".join(f"{a.upper()}{v:.4f}" for a, v in start.items()), allow=MOVE)
    tg.wait_idle()


def _track(tg, tokens, timeout=20.0):
    """Poll tokens until the machine is idle again; return {token: (min, max)}."""
    seen = {t: [] for t in tokens}
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        for t in tokens:
            seen[t].append(tg.get(t))
        if tg.stat() in tg.dialect.idle_stats and tg.get("qr") == tg.planner_buffers:
            return {t: (min(v), max(v)) for t, v in seen.items()}
    raise TimeoutError(f"not idle after {timeout}s")


@pytest.mark.motion
@pytest.mark.bug("E029", on=["d440", "d449"])
def test_g18_cw_arc_takes_short_path_below(tg):
    start = _mpo(tg)
    try:
        assert tg.send_gcode("G18 G91 G2 X2 Z0 I1 K0.75 F120", allow=MOVE).status == 0
        low, high = _track(tg, ["mpoz"])["mpoz"]
        assert tg.get("mpox") == pytest.approx(start["x"] + 2, abs=0.01)
        assert high < start["z"] + 0.2
        assert low < start["z"] - 0.3
    finally:
        tg.send_gcode("G17", check=True)
        _return_to(tg, start)
