"""E017: a full circle written with an explicit endpoint equal to the start works for G2 but alarms for G3.

edge plan_arc.c:309 gives angular_travel == 0; the CW fix-up at :313 adds 2*pi, the CCW one at :315
leaves 0, and :328-329 raises ALARM 159 "arc has no movement". Master computes 0 travel for both
directions and returns STAT_MINIMUM_LENGTH_MOVE (201) without moving (master plan_arc.c:166-167, :285-286).
"""
import time

import pytest

MOVE = "full-circle arc test (radius 1 mm, returns to start)"


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


def _full_circle(tg, word, plane="G17", helix=False):
    start = _mpo(tg)
    try:
        axes, offset, normal = {"G17": ("XY", "I", "Z"), "G18": ("XZ", "I", "Y"),
                                "G19": ("YZ", "J", "X")}[plane]
        linear = f" {normal}0.2" if helix else ""
        words = f"{axes[0]}0 {axes[1]}0 {offset}-1{linear}"
        assert tg.send_gcode(f"{plane} G91 {word} {words} F120", allow=MOVE).status == 0
        token = "mpo" + axes[0].lower()
        span = _track(tg, [token])
        tg.wait_idle()
        assert tg.stat() != tg.dialect.stat("ALARM")
        assert span[token][0] == pytest.approx(start[axes[0].lower()] - 2, abs=0.1)
        for axis in "xyz":
            expected = start[axis] + (0.2 if helix and axis == normal.lower() else 0)
            assert tg.get("mpo" + axis) == pytest.approx(expected, abs=0.01)
    finally:
        _return_to(tg, start)


@pytest.mark.motion
@pytest.mark.bug("E017", on=["d440"])
@pytest.mark.parametrize("plane", ["G17", "G18", "G19"])
def test_g3_full_circle_with_endpoint(tg, plane):
    _full_circle(tg, "G3", plane)


@pytest.mark.motion
@pytest.mark.dialect("d449")
@pytest.mark.parametrize("plane", ["G17", "G18", "G19"])
def test_g2_full_circle_with_endpoint(tg, plane):
    _full_circle(tg, "G2", plane)


@pytest.mark.motion
@pytest.mark.bug("E017", on=["d440"])
def test_g3_helical_full_circle_with_endpoint(tg):
    _full_circle(tg, "G3", helix=True)
