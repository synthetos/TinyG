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


def _full_circle(tg, word):
    start = _mpo(tg)
    try:
        assert tg.send_gcode(f"G17 G91 {word} X0 Y0 I-1 F600", allow=MOVE).status == 0
        span = _track(tg, ["mpox"])
        assert tg.stat() != tg.dialect.stat("ALARM")
        assert span["mpox"][0] == pytest.approx(start["x"] - 2, abs=0.1)     # reached the far side
        assert tg.get("mpox") == pytest.approx(start["x"], abs=0.01)
        assert tg.get("mpoy") == pytest.approx(start["y"], abs=0.01)
    finally:
        _return_to(tg, start)


@pytest.mark.motion
@pytest.mark.bug("E017", on=["d440", "d449"])
def test_g3_full_circle_with_endpoint(tg):
    _full_circle(tg, "G3")


@pytest.mark.motion
@pytest.mark.dialect("d449")
def test_g2_full_circle_with_endpoint(tg):
    _full_circle(tg, "G2")
