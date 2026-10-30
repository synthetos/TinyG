"""E008: every radius-format arc (G2/G3 ... R) raises ALARM 157 "arc offsets missing or zero" on edge.

edge plan_arc.c:237-241 alarms when I/J/K are all zero, which is always true in R mode, before
_compute_arc() derives the offsets from the radius at :281-282. Master has no such check.
"""
import pytest

MOVE = "R-format arc test (2 mm, relative, returns to start)"


def _mpo(tg):
    return {a: tg.get(f"mpo{a}") for a in "xyz"}


def _return_to(tg, start):
    if tg.stat() == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    tg.send_gcode("G90 G53 G0 " + " ".join(f"{a.upper()}{v:.4f}" for a, v in start.items()), allow=MOVE)
    tg.wait_idle()


@pytest.mark.motion
@pytest.mark.bug("E008", on=["d449"])
def test_radius_arcs_run(tg):
    start = _mpo(tg)
    try:
        assert tg.send_gcode("G17 G91 G2 X2 Y0 R1.5 F600", allow=MOVE).status == 0
        tg.wait_idle()
        assert tg.get("mpox") == pytest.approx(start["x"] + 2, abs=0.01)
        assert tg.get("mpoy") == pytest.approx(start["y"], abs=0.01)
        assert tg.send_gcode("G91 G3 X-2 Y0 R1.5 F600", allow=MOVE).status == 0
        tg.wait_idle()
        assert tg.get("mpox") == pytest.approx(start["x"], abs=0.01)
    finally:
        _return_to(tg, start)
