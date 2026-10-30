"""Reference path simulator checks; no board needed."""
import math

import pytest

from tinyg_hil import gsim


def _extent(points, axis):
    vals = [p[axis] for p in points]
    return min(vals), max(vals)


def test_line_absolute_and_relative():
    m = gsim.simulate(["G21 G90 G1 X10 Y5 F100", "G91 G1 X-2"])
    assert m.position["X"] == pytest.approx(8) and m.position["Y"] == pytest.approx(5)


def test_inches():
    m = gsim.simulate(["G20 G90 G0 X1"])
    assert m.position["X"] == pytest.approx(25.4)


def test_g17_cw_quarter_circle():
    m = gsim.simulate(["G17 G90 G2 X1 Y-1 I0 J-1 F100"], start={"X": 0, "Y": 0})
    pts = gsim.path(m)
    assert _extent(pts, "X") == pytest.approx((0, 1), abs=1e-3)
    for p in pts:     # all points on the circle centred at (0,-1)
        assert math.hypot(p["X"], p["Y"] + 1) == pytest.approx(1, abs=1e-6)


@pytest.mark.parametrize("r, ymax", [(1.5, 0.382), (-1.5, 2.618)])
def test_r_arc_short_and_long(r, ymax):
    m = gsim.simulate([f"G17 G90 G2 X2 Y0 R{r} F100"])
    assert _extent(gsim.path(m), "Y")[1] == pytest.approx(ymax, abs=2e-3)


def test_g18_cw_follows_nist_direction():
    # E029: viewed from +Y, G2 in the ZX plane from X0 to X2 with centre (X1, Z0.75) dips to Z-0.5
    m = gsim.simulate(["G18 G91 G2 X2 Z0 I1 K0.75 F120"])
    lo, hi = _extent(gsim.path(m), "Z")
    assert lo == pytest.approx(-0.5, abs=2e-3) and hi == pytest.approx(0, abs=1e-6)


def test_full_circle():
    m = gsim.simulate(["G17 G91 G3 X0 Y0 I-1 F100"])
    assert _extent(gsim.path(m), "X") == pytest.approx((-2, 0), abs=2e-3)


def test_g92_and_g10_l20():
    m = gsim.simulate(["G28.3 X3", "G92 X0", "G0 X1"])
    assert m.position["X"] == pytest.approx(4) and m.work("X") == pytest.approx(1)
    m = gsim.simulate(["G28.3 X3", "G10 L20 P1 X0", "G90 G0 X1"])
    assert m.position["X"] == pytest.approx(4)
