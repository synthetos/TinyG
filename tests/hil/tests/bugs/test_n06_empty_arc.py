"""N06: plan_arc.c starts a cycle, but mp_aline discards the sole tiny chord.

NIST RS274NGC v3 3.5.3 allows center-format full circles. Under TinyG's chordal
approximation a sub-resolution circle must complete, not leave RUN with no work.
"""
import pytest

pytestmark=[pytest.mark.motion, pytest.mark.resets_board, pytest.mark.persists,
            pytest.mark.dialect("d449"), pytest.mark.bug("N06", on=[])]

def test_subresolution_circle_finishes(tg, keep):
    keep("ct")
    tg.set("ct", .01)
    try:
        tg.send_gcode("G21 G17 G91 G91.1 G94", check=True)
        tg.send_gcode("G2 X0 Y0 I0.0001 J0 F100", check=True,
                      allow="full circle of radius 0.0001 mm")
        tg.wait_idle(timeout=3)
        assert tg.get("qr")==32
    finally:
        tg.reset()
