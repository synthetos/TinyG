"""N12c: refuse an inverse feed needing more than the segment counter can hold.

NIST RS274NGC v3 3.5.19 defines G93 duration as 1/F minutes. The implementation
must report a range error before scheduling an unrepresentable duration.
"""
import pytest
pytestmark=[pytest.mark.motion,pytest.mark.resets_board,pytest.mark.dialect("d449"),pytest.mark.bug("N12c",on=["d449"])]
def test_inverse_feed_duration_refused(tg):
    tg.send_gcode("G21 G91 G93",check=True)
    try:
        r=tg.send_line("G1 X0.01 F0.0000001")
        assert r.status==tg.dialect.status("INPUT_VALUE_RANGE_ERROR"),"unrepresentable inverse duration accepted"
        assert tg.get("cycs")==0
    finally:
        tg.reset();tg.send_gcode("G21 G90 G94",check=True)
