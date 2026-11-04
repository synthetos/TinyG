"""N19: chordal tolerance is a positive geometric error bound. TinyG wiki TinyG-Configuration-for-Firmware-Version-0.97, CT; NIST RS274NGC v3 section 3.5.3 permits full circles. Negative zero must not discard a circle (plan_arc.c segment calculation)."""
import pytest
pytestmark=[pytest.mark.persists,pytest.mark.dialect("d449"),pytest.mark.bug("N19",on=[])]
def test_nonpositive_tolerance_refused(tg,keep):
    keep("ct")
    before=tg.get("ct")
    for value in (-1,0,-0.0):
        reply=tg.send_json({"ct":value})
        assert reply.status==tg.dialect.status("INPUT_VALUE_RANGE_ERROR"),reply.raw
        assert tg.get("ct")==pytest.approx(before)
