"""N25: rotary radius must keep the linear-to-degree scale finite and nonzero. TinyG configuration wiki aRA/Radius mode: a circumference maps to 360 degrees. Limit radius to 1000000 mm (1 km); even one mm then exceeds the planner numeric-zero tolerance in degrees. Validate after inch conversion and before storage."""
import pytest
pytestmark=[pytest.mark.persists,pytest.mark.dialect("d449"),pytest.mark.bug("N25",on=["d449"])]
def test_radius_upper_bound(tg,keep):
    keep("ara","bra","cra")
    tg.send_gcode("G21",check=True)
    for token in ("ara","bra","cra"):
        before=tg.get(token)
        reply=tg.send_json({token:1e38})
        assert reply.status==tg.dialect.status("INPUT_VALUE_RANGE_ERROR"),reply.raw
        assert tg.get(token)==pytest.approx(before)
