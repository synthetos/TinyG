"""N29: bound junction deviations before squaring them. TinyG configuration wiki xJD and JA define the controlling corner circle. Accept 0.00001 through 1000000 canonical units, keep ordinary junction results unchanged, and prevent an extreme acceleration product from returning infinity."""
import pytest
pytestmark=[pytest.mark.persists,pytest.mark.dialect("d449"),pytest.mark.bug("N29",on=["d449"])]
def test_junction_deviation_upper_bound(tg,keep):
    keep("xjd","yjd","zjd","ajd","bjd","cjd")
    tg.send_gcode("G21",check=True)
    for token in ("xjd","yjd","zjd","ajd","bjd","cjd"):
        before=tg.get(token)
        reply=tg.send_json({token:1e30})
        assert reply.status==tg.dialect.status("INPUT_VALUE_RANGE_ERROR"),reply.raw
        assert tg.get(token)==pytest.approx(before)
