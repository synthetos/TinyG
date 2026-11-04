"""N26: validate integers before narrowing their stored width. TinyG configuration wiki SL defines soft-limit enable as 0 or 1. Large integer values must not silently disable limits or change another byte setting."""
import pytest
pytestmark=[pytest.mark.persists,pytest.mark.dialect("d449"),pytest.mark.bug("N26",on=["d449"])]
def test_soft_limit_integer_does_not_wrap(tg,keep):
    keep("sl")
    tg.send_json({"sl":1},check=True)
    for value in (2147483648,256,2):
        reply=tg.send_json({"sl":value})
        assert reply.status==tg.dialect.status("INPUT_VALUE_RANGE_ERROR"),reply.raw
        assert tg.get("sl")==1
