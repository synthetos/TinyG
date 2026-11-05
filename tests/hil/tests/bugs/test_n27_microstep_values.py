"""N27: microstep configuration must match the onboard driver pins. TinyG configuration wiki 1MI lists 1,2,4,8. This test requires unsupported external-driver values to be refused rather than warned and narrowed; invalid settings must not change scaling, reset the stepper or touch pins."""
import pytest
pytestmark=[pytest.mark.persists,pytest.mark.dialect("d449"),pytest.mark.bug("N27",on=[])]
def test_microsteps_refuse_unsupported_values(tg,keep):
    keep("4mi")
    tg.send_json({"4mi":8},check=True)
    for value in (2147483648,256,16,3):
        reply=tg.send_json({"4mi":value})
        assert reply.status==tg.dialect.status("INPUT_VALUE_RANGE_ERROR"),reply.raw
        assert tg.get("4mi")==8
