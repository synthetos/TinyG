"""N28: status reports wait for elapsed time across the 32-bit tick wrap. TinyG configuration wiki SI/SV: si is the automatic-report interval; sv=0 disables reports. Intervals are bounded below half the tick range; ASAP and timed requests retain first-request-wins behaviour."""
import pytest
pytestmark=[pytest.mark.persists,pytest.mark.dialect("d449"),pytest.mark.bug("N28",on=["d449"])]
def test_status_interval_upper_bound(tg,keep):
    keep("si")
    tg.send_json({"si":250},check=True)
    reply=tg.send_json({"si":2147483648})
    assert reply.status==tg.dialect.status("INPUT_VALUE_RANGE_ERROR"),reply.raw
    assert tg.get("si")==250
