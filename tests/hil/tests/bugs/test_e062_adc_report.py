"""E062: raw signed ADCB/PB3 report. Analog accuracy needs a known voltage input."""
import pytest

def _raw(value):
    assert isinstance(value, (int, float)) and value == int(value)
    assert -2048 <= value <= 2047

@pytest.mark.bug("E062", on=["d440"])
def test_adc3_query_is_read_only(tg):
    _raw(tg.get("adc3"))
    assert tg.send_json({"adc3": 7}).status != 0

@pytest.mark.bug("E062", on=["d440"])
def test_m105_has_json_adc_value(tg):
    response = tg.send_gcode("M105")
    assert response.status == 0
    _raw(response.body["adc3"])
    assert not any(line.startswith("ADC3:") for line in response.text())
