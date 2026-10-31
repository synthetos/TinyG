"""Layout-2 E060 placeholders; retire these checks when the gated runtime feature lands."""
import pytest

@pytest.mark.dialect("d449")
@pytest.mark.parametrize("token,value", [("spph", 1), ("spde", 1.0), ("coph", 0)])
def test_hold_placeholders_report_stubbed(tg, token, value):
    assert tg.send_json({token: None}).status == 17
    assert tg.send_json({token: value}).status == 17
