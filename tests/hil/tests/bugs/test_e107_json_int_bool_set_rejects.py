"""E107: JSON SETs of integer tokens reject 8.0 and true/false (116) and wrap negatives ({"si":-1}).

json_parser.c:445-448 accepts only a JSON integer for TYPE_INTEGER rows; util.c:244 rejects any '.';
util.c:246 strtol into a uint32_t wraps -1 to 4294967295, which passes report.c:570.
Master parsed every number with strtod and accepted true/false (master json_parser.c:258-262, 290-296).
"""
import pytest

pytestmark = pytest.mark.persists


@pytest.mark.bug("E107", on=[])
def test_integral_float_accepted(tg, keep):
    keep("1mi")
    value = tg.get("1mi")
    assert tg.send_json({"1mi": float(value)}).status == 0
    assert tg.get("1mi") == value


@pytest.mark.bug("E107", on=[])
def test_boolean_accepted_for_integer_token(tg, keep):
    keep("sl")
    value = tg.get("sl")
    assert tg.send_json({"sl": bool(value)}).status == 0
    assert tg.get("sl") == value


@pytest.mark.bug("E107", on=[])
def test_negative_interval_not_wrapped(tg, keep):
    keep("si")
    resp = tg.send_json({"si": -1})
    assert resp.status != 0 or 100 <= tg.get("si") < 2 ** 31


@pytest.mark.dialect("d449")
def test_unknown_test_number_reaches_test_dispatch(tg):
    # 99 is not a motion program; reject it in run_test, not as a boolean value.
    response = tg.send_json({"test": 99})
    assert response.status == tg.dialect.status("ERROR")
