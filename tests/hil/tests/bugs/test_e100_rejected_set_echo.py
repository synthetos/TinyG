"""E100: a rejected numeric SET echoes the reset sentinel (4294967295, or a NULL string) as the value.

_marshall_nv_pair resets the nv (json_parser.c:393, config.c:655,659) and returns from str2long
(json_parser.c:445-447, util.c:244) or the string check (json_parser.c:439-443) before a value is
written, and json_print_response still echoes the config token at jv>=3 (json_parser.c:668-671).
Master has no integer config type and parses these inputs as values, so these are d449-only.
"""
import pytest

pytestmark = [pytest.mark.dialect("d449"), pytest.mark.persists]


@pytest.mark.bug("E100", on=[])
@pytest.mark.parametrize("line, path", [
    ('{"1mi":8.5}', ("1mi",)),          # non-integral float to an integer token
    ('{"1":{"mi":8.5}}', ("1", "mi")),  # the same through the group form UGS sends
    ('{"xvm":"100"}', ("xvm",)),        # string to a float token
])
def test_rejected_set_echoes_null(tg, keep, line, path):
    keep("1mi", "xvm")
    resp = tg.send_json(line)
    echo = resp.body
    for key in path:
        echo = echo.get(key) if isinstance(echo, dict) else None
    assert resp.status != 0
    assert echo is None


def test_accepted_integer_set_echoes_value(tg, keep):
    keep("1mi")
    value = tg.get("1mi")
    resp = tg.send_json({"1mi": value}, check=True)
    assert resp.body["1mi"] == value
