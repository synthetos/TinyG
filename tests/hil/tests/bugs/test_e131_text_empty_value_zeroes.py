"""E131: in text mode an empty, non-numeric or trailing-space value ('$xvm=', '$xvm ', '$xvm=abc') sets 0 and persists it.

text_parser.c:129 splits on ' =:|\\t', then text_parser.c:141-148 runs strtof/atol with no end-pointer
check and text_parser.c:94-96 sets and persists. Master kept these as a display (master text_parser.c:128-129).
"""
import pytest


@pytest.mark.persists
@pytest.mark.parametrize("line", ["$xvm=", "$xvm ", "$xvm=abc"])
def test_text_set_without_number_keeps_value(tg, keep, line):
    keep("xvm")
    before = tg.get("xvm")
    tg.send_text(line)
    assert tg.get("xvm") == pytest.approx(before)


@pytest.mark.persists
@pytest.mark.dialect("d449")
@pytest.mark.parametrize("token,value", [
    ("xvm", "12oops"), ("xvm", "nan"), ("xvm", "inf"),
    ("1mi", "abc"), ("1mi", "8oops"), ("1mi", "4294967296"),
])
def test_malformed_text_set_reports_error_without_change(tg, keep, token, value):
    keep(token)
    before = tg.get(token)
    lines = tg.send_text(f"${token}={value}")
    assert any("err:" in line for line in lines), lines
    assert tg.get(token) == pytest.approx(before)
