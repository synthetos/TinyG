"""E131: in text mode an empty, non-numeric or trailing-space value ('$xvm=', '$xvm ', '$xvm=abc') sets 0 and persists it.

text_parser.c:129 splits on ' =:|\\t', then text_parser.c:141-148 runs strtof/atol with no end-pointer
check and text_parser.c:94-96 sets and persists. Master kept these as a display (master text_parser.c:128-129).
"""
import pytest


@pytest.mark.persists
@pytest.mark.bug("E131", on=["d449"])
@pytest.mark.parametrize("line", ["$xvm=", "$xvm ", "$xvm=abc"])
def test_text_set_without_number_keeps_value(tg, keep, line):
    keep("xvm")
    before = tg.get("xvm")
    tg.send_text(line)
    assert tg.get("xvm") == pytest.approx(before)
