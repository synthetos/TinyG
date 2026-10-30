"""E053: a line that starts with '?' followed by more text is silently dropped, with no response.

controller.c:279-282 (edge) `else if ((*cs.bufp == '?') && (strlen(cs.bufp) > 1)) { cs.bufp++; }`
is a link in _dispatch_kernel's else-if chain, so after the '?' is skipped no parser runs and the
host loses the response for that line. 440 sends such lines to the text parser instead.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")


@pytest.mark.bug("E053", on=[])
def test_leading_question_mark_line_gets_response(tg):
    resp = tg.send_line('?{"fb":null}', timeout=1.5)
    assert resp.status == 0
    assert resp.body.get("fb") == pytest.approx(tg.identity["fb"])


def test_plain_line_gets_response(tg):
    assert tg.send_line('{"fb":null}', timeout=1.5).body.get("fb") == pytest.approx(tg.identity["fb"])
