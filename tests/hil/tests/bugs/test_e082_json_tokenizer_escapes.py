"""E082: the edge JSON tokenizer mishandles escapes in string values.

json_parser.c:248-251 (`if (*str == '\\') { if (*(str+1) == '"') { str++; } continue; }`) drops
both characters of an escaped quote, and for an escaped backslash right before the closing quote
it skips onto that quote and eats it, so the line fails with a syntax error. (The pointer-compare
safety valve at :340 and the NULL return of _skip_whitespace at :193-203 only show on ARM; the
trailing-whitespace test below is a regression guard for them.)
"""
import pytest


def _send_any(tg, line, timeout=2.0):
    """Like send_line, but also returns a JSON line that fails to parse instead of timing out."""
    tg.transport.write(line.encode() + b"\n")
    return tg.wait_for(lambda e: e.kind in ("response", "malformed"), timeout)


@pytest.mark.bug("E082", on=[])
def test_escaped_backslash_before_closing_quote(tg):
    # gc value is: G90 (abc\    (a comment may omit its closing paren)
    ev = _send_any(tg, r'{"gc":"G90 (abc\\"}')
    assert ev.kind == "response", ev.raw
    assert ev.data["f"][1] == 0


# The quotes have to survive the tokenizer (E082) and be re-escaped on output (E102).
# 440 has no escape handling at all: its string ends at the first quote.
@pytest.mark.bug("E082", on=["d440"])
@pytest.mark.bug("E102", on=["d440", "d449"])
def test_escaped_quote_kept_in_message(tg):
    ev = _send_any(tg, r'{"gc":"G90 (MSG say \"hi\")"}')
    assert ev.kind == "response", ev.raw
    assert ev.data["r"].get("msg", "").strip() == 'say "hi"'


def test_trailing_whitespace_after_object(tg):
    assert tg.send_line('{"fb":null}   ', check=True).body.get("fb") == pytest.approx(tg.identity["fb"])
