"""E102: strings in edge JSON output go out unescaped or unterminated, giving invalid JSON.

util.c:292-300 strcat_string() copies nv->str verbatim, so a (MSG ...) containing a quote breaks
the response (master's sprintf("\"%s\"") is the same). util.c:177-187 str_escape() never writes
a NUL and json_parser.c:645 escapes into the input buffer itself, so the "err" echo of a
malformed line picks up stale bytes past the original end. The er-report part of this finding is
covered by E127; the banner moving from r{} to sr{} is a policy question and is not tested.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")


def _send_any(tg, line, timeout=2.0):
    """Like send_line, but also returns a JSON line that fails to parse instead of timing out."""
    tg.transport.write(line.encode() + b"\n")
    return tg.wait_for(lambda e: e.kind in ("response", "malformed"), timeout)


def test_message_with_quotes_is_valid_json(tg):
    ev = _send_any(tg, 'G90 (MSG say "hi")')
    assert ev.kind == "response", ev.raw
    assert ev.data["r"].get("msg", "").strip() == 'say "hi"'


def test_syntax_error_echo_is_exact(tg):
    # Fill the input buffers with non-NUL bytes so an unterminated copy shows up.
    for _ in range(10):
        tg.send_gcode("G90 (" + "x" * 140 + ")", check=True)
    ev = _send_any(tg, '{"xvm":}')
    assert ev.kind == "response", ev.raw
    assert ev.data["f"][1] == tg.dialect.status("JSON_SYNTAX_ERROR")
    assert ev.data["r"]["err"] == '{"xvm":}'
