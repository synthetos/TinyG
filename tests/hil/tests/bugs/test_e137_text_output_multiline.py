"""E137: text output regressions: {txt:"$h"} prints raw newlines into the JSON string, and automatic
text-mode status reports are one line per item instead of one inline-pairs line.

controller.c:325-333 opens {"r":{"txt":" and runs text_parser in JSON_MODE_TXT_OVERRIDE; help.c:88-117
then printf_P's multi-line help (and help.c:77 a SYSTEM READY message) unescaped. report.c:433
prints automatic reports with TEXT_RESPONSE (text_parser.c:193-206); master report.c:300 used TEXT_INLINE_PAIRS.
"""
import re

import pytest

ASYNC = ("status", "queue", "rx", "exception")


def _lines_after(tg, data: bytes, quiet: float = 0.6, limit: float = 6.0):
    """Write raw bytes and return every line until the board is quiet (no response parsing)."""
    tg.drain(quiet=0.05)
    tg.transport.write(data)
    return tg.drain(quiet=quiet, limit=limit)


@pytest.mark.dialect("d449")
@pytest.mark.bug("E137", on=["d449"])
def test_txt_help_is_one_json_response(tg):
    replies = [e for e in _lines_after(tg, b'{"txt":"$h"}\n') if e.kind not in ASYNC]
    assert [e.kind for e in replies] == ["response"]


@pytest.mark.motion
def test_text_mode_auto_status_report_is_inline(tg):
    if "posx" not in (tg.get("sr") or {}):
        pytest.skip("posx is not in the status-report list")
    events = _lines_after(tg, b"$fb\n", quiet=0.3)         # any $ command switches to text mode
    try:
        for move in ("G91 G1 X1 F300", "G1 X-1"):
            tg.guard.check_gcode(move, allow="1 mm relative move and back")
            events += _lines_after(tg, move.encode() + b"\n", quiet=1.0)
    finally:
        tg.get("fb")                                        # a JSON line switches back
        tg.wait_idle()
        tg.send_gcode("G90", check=True)
    inline = re.compile(r"(^|,)posx:-?\d")
    assert any(inline.search(e.raw) for e in events if e.kind == "text")
