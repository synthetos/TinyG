"""E097: automatic status reports in text mode are multi-line, so OpenPnP's `.*stat:3.*` never matches.

sr_status_report_callback prints with nv_print_list(STAT_OK, TEXT_RESPONSE, JSON_OBJECT)
(report.c:433), one `Machine state:       Stop` line per key; master (report.c:300) uses
TEXT_INLINE_PAIRS and prints `posx:0.100,...,stat:3`.
"""
import re

import pytest


def _text_mode(tg, line: str, allow: str | None = None, timeout: float = 3.0) -> list[str]:
    """Send one line in text mode (motion-guarded) and return its output up to the prompt."""
    tg.guard.check_gcode(line, allow)
    tg.transport.write(line.encode() + b"\n")
    lines = []
    while True:
        ev = tg.wait_for(lambda e: True, timeout)
        lines.append(ev.raw)
        if ev.kind == "prompt":
            return lines


@pytest.mark.motion
@pytest.mark.bug("E097", on=[])
def test_auto_status_report_inline_pairs(tg):
    _text_mode(tg, "$fb")       # any $ line switches the board to text mode
    try:
        _text_mode(tg, "G91")
        lines = _text_mode(tg, "G0 X0.1", allow="0.1 mm move")
        lines += [e.raw for e in tg.drain(quiet=1.0, limit=5.0) if e.kind == "text"]
    finally:
        _text_mode(tg, "G0 X-0.1", allow="return to start")
        tg.drain(quiet=1.0, limit=5.0)
        tg.get("fb")            # back to JSON mode
        tg.wait_idle()
    assert any(re.search(r"(^|,)stat:3(,|$)", line) for line in lines), lines
    assert any(re.search(r"(^|,)posx:-?\d+\.\d+", line) for line in lines), lines
