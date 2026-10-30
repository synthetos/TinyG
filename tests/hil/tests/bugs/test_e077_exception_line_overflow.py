"""E077: rpt_exception() overflows its 128-byte stack buffer when the alarm message carries a
long input line.

report.c:120-123 sprintf_P's "...msg":"%s - %s" into char line_msg[LINE_MSG_LEN] (128,
error.h:53) with msg = cs.saved_buf (up to 127 chars, gcode_parser.c:423/428). With 'Gcode
command unsupported' any line over 60 chars overflows; the 89-char line below writes 157 bytes.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")     # 440's rpt_exception has no msg argument

LONG = "G43 H1 (" + "x" * 80 + ")"           # plain G43 is unsupported -> cm_alarm(..., saved_buf)


def _alarm_report(tg, line):
    """Send a line that raises an unsupported-command alarm; return (response, events seen)."""
    resp = tg.send_gcode(line, timeout=3)
    return resp, resp.events + tg.drain(quiet=0.5)


def _reports(events):
    return [e.kind for e in events if e.kind in ("exception", "malformed", "banner")]


@pytest.mark.resets_board
@pytest.mark.parametrize("line", [LONG, 'G43 H1 ("quoted" \\ %s %n)'])
def test_long_line_alarm_report_is_bounded(tg, line):
    try:
        resp, events = _alarm_report(tg, line)
        assert resp.status == tg.dialect.status("GCODE_COMMAND_UNSUPPORTED")
        assert _reports(events) == ["exception"]
        alive = tg.send_json({"fb": None}, timeout=6, check=True)   # no crash, hang or restart
        assert _reports(alive.events) == []
    finally:
        # A smashed frame can restart the board on its own, which leaves the planner dead
        # until a real ^X, so always reset rather than trying to detect it.
        tg.reset()


def test_short_line_alarm_report(tg):
    try:
        resp, events = _alarm_report(tg, "G43 H1")
        assert resp.status == tg.dialect.status("GCODE_COMMAND_UNSUPPORTED")
        assert _reports(events) == ["exception"]
    finally:
        tg.clear_alarm()
