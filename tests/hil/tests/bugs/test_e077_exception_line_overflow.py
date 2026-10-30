"""E077: rpt_exception() overflows its 128-byte stack buffer when the alarm message carries a
long input line.

(Since E042 an unsupported G-code is an error, not an alarm, so the alarm with the input line comes
from a soft limit: cm_alarm(status, cs.saved_buf) in cm_straight_traverse.)

report.c:120-123 sprintf_P's "...msg":"%s - %s" into char line_msg[LINE_MSG_LEN] (128,
error.h:53) with msg = cs.saved_buf (up to 127 chars, gcode_parser.c:423/428). With 'Gcode
command unsupported' any line over 60 chars overflows; the 89-char line below writes 157 bytes.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")     # 440's rpt_exception has no msg argument

LONG = "G0 X200 (" + "x" * 80 + ")"          # beyond the soft limit -> cm_alarm(..., saved_buf)


@pytest.fixture(autouse=True)
def soft_limit_x(tg, keep):
    """Soft limits on, X travel 0..100 and X marked homed, so 'G0 X200' alarms with its line."""
    keep("sl", "xtn", "xtm")
    tg.set("sl", 1)
    tg.set("xtn", 0)
    tg.set("xtm", 100)
    tg.send_gcode("G28.3 X0", check=True)


def _alarm_report(tg, line):
    """Send a line that raises a soft-limit alarm quoting the line; return (response, events seen)."""
    resp = tg.send_gcode(line, timeout=3, allow="beyond the soft limit: alarms before any motion")
    return resp, resp.events + tg.drain(quiet=0.5)


def _reports(events):
    return [e.kind for e in events if e.kind in ("exception", "malformed", "banner")]


@pytest.mark.resets_board
@pytest.mark.parametrize("line", [LONG, 'G0 X200 ("quoted" \\ %s %n)'])
def test_long_line_alarm_report_is_bounded(tg, line):
    try:
        resp, events = _alarm_report(tg, line)
        assert resp.status == tg.dialect.status("SOFT_LIMIT_EXCEEDED")
        assert _reports(events) == ["exception"]
        alive = tg.send_json({"fb": None}, timeout=6, check=True)   # no crash, hang or restart
        assert _reports(alive.events) == []
    finally:
        # A smashed frame can restart the board on its own, which leaves the planner dead
        # until a real ^X, so always reset rather than trying to detect it.
        tg.reset()


def test_short_line_alarm_report(tg):
    try:
        resp, events = _alarm_report(tg, "G0 X200")
        assert resp.status == tg.dialect.status("SOFT_LIMIT_EXCEEDED")
        assert _reports(events) == ["exception"]
    finally:
        tg.clear_alarm()
