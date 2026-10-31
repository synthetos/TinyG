"""E127: ALARM/exception reports embed the raw input line unescaped, so an alarm raised by a
JSON-wrapped command produces an er line that is not valid JSON.

report.c:121-123 formats "msg":"%s - %s" with msg = cs.saved_buf, the whole input line
(controller.c:266), passed by gcode_parser.c:423/428, canonical_machine.c:1103/1248 and
controller.c:218/231. For {"gc":...} the line's own quotes land inside the msg string.

Since E042 an unsupported G-code is an error, not an alarm, so the alarm quoting the line comes from
a soft limit (cm_alarm(status, cs.saved_buf)), as in test_e077.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")     # 440's er report carries no input line


@pytest.fixture(autouse=True)
def soft_limit_x(tg, keep):
    """Soft limits on, X travel 0..100 and X marked homed, so 'G0 X200' alarms with its line."""
    keep("sl", "xtn", "xtm")
    tg.set("sl", 1)
    tg.set("xtn", 0)
    tg.set("xtm", 100)
    tg.send_gcode("G28.3 X0", check=True)


def _alarm_reports(tg, send):
    try:
        resp = send()
        assert resp.status != 0 and tg.stat() == tg.dialect.stat("ALARM"), resp.raw
        return [e for e in resp.events + tg.drain(quiet=0.3) if e.kind in ("exception", "malformed")]
    finally:
        tg.clear_alarm()


def test_alarm_from_json_command_is_valid_json(tg):
    reports = _alarm_reports(tg, lambda: tg.send_json({"gc": "G0 X200"}, allow="beyond the soft limit: alarms before any motion"))
    assert [e.kind for e in reports] == ["exception"], [e.raw for e in reports]


def test_alarm_from_raw_gcode_is_valid_json(tg):
    reports = _alarm_reports(tg, lambda: tg.send_gcode("G0 X200", allow="beyond the soft limit: alarms before any motion"))
    assert [e.kind for e in reports] == ["exception"], [e.raw for e in reports]
