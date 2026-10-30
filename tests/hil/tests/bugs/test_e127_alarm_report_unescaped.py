"""E127: ALARM/exception reports embed the raw input line unescaped, so an alarm raised by a
JSON-wrapped command produces an er line that is not valid JSON.

report.c:121-123 formats "msg":"%s - %s" with msg = cs.saved_buf, the whole input line
(controller.c:266), passed by gcode_parser.c:423/428, canonical_machine.c:1103/1248 and
controller.c:218/231. For {"gc":"G41"} the line's own quotes land inside the msg string.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")     # 440's er report carries no input line


def _alarm_reports(tg, send):
    try:
        resp = send()
        assert resp.status == tg.dialect.status("GCODE_COMMAND_UNSUPPORTED")
        return [e for e in resp.events + tg.drain(quiet=0.3) if e.kind in ("exception", "malformed")]
    finally:
        tg.clear_alarm()


@pytest.mark.bug("E127", on=["d449"])
def test_alarm_from_json_command_is_valid_json(tg):
    reports = _alarm_reports(tg, lambda: tg.send_json({"gc": "G41"}))   # G41 is unsupported -> alarm
    assert [e.kind for e in reports] == ["exception"], [e.raw for e in reports]


def test_alarm_from_raw_gcode_is_valid_json(tg):
    reports = _alarm_reports(tg, lambda: tg.send_gcode("G41"))
    assert [e.kind for e in reports] == ["exception"], [e.raw for e in reports]
