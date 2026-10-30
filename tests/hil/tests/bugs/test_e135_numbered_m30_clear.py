"""E135: M2/M30 clear an ALARM only when the normalized block is exactly "M2" or "M30"; "N10 M30" stays alarmed.

edge gcode_parser.c:179-189 _parse_clear() compares str[0..3] against "M30"/"M2", so a line number or any
other word skips cm_clear() and :72 `ritorno(cm_is_alarmed())` returns 204. Master never clears an alarm
from G-code, and has no {alarm} command to raise one, so this runs on 449 only.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")


@pytest.mark.bug("E135", on=["d449"])
def test_numbered_m30_clears_alarm(tg):
    tg.send_json({"alarm": None})
    assert tg.stat() == tg.dialect.stat("ALARM")
    assert tg.send_gcode("N10 M30").status == 0
    assert tg.stat() != tg.dialect.stat("ALARM")


def test_bare_m30_clears_alarm(tg):
    tg.send_json({"alarm": None})
    assert tg.stat() == tg.dialect.stat("ALARM")
    assert tg.send_gcode("M30").status == 0
    assert tg.stat() != tg.dialect.stat("ALARM")
