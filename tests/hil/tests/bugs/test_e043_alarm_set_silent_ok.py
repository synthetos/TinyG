"""E043: in ALARM, JSON SETs (including {"gc":...}) are skipped but answered with status 0.

json_parser.c:154-161 wraps nv_set() in `if (!cm_is_alarmed())` with no else branch, so the
kernal returns STAT_OK for a SET that never ran. gc goes through nv_set (config_app.c:499), so
M2/M30 inside {"gc":...} never reach _parse_clear (gcode_parser.c:71). Raw G-code returns 204.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")     # 440 has no {alarm:n} trigger


@pytest.fixture
def alarmed(tg):
    tg.send_json({"alarm": None}, check=True)
    assert tg.stat() == tg.dialect.stat("ALARM")
    yield
    tg.clear_alarm()


def test_gc_set_rejected_in_alarm(tg, alarmed):
    assert tg.send_json({"gc": "G91"}).status == tg.dialect.status("COMMAND_REJECTED_BY_ALARM")
    assert tg.get("dist") == 0


@pytest.mark.persists
def test_config_set_rejected_in_alarm(tg, alarmed, keep):
    keep("xvm")
    xvm = tg.get("xvm")     # write the current value so nothing changes even if the SET runs
    assert tg.send_json({"xvm": xvm}).status == tg.dialect.status("COMMAND_REJECTED_BY_ALARM")


@pytest.mark.bug("E043", on=["d449"])
def test_gc_m30_clears_alarm(tg, alarmed):
    tg.send_json({"gc": "M30"})
    assert tg.stat() != tg.dialect.stat("ALARM")


def test_raw_gcode_rejected_in_alarm(tg, alarmed):
    assert tg.send_gcode("G91").status == tg.dialect.status("COMMAND_REJECTED_BY_ALARM")
    assert tg.get("dist") == 0
