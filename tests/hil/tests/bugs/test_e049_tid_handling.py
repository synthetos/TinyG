"""E049: the transaction id (tid) is lost on errors, rounded through a float, filtered at jv<3,
and no longer emitted at the top level.

json_parser.c:137-160 executes pairs one by one and returns on the first error, so a trailing tid
is never marshalled. config_app.c:476 makes tid TYPE_FLOAT (16777217 -> 16777216).
json_parser.c:668-671 skips it as a config echo when jv < 3. f33b062f (448.30) removed the
top-level emission that 446.x hosts (issue #197) rely on.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")     # 440 has no tid


def _tid(resp):
    return resp.event.data.get("tid", resp.body.get("tid"))


@pytest.mark.bug("E049", on=[])
def test_tid_echoed_on_error(tg):
    resp = tg.send_json('{"gc":"M200","tid":17}')
    assert resp.status == tg.dialect.status("MCODE_COMMAND_UNSUPPORTED")
    assert _tid(resp) == 17


@pytest.mark.bug("E049", on=[])
def test_tid_exact_above_float_precision(tg):
    assert _tid(tg.send_json('{"fb":null,"tid":16777217}', check=True)) == 16777217


@pytest.mark.bug("E049", on=[])
def test_tid_at_top_level(tg):
    assert tg.send_json('{"fb":null,"tid":33}', check=True).event.data.get("tid") == 33


@pytest.mark.persists
@pytest.mark.parametrize("jv", [1, 2])
@pytest.mark.bug("E049", on=[])
def test_tid_echoed_at_low_jv(tg, keep, jv):
    keep("jv")
    tg.set("jv", jv)
    assert _tid(tg.send_json('{"fb":null,"tid":5}', check=True)) == 5
