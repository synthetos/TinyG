"""E024: feed and spindle overrides have no mfo/sso tokens and M48-M51 are stored but never applied.

canonical_machine.c:1431-1490 only records cm.gmx.*_override_* (`STUBBED: change spindle speed`)
and config_app.c has no sso/ssoe/mfo rows, so CNCjs's {sso:n} gets status 100 (issue #273).
Master lacks them too.
"""
import pytest


@pytest.mark.bug("E024", on=["d440", "d449"])
@pytest.mark.parametrize("token", ["sso", "mfo"])
def test_override_token(tg, token):
    resp = tg.send_json({token: None})
    assert resp.status == 0
    assert isinstance(resp.body.get(token), (int, float))
