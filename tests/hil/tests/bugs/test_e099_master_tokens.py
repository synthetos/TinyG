"""E099: master tokens net, ma and fd are gone, so a ChiliPeppr restore of a 440.xx backup stops at [net].

Edge's cfgArray has no net/ma/fd rows (master config_app.c:484, 508, 509), so a SET gets
{"r":{},"f":[3,100,N]} via nv_get_index NO_MATCH (json_parser.c:401-402); ChiliPeppr's restore
waits for the key to be echoed in r and never moves on.
"""
import pytest


@pytest.mark.bug("E099", on=["d449"])
@pytest.mark.parametrize("token, value", [("net", 0), ("fd", 0), ("ma", 0.1)])
def test_master_token_accepted(tg, token, value):
    resp = tg.send_json({token: value}, allow="master compatibility token")
    assert resp.status == 0
    assert token in resp.body


@pytest.mark.bug("E099", on=["d449"])
def test_net_text_mode(tg):
    lines = tg.send_text("$net=0")
    assert "ok>" in lines[-1], lines


@pytest.mark.bug("E099", on=["d449"])
@pytest.mark.parametrize("token", ["net", "fd", "ma"])
def test_retired_token_get_is_zero(tg, token):
    resp = tg.send_json({token: None}, allow="retired compatibility token")
    assert resp.status == 0
    assert resp.body[token] == 0


@pytest.mark.parametrize("token, value", [("net", 1), ("fd", 1), ("ma", -1)])
def test_unsupported_legacy_setting_rejected(tg, token, value):
    resp = tg.send_json({token: value}, allow="unsupported legacy setting")
    assert resp.status != 0
