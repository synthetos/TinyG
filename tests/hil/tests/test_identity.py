"""Identity and JSON protocol basics (harness design I1-I15)."""
import pytest


def test_identity(tg):
    ident = tg.identity
    assert ident["fv"] == pytest.approx(0.97)
    assert ident["hp"] == 1
    assert ident["hv"] >= 6
    assert isinstance(ident["id"], str) and ident["id"]
    assert ident["fb"] == pytest.approx(tg.dialect.table["fb"], abs=1) or ident["fb"] < 445


def test_footer(tg):
    resp = tg.send_json({"fb": None})
    assert resp.status == 0
    assert resp.footer.rev in tg.dialect.footer_revs
    if resp.footer.rev == 1:
        assert resp.footer.checksum_ok
    if resp.footer.rev in (1, 2):
        assert resp.footer.count == len('{"fb":null}') + 1


def test_unknown_token(tg):
    assert tg.send_json({"xyzzy": None}).status == tg.dialect.status("UNRECOGNIZED_NAME")


def test_malformed_json(tg):
    assert tg.send_line('{"fb":null').status == tg.dialect.status("JSON_SYNTAX_ERROR")


def test_set_readonly(tg):
    resp = tg.send_json({"fb": 500})
    assert resp.status != 0
    assert tg.get("fb") == pytest.approx(tg.identity["fb"])


def test_text_mode_round_trip(tg):
    lines = tg.send_text("$fb")
    assert any(line.startswith("[fb]") for line in lines)
    assert lines[-1].startswith("tinyg [")
    assert tg.send_gcode("G21").raw.startswith("{")     # back in JSON mode


@pytest.mark.dialect("d440")
def test_multi_pair_runs_first_pair_only_440(tg):
    body = tg.send_json('{"fv":null,"hp":null}').body
    assert body["fv"] == pytest.approx(0.97) and body["hp"] is None


@pytest.mark.dialect("d449")
def test_multi_pair_runs_all_449(tg):
    body = tg.send_json('{"fv":null,"hp":null}').body
    assert body["fv"] == pytest.approx(0.97) and body["hp"] == 1


@pytest.mark.dialect("d449")
def test_tid_echo_449(tg):
    resp = tg.send_json('{"tid":7,"fb":null}')
    assert resp.status == 0 and resp.event.data.get("tid", resp.body.get("tid")) == 7
