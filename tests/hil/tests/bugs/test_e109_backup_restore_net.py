"""E109: replaying a 440.20 settings backup on edge fails at net, and a group chunk containing net is rejected whole.

net has no cfgArray row on edge, so marshalling returns STAT_UNRECOGNIZED_NAME for the whole object
(json_parser.c:401-402) before any child is set; it is the only key of the backup that master
accepts and edge rejects (master config_app.c:484).
"""
import pytest


@pytest.mark.persists
@pytest.mark.bug("E109", on=["d449"])
def test_sys_group_with_net_restores(tg, keep):
    keep("mt")
    mt = tg.get("mt")
    resp = tg.send_json({"sys": {"net": 0, "mt": mt}})
    assert resp.status == 0, resp.raw
    assert tg.values_equal("mt", tg.get("mt"), mt)


def test_sys_group_without_net_restores(tg, keep):
    keep("mt")
    mt = tg.get("mt")
    assert tg.send_json({"sys": {"mt": mt}}).status == 0
