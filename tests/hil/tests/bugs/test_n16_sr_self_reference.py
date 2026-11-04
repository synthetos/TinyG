"""N16: reject commands/reports selected as status values, before storing them.

TinyG wiki Status Reports: selectors name status values. Disabling automatic
reports lets the stock rejection test fail without deliberately recursing.
"""
import pytest
from tinyg_hil.snapshot import set_status_report
pytestmark=[pytest.mark.dialect("d449"),pytest.mark.persists,pytest.mark.bug("N16",on=[])]
@pytest.mark.parametrize("token",["sr","srs","qr","rx","sys","help","defa","boot","test"])
def test_report_selector_refuses_commands(tg,keep,token):
    keys=list(tg.get("sr"))
    keep("sv")
    tg.set("sv",0)
    try:
        r=tg.send_json({"sr":{token:True}})
        assert r.status==tg.dialect.status("INPUT_VALUE_RANGE_ERROR")
        assert list(tg.get("sr"))==keys
        r=tg.send_json({"srs":{"set":{token:True}}})
        assert r.status==tg.dialect.status("INPUT_VALUE_RANGE_ERROR")
        assert list(tg.get("sr"))==keys
    finally:
        set_status_report(tg,keys)
