"""N12b: a tiny step angle must be refused before derived steps/unit overflows.

TinyG Configuration wiki 0.97, Motor Settings: sa/tr/mi define steps/unit.
The test does not move with the invalid geometry; rejection is the contract.
"""
import pytest
pytestmark=[pytest.mark.persists,pytest.mark.dialect("d449"),pytest.mark.bug("N12b",on=["d449"])]
def test_tiny_step_angle_refused(tg,keep):
    keep("1sa","1tr","1mi")
    tg.send_gcode("G21",check=True)
    before={key:tg.get(key) for key in ("1sa","1tr","1mi")}
    r=tg.send_line('{"1sa":1e-38}')
    assert r.status==tg.dialect.status("INPUT_VALUE_RANGE_ERROR")
    for key,value in before.items():
        assert tg.get(key)==value
