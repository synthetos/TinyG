"""N12a: radius mode requires a positive finite radius before a move is accepted.

TinyG Configuration wiki (0.97), Radius mode: linear travel maps to degrees
through the radius; zero radius cannot describe that conversion.
"""
import pytest
pytestmark=[pytest.mark.persists,pytest.mark.dialect("d449"),pytest.mark.bug("N12a",on=["d449"])]
def test_radius_write_refuses_invalid_values(tg,keep):
    keep("aam","ara")
    tg.send_gcode("G21",check=True)
    tg.set("aam",3)
    before=tg.get("ara")
    for value in ("0","-1","NaN","Infinity","1e999"):
        r=tg.send_line('{"ara":'+value+'}')
        assert r.status!=0,f"invalid radius accepted: {value}"
        assert tg.get("ara")==before
    assert tg.stat()!=tg.dialect.stat("PANIC")
