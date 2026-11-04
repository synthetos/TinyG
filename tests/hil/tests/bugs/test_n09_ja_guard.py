"""N09: refuse nonpositive junction acceleration before storage.

TinyG wiki TinyG-Configuration-for-Firmware-Version-0.97, $JA, defines ja as the acceleration for corner speed.
A negative value takes sqrt of a negative number and removes the corner limit.
No motion is needed: SET status and the retained value expose the error.
"""
import pytest
pytestmark=[pytest.mark.persists,pytest.mark.dialect("d449"),pytest.mark.bug("N09",on=[])]
def test_invalid_ja_keeps_previous_value(tg,keep):
    keep("ja")
    tg.send_gcode("G21",check=True)
    before=tg.get("ja")
    for value in (-1,0):
        reply=tg.send_json({"ja":value})
        assert reply.status==tg.dialect.status("INPUT_LESS_THAN_MIN_VALUE"),reply.raw
        assert tg.get("ja")==pytest.approx(before)
    for text in ('{"ja":NaN}','{"ja":Infinity}','{"ja":1e999}'):
        reply=tg.send_line(text)
        assert reply.status!=0,reply.raw
        assert tg.get("ja")==pytest.approx(before)

def test_inches_overflow_keeps_previous_value(tg,keep):
    keep("ja")
    tg.send_gcode("G20",check=True)
    try:
        before=tg.get("ja")
        reply=tg.send_json({"ja":1e38})
        assert reply.status!=0,reply.raw
        assert tg.get("ja")==pytest.approx(before)
    finally:
        tg.send_gcode("G21",check=True)
