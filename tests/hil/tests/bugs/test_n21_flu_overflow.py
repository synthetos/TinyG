"""N21: finite input must remain finite in canonical units and expanded jerk. TinyG wiki TinyG-Configuration-for-Firmware-Version-0.97, units conversion and xJM/xJH million-unit notation; set_flu and jerk setters must validate before storage."""
import pytest
pytestmark=[pytest.mark.persists,pytest.mark.dialect("d449"),pytest.mark.bug("N21",on=[])]
def test_inch_overflow_keeps_setting(tg,keep):
    keep("g54x","xjd","xjm","xjh")
    tg.send_gcode("G20",check=True)
    try:
        for token in ("g54x","xjd","xjm","xjh"):
            before=tg.get(token)
            reply=tg.send_json({token:1e38})
            assert reply.status==tg.dialect.status("INPUT_VALUE_RANGE_ERROR"),reply.raw
            assert tg.get(token)==pytest.approx(before)
    finally:
        tg.send_gcode("G21",check=True)
