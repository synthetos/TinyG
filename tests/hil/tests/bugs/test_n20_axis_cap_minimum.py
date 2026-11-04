"""N20: axis rate caps must support a representable move duration. TinyG wiki TinyG-Configuration-for-Firmware-Version-0.97, xVM/xFR; NIST RS274NGC v3 3.5.1/3.5.2. Refuse rates below 0.00001 canonical units/minute (the existing numeric zero tolerance); refuse move durations beyond the segment counter range before queuing."""
import pytest
pytestmark=[pytest.mark.persists,pytest.mark.dialect("d449"),pytest.mark.bug("N20",on=["d449"])]
def test_subminimum_axis_caps(tg,keep):
    keep("xvm","xfr")
    tg.send_gcode("G21",check=True)
    for token in ("xvm","xfr"):
        before=tg.get(token)
        reply=tg.send_json({token:1e-38})
        assert reply.status==tg.dialect.status("INPUT_LESS_THAN_MIN_VALUE"),reply.raw
        assert tg.get(token)==pytest.approx(before)
