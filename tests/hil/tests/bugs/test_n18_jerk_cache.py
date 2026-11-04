"""N18: positive canonical jerk must be at least one million units/minute cubed. TinyG wiki TinyG-Configuration-for-Firmware-Version-0.97, xJM/xJH: inputs below one million use million-unit notation. The lower bound matches M201.3; boot-zero cache must always initialize (plan_line.c)."""
import pytest
pytestmark=[pytest.mark.persists,pytest.mark.dialect("d449"),pytest.mark.bug("N18",on=["d449"])]
def test_small_jerk_refused(tg,keep):
    keep("xjm","xjh")
    tg.send_gcode("G21",check=True)
    for token in ("xjm","xjh"):
        before=tg.get(token)
        for value in (1e-38,0.001):
            reply=tg.send_json({token:value})
            assert reply.status==tg.dialect.status("INPUT_LESS_THAN_MIN_VALUE"),reply.raw
            assert tg.get(token)==pytest.approx(before)
