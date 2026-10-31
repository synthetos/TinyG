"""Diagnosis: what PWM duty does M3 produce when $p1csl == $p1csh (0/0 in cm_get_spindle_pwm)?

The PWM1 timer is TCD1 (hardware.h); its PER and CCB registers are read over the ICE after M3, for
a normal speed range and for an equal one. Only the record matters.
"""
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice, pytest.mark.dialect("d449")]

TCD1 = 0x0940
PER, CCB = TCD1 + 0x26, TCD1 + 0x2A


def _duty(ice):
    with ice.halted():
        per = int.from_bytes(ice.read(PER, 2), "little")
        ccb = int.from_bytes(ice.read(CCB, 2), "little")
    return {"per": per, "ccb": ccb, "duty": ccb / per if per else None}


@pytest.mark.persists
def test_pwm_with_equal_speed_range(tg, ice, keep, record):
    keep("p1csl")
    keep("p1csh")
    res = {"phase": {t: tg.get(t) for t in ("p1cpl", "p1cph", "p1pof")}}
    try:
        tg.set("p1csl", 1000)
        tg.set("p1csh", 2000)
        tg.send_gcode("M3 S1500", check=True, allow="spindle PWM only")
        time.sleep(0.3)
        res["normal_S1500"] = _duty(ice)
        tg.send_gcode("M5", check=True)
        tg.set("p1csh", 1000)                  # equal range
        tg.send_gcode("M3 S1000", check=True, allow="spindle PWM only")
        time.sleep(0.3)
        res["equal_S1000"] = _duty(ice)
        tg.send_gcode("M3 S5000", check=True, allow="spindle PWM only")
        time.sleep(0.3)
        res["equal_S5000"] = _duty(ice)
    finally:
        tg.send_gcode("M5")
        time.sleep(0.3)
        res["after_M5"] = _duty(ice)
        record("pwm_equal_range", res)
