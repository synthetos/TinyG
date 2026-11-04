"""N12: motor geometry above the DDA pulse capacity must not silently lose steps.

TinyG configuration wiki (0.97): step angle, microsteps and travel/revolution
set motor steps per unit. This requests 80 kHz on a 50 kHz DDA; planned speed
must be limited while preserving the requested endpoint and pulse count.
"""
import pytest
pytestmark=[pytest.mark.motion,pytest.mark.ice,pytest.mark.persists,
 pytest.mark.dialect("d449"),pytest.mark.bug("N12",on=[])]

def test_n12_step_rate_overrun(tg, keep, record, ice):
    """A geometry asking for far more than the 50 kHz step tick can give at F30: steps the firmware counts vs commanded."""
    keep("1sa", "1mi", "1tr", "xvm", "xfr", "1ma")
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    for tok, val in (("1ma", 0), ("1sa", 1.8), ("1mi", 8), ("1tr", 0.01), ("xfr", 60), ("xvm", 60)):
        tg.set(tok, val, allow="step-rate overrun geometry")
    spu = ice.peek("st_cfg.mot[0].steps_per_unit")
    before = ice.peek("en.en[0].encoder_steps")
    tg.send_line("G91 G1 X1 F30")
    tg.wait_idle(timeout=60)
    after = ice.peek("en.en[0].encoder_steps")
    info = {"steps_per_unit": spu, "commanded_steps": spu * 1.0, "firmware_counted": after - before,
            "posx": tg.get("posx")}
    tg.send_gcode("G90", check=True)
    record("n12_rate", info)
    assert abs(info["firmware_counted"] - info["commanded_steps"]) <= 2, f"step count mismatch: {info}"
