"""E159: a value followed by an E word is read as an exponent: G1 X10 E5 commands X = 1,000,000.

gcode_parser.c _get_next_gcode_word() reads every value with strtof(), which accepts exponents, so
the E word of 3D-printer G-code (or any digit string followed by E) scales the value before it
instead of failing as an unsupported word (E042). Here G91 X0.001E3 would move 1 mm on the old
firmware; it must be rejected with nothing executed.
"""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.dialect("d449")]


@pytest.mark.bug("E159", on=["d449"])
def test_e_word_is_not_an_exponent(tg):
    start = tg.get("posx")
    try:
        r = tg.send_gcode("G91 G1 X0.001E3 F300", allow="1 mm at most on the old firmware")
        assert r.status == tg.dialect.status("GCODE_COMMAND_UNSUPPORTED"), r.status
        time.sleep(0.5)
        assert tg.get("posx") == pytest.approx(start, abs=0.001), "the E word scaled X and it moved"
        assert tg.send_gcode("G1 X0.00001 E5 F300", allow="1 mm at most").status == tg.dialect.status("GCODE_COMMAND_UNSUPPORTED")
    finally:
        tg.wait_idle()
        tg.send_gcode(f"G90 G0 X{start:.3f}", allow="return")
        tg.wait_idle()
