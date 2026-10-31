"""E161: characters the normaliser deletes splice the digits around them into a different number.

gcode_parser.c _normalize_gcode_block() keeps only letters, digits, '-' and '.', so X0.001*9 ran
as X0.0019 and F100*5 as F1005 (status 0); a RepRap checksum line such as N10 G1 X10 Y5*45 would
run as Y545. Whitespace and '%' are still dropped and '+' is a sign; any other character now makes
the line malformed, with nothing executed. Probe moves are at most 0.01 mm on the old firmware.
"""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.dialect("d449")]

BAD = ["G91 G1 X0.001*9 F100", "G91 G1 X0.001 F100*5", "G91 G1 X0.0#01 F100", "G91 G1 X0.0'01 F100",
       "G91 G1 X0.00[1] F100", "G91 G1 X0.00=1 F100"]


def test_invalid_characters_are_not_spliced(tg):
    start = tg.get("posx")
    try:
        for line in BAD:
            before = tg.get("posx")
            r = tg.send_gcode(line, allow="0.01 mm at most on the old firmware")
            tg.wait_idle()
            assert r.status == tg.dialect.status("INVALID_OR_MALFORMED_COMMAND"), (line, r.status)
            assert tg.get("posx") == pytest.approx(before, abs=1e-4), f"{line} moved"
        for line in ("G91 G1 X+0.001 F100", "G91\tG1 X0.001 F100", "G91 G1 X0.001 F100 (comment * # ok)"):
            assert tg.send_gcode(line, allow="0.001 mm").status == 0, line
            tg.wait_idle()
    finally:
        tg.send_gcode(f"G90 G0 X{start:.3f}", allow="return")
        tg.wait_idle()
