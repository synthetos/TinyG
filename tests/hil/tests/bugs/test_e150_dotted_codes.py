"""E150: an unimplemented dotted G/M code runs as its integer code and answers OK.

gcode_parser.c switches on (uint16_t)value; only bases with a _point() sub-switch (G28, G30, G38,
G43, G61, G90, G91, G92, M201) look at the decimal. So G59.1-G59.3 (NIST coordinate systems 7-9)
select G59, M3.1 starts the spindle and M30.1 ends the program, all with status 0.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")

G_UNSUPPORTED = ["G54.1", "G59.1", "G59.2", "G59.3", "G17.1", "G21.5", "G64.1", "G4.1 P0.1", "G0.5 X0", "G1.1 X0 F100",
                 "G92.19", "G61.15", "G90.11"]
M_UNSUPPORTED = ["M3.1", "M5.5", "M30.1", "M7.2"]
STILL_SUPPORTED = ["G61.1", "G61.10", "G61", "G90.1", "G91.1", "G90", "G92.1", "G92.10", "G43.1 Z0", "G49"]


def test_unimplemented_dotted_codes_are_rejected(tg):
    g_err = tg.dialect.status("GCODE_COMMAND_UNSUPPORTED")
    m_err = tg.dialect.status("MCODE_COMMAND_UNSUPPORTED")
    tg.send_gcode("G54", check=True)
    try:
        for code in G_UNSUPPORTED:
            assert tg.send_gcode(code, allow="rejected: no motion").status == g_err, code
        assert tg.get("coor") == 1, "a rejected G59.x changed the coordinate system"
        for code in M_UNSUPPORTED:
            assert tg.send_gcode(code).status == m_err, code
        for code in STILL_SUPPORTED:
            assert tg.send_gcode(code).status == 0, code
    finally:
        tg.send_gcode("M5")
        tg.send_gcode("G54 G90 G90.1 G61")
