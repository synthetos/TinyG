"""E005: G and M numbers are cast to uint8 before the switch, so M256-M265 alias to M0-M9 and G256+ to G0+.

edge gcode_parser.c:286 `switch((uint8_t)value)` (G words) and :384 `switch((uint8_t)value)` (M words).
Master fixed the M switch (master gcode_parser.c:352 uint16) but not the G switch (:259).
M259/M260 alias to spindle on and must never be sent; M265 (=M9) and M258 (=M2) are harmless.
"""


def test_m265_is_unsupported(tg):
    assert tg.send_gcode("M265").status == tg.dialect.status("MCODE_COMMAND_UNSUPPORTED")


def test_m258_does_not_run_program_end(tg):
    tg.send_gcode("G91", check=True)
    tg.send_gcode("M258")
    tg.wait_idle()
    assert tg.get("dist") == 1      # M2 would have reset the distance mode to G90


def test_g260_is_unsupported(tg):
    # Stock firmware runs it as G4 P0.01. Fixed edge returns 131 and alarms (E042); the next test clears it.
    assert tg.send_gcode("G260 P0.01").status == tg.dialect.status("GCODE_COMMAND_UNSUPPORTED")
