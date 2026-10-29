"""Gcode modal state read back through JSON (harness design M1-M8). No motion."""
import pytest


@pytest.mark.parametrize("gcode, token, value", [
    ("G20", "unit", 0), ("G21", "unit", 1),
    ("G18", "plan", 1), ("G19", "plan", 2), ("G17", "plan", 0),
    ("G91", "dist", 1), ("G90", "dist", 0),
    ("G55", "coor", 2), ("G54", "coor", 1),
    ("G61", "path", 0), ("G61.1", "path", 1), ("G64", "path", 2),
    ("G93", "frmo", 0), ("G94", "frmo", 1),
])
def test_modal(tg, gcode, token, value):
    tg.send_gcode(gcode, check=True)
    assert tg.get(token) == value


def test_m2_resets_modes(tg):
    tg.send_gcode("G91 G55 G18", check=True)
    tg.send_gcode("G20", check=True)
    tg.send_gcode("M2", check=True)
    assert (tg.get("dist"), tg.get("coor"), tg.get("plan")) == (0, 1, 0)
    assert tg.get("unit") == 0      # M2 does not reset units


def test_line_number(tg):
    assert tg.send_gcode("N10 G21", check=True).body.get("n") == 10


def test_unsupported_mcode(tg):
    assert tg.send_gcode("M200").status == tg.dialect.status("MCODE_COMMAND_UNSUPPORTED")
    assert tg.stat() not in (tg.dialect.stat("ALARM"),)

