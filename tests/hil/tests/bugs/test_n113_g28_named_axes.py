"""N113: G28/G30 with axis words must move only the named axes to the stored position.

LinuxCNC G-code reference, G28/G30: "G28 axes - makes a rapid traverse move to the position specified by axes
including any offsets, then will make a rapid traverse move to the absolute position of the values in parameters
5161-5166 for all axes specified. Any axis not specified will not move." Without axis words every axis goes to the
stored position. The TinyG and g2core wikis (Gcode-Support, "G28, G28.1, G30, G30.1") say the same: "Axes that are not
specified are ignored (not moved)". NIST RS274NGC v3 3.5.8 does not say which axes move in the second leg.

canonical_machine.c _goto_stored_position() ran the intermediate leg with the block's axis flags but the second leg
with FLAGS_ALL, so `G91 G28 Z0` (the CAM idiom for "retract Z only") also sent X and Y to the stored position, in one
rapid together with Z. Master 440.x does the same.

The stored position is X2 Y1 Z1 (machine), the tool starts at X0 Y0 Z0; moves stay within 2.3 mm.
"""
import pytest

pytestmark = [pytest.mark.motion, pytest.mark.timeout(120)]

MOVE = "N113 stored-position test (<= 2.3 mm from a G28.3 origin, returns to it)"
STORED = {"x": 2.0, "y": 1.0, "z": 1.0}


def _pos(tg):
    return {a: round(float(tg.get(f"mpo{a}")), 3) for a in "xyz"}


def _setup(tg, stored):
    for line in ("G21 G90", "G28.3 X0 Y0 Z0", "G53 G0 X2 Y1 Z1", f"{stored}.1", "G53 G0 X0 Y0 Z0"):
        tg.send_gcode(line, check=True, allow=MOVE)
    tg.wait_idle()
    assert _pos(tg) == {"x": 0.0, "y": 0.0, "z": 0.0}


def _run(tg, stored, line):
    _setup(tg, stored)
    tg.send_gcode(line.format(g=stored), check=True, allow=MOVE)
    tg.wait_idle(timeout=30)
    return _pos(tg)


def _home(tg):
    if tg.stat() == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    tg.send_gcode("G21 G90 G53 G0 X0 Y0 Z0", allow=MOVE)
    tg.wait_idle()


@pytest.mark.bug("N113", on=["d440", "d449"])
@pytest.mark.parametrize("stored", ["G28", "G30"])
@pytest.mark.parametrize("line, expect", [
    ("G91 {g} Z0", {"x": 0.0, "y": 0.0, "z": 1.0}),          # CAM "retract Z": only Z goes to the stored Z
    ("G90 {g} Z0.3", {"x": 0.0, "y": 0.0, "z": 1.0}),        # through Z0.3, then only Z to the stored Z
    ("G90 {g} X0.5 Y0", {"x": 2.0, "y": 1.0, "z": 0.0}),     # X and Y named: Z stays
], ids=["g91-z0", "g90-z", "g90-xy"])
def test_named_axes_only(tg, record, stored, line, expect):
    try:
        got = _run(tg, stored, line)
        record("n113", {"line": line.format(g=stored), "expect": expect, "got": got})
        assert got == pytest.approx(expect, abs=0.005), got
    finally:
        _home(tg)


@pytest.mark.parametrize("stored", ["G28", "G30"])
def test_no_axis_words_moves_all(tg, record, stored):
    """Plain G28/G30 still takes every axis to the stored position (unchanged behaviour)."""
    try:
        got = _run(tg, stored, "G90 {g}")
        record("n113_all", {"line": stored, "got": got})
        assert got == pytest.approx(STORED, abs=0.005), got
    finally:
        _home(tg)
