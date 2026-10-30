"""E023: G10 L20 ignores the G92 origin offset and the tool length offset, and rejects P0.

edge canonical_machine.c:925 `cm.offset[cs][axis] = cm.gmx.position[axis] - _to_millimeters(offset[axis])`,
while the work position is position - (G5x + G92) + tool_offset (:382, :308-313). :911 rejects P0.
Master has no L20 at all: every G10 is treated as L2 (master canonical_machine.c:662-674).
G10 offsets persist to EEPROM via the deferred write, so each test keeps the touched g54 values.
"""
import pytest


@pytest.mark.persists
@pytest.mark.bug("E023", on=["d440"])
def test_g10_l20_with_g92_active(tg, keep):
    keep("g54x")
    try:
        for line in ("G21 G54", "G28.3 X0", "G10 L2 P1 X0", "G92 X5", "G10 L20 P1 X0"):
            tg.send_gcode(line, check=True)
        tg.wait_idle()
        assert tg.get("posx") == pytest.approx(0, abs=0.001)
    finally:
        tg.send_gcode("G92.1", check=True)


@pytest.mark.persists
@pytest.mark.dialect("d449")         # master has no G43.1
def test_g10_l20_with_tool_offset(tg, keep):
    keep("g54z")
    try:
        for line in ("G21 G54", "G43.1 Z10", "G10 L20 P1 Z0"):
            tg.send_gcode(line, check=True)
        tg.wait_idle()
        assert tg.get("posz") == pytest.approx(0, abs=0.001)
    finally:
        tg.send_gcode("G49", check=True)     # M2 does not cancel the tool offset


@pytest.mark.persists
@pytest.mark.bug("E023", on=["d440"])
def test_g10_l20_p0_is_active_system(tg, keep):
    keep("g54x")
    tg.send_gcode("G21 G54", check=True)
    assert tg.send_gcode("G10 L20 P0 X0").status == 0
    tg.wait_idle()
    assert tg.get("posx") == pytest.approx(0, abs=0.001)


@pytest.mark.persists
@pytest.mark.dialect("d449")
def test_g10_l20_inactive_system_inherits_origin_offset(tg, keep):
    keep("g54x", "g55x")
    try:
        for line in ("G21 G54", "G28.3 X0", "G10 L2 P1 X0", "G92 X5", "G10 L20 P2 X2"):
            tg.send_gcode(line, check=True)
        tg.wait_idle()
        assert tg.get("posx") == pytest.approx(5, abs=0.001)
        tg.send_gcode("G55", check=True)
        tg.wait_idle()
        assert tg.get("posx") == pytest.approx(2, abs=0.001)
    finally:
        tg.send_gcode("G92.1", check=True)
        tg.send_gcode("G54", check=True)


@pytest.mark.persists
@pytest.mark.dialect("d449")
def test_g10_l2_inches_preserves_rotary_degrees(tg, keep):
    keep("g54x", "g54a")
    try:
        tg.send_gcode("G20 G10 L2 P1 X1 A30", check=True)
        tg.wait_idle()
        tg.send_gcode("G21", check=True)
        assert tg.get("g54x") == pytest.approx(25.4, abs=0.001)
        assert tg.get("g54a") == pytest.approx(30, abs=0.001)
    finally:
        tg.send_gcode("G21", check=True)


@pytest.mark.persists
@pytest.mark.dialect("d449")
@pytest.mark.parametrize("pword", ["P-1", "P7", "P256", "P1.5", ""])
def test_g10_invalid_system_does_not_change_offsets(tg, keep, pword):
    keep("g54x")
    tg.send_gcode("G21 G54", check=True)
    before = tg.get("g54x")
    assert tg.send_gcode(f"G10 L20 {pword} X9").status == 161
    assert tg.get("g54x") == pytest.approx(before, abs=0.001)
