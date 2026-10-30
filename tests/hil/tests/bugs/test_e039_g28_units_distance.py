"""E039: G28 feeds the stored machine-mm position back through cm_set_model_target(), so G20 scales it by 25.4 and G91 adds it.

edge canonical_machine.c:1138-1144 cm_goto_g28_position() traverses to cm.gmx.g28_position (raw mm,
stored at :1134); cm_set_model_target() applies _to_millimeters() again (canonical_machine.h:42) and in
G91 does `target += value` (:495-496). Master has the same structure (master canonical_machine.c:907-914).
G28.3 zeroes the machine position first so the buggy target stays small (2.54 mm).
"""
import pytest

MOVE = "G28 test (X <= 2.6 mm from a G28.3 origin, returns to it)"


def _back_to_origin(tg):
    if tg.stat() == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    tg.send_gcode("G21 G90 G53 G0 X0 Y0 Z0", allow=MOVE)
    tg.wait_idle()


def _store_g28_at_x(tg, x):
    for line in ("G21 G90", "G28.3 X0 Y0 Z0 A0 B0 C0", f"G53 G0 X{x}", "G28.1", "G53 G0 X0"):
        tg.send_gcode(line, check=True, allow=MOVE)
    tg.wait_idle()


@pytest.mark.motion
@pytest.mark.bug("E039", on=["d440", "d449"])
def test_g28_in_inches(tg):
    try:
        _store_g28_at_x(tg, 0.1)
        tg.send_gcode("G20", check=True)
        tg.send_gcode("G28", check=True, allow=MOVE)
        tg.wait_idle()
        assert tg.get("mpox") == pytest.approx(0.1, abs=0.005)    # mpo is always mm
    finally:
        _back_to_origin(tg)


@pytest.mark.motion
@pytest.mark.bug("E039", on=["d440", "d449"])
def test_g28_in_incremental_mode(tg):
    try:
        _store_g28_at_x(tg, 0.1)
        tg.send_gcode("G91 G0 X1", check=True, allow=MOVE)
        tg.send_gcode("G28", check=True, allow=MOVE)
        tg.wait_idle()
        assert tg.get("mpox") == pytest.approx(0.1, abs=0.005)
    finally:
        _back_to_origin(tg)
