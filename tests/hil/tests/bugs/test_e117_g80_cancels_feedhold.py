"""E117: G80 (or any non-motion block in motion mode CANCEL) runs cm_cycle_end() from the parser and drops a feedhold.

edge gcode_parser.c:534 `case MOTION_MODE_CANCEL: { status = cm_cancel_motion_mode(); break;}` and
canonical_machine.c:1118-1121 call cm_cycle_end(), which (when cycle_state != CYCLE_OFF, :1786) runs
_exec_program_finalize synchronously: `cm.hold_state = FEEDHOLD_OFF; ... cm_set_motion_state(MOTION_STOP);
mp_zero_segment_velocity();` (:1749-1756). During a hold that ends the hold without '~'. A '~' resume sets
CYCLE_MACHINING (:1628), so the second hold below is exposed. Master G80 only sets the model
(master gcode_parser.c:507).
"""
import time

import pytest

MOVE = "4 mm +X at F120 with two feedholds"


def _wait_hold(tg, timeout=3.0):
    end = time.monotonic() + timeout
    while tg.get("hold") != tg.dialect.hold("HOLD"):
        assert time.monotonic() < end, "feedhold did not reach HOLD"


@pytest.mark.motion
@pytest.mark.resets_board
def test_g80_keeps_feedhold(tg):
    start = tg.get("posx")
    tg.send_gcode("G91 G1 X4 F120", check=True, allow=MOVE)
    try:
        time.sleep(0.5)
        tg.send_raw(b"!")
        _wait_hold(tg)
        tg.send_raw(b"~")
        time.sleep(0.3)
        tg.send_raw(b"!")
        _wait_hold(tg)
        tg.send_gcode("G80", check=True)
        time.sleep(0.5)
        assert tg.get("hold") == tg.dialect.hold("HOLD"), "G80 ended the feedhold"
    finally:
        if tg.get("hold") == tg.dialect.hold("HOLD"):
            tg.send_raw(b"%")                # flush the held block; the model is resynced to the runtime
            tg.wait_idle()
            tg.send_gcode(f"G91 G1 X{start - tg.get('posx'):.4f} F600", allow=MOVE)
            tg.wait_idle()
        else:
            tg.reset()                       # the held block would replay from the hold point (E030)
