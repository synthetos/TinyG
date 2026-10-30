"""E116: a feedhold request arriving as the last block finishes is never cleared: hold stays REQUESTED (or SYNC).

edge canonical_machine.c:1554-1555 accepts '!' whenever motion_state != MOTION_STOP. cm_start_hold
(:1603-1607) only acts `if (mp_has_runnable_buffer())` and has no else branch, so once the exec has freed the
last buffer the request is never dropped; plan_exec.c:315-316 also skips cm_cycle_end unless hold is OFF.
'~' needs FEEDHOLD_HOLD (:1613) and '%' only ends a hold in HOLD (:1687), so the next move is held at once.
Master clears the request every pass (master canonical_machine.c:1265-1270).

The '!' is swept across the end of a short move. On stock 449.01 motion_state also stays RUN after every G0/G1
(E033), so any '!' that lands after the move leaves REQUESTED and the test fails deterministically; once E033
is fixed it becomes a bounded (<= ~55 s) race test of the window described above.
"""
import random
import time

import pytest

MOVE = "alternating 1 mm X moves at F600 with a '!' near the end of each"


def _move_time(tg):
    """Seconds from the move's response until its last buffer is freed (qr back to 32)."""
    tg.send_gcode("G91 G1 X1 F600", check=True, allow=MOVE)
    t0 = time.monotonic()
    while tg.get("qr") != tg.planner_buffers:
        assert time.monotonic() - t0 < 5
    elapsed = time.monotonic() - t0
    tg.wait_idle()
    tg.send_gcode("G91 G1 X-1 F600", check=True, allow=MOVE)
    tg.wait_idle()
    return elapsed


@pytest.mark.motion
@pytest.mark.resets_board
@pytest.mark.bug("E116", on=["d449"])
def test_late_feedhold_never_sticks(tg):
    rng = random.Random(116)
    t_end = _move_time(tg)
    ok = {tg.dialect.hold("OFF"), tg.dialect.hold("HOLD")}
    deadline = time.monotonic() + 50
    step, stuck = 1, None
    try:
        while time.monotonic() < deadline:
            tg.send_gcode(f"G91 G1 X{step} F600", check=True, allow=MOVE)
            time.sleep(max(0.0, t_end + rng.uniform(-0.04, 0.02)))
            tg.send_raw(b"!")
            time.sleep(0.5)
            hold = tg.get("hold")
            if hold not in ok:
                stuck = hold
                break
            if hold == tg.dialect.hold("HOLD"):
                tg.send_raw(b"~")
            tg.wait_idle(timeout=5)
            step = -step
        assert stuck is None, f"hold stuck at {stuck} with nothing moving"
    finally:
        if stuck is not None or tg.get("hold") != tg.dialect.hold("OFF"):
            tg.reset()
        elif step < 0:                       # last move went +1 and was not undone
            tg.send_gcode("G91 G1 X-1 F600", allow=MOVE)
            tg.wait_idle()
