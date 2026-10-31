"""E033: G0/G1 never call cm_cycle_start(), so stat never reads RUN and the cycle never ends.

edge canonical_machine.c:1108-1113 and 1252-1257 test
`if ((status == STAT_MINIMUM_LENGTH_MOVE) || (status != STAT_MINIMUM_TIME_MOVE)) { status = STAT_OK; } else
{ cm_cycle_start(); }`, so only a MIN_TIME move starts a cycle (inverted by 6f5ed663). cm_get_combined_state
(:177-178) then reports machine_state (READY/STOP) during motion, and cm_cycle_end (:1786) never runs, leaving
motion_state at MOTION_RUN after the move: a later '!' while idle latches FEEDHOLD_REQUESTED (:1554-1555) and
holds the next move. Master calls cm_cycle_start() before mp_aline (canonical_machine.c:888).
"""
import time

import pytest

MOVE = "2 mm +X at F120, then back"


def _sample(tg):
    return tg.send_json({"stat": None, "vel": None}, check=True).body


@pytest.mark.motion
def test_stat_is_run_during_g1(tg):
    tg.send_gcode("G91 G1 X2 F120", check=True, allow=MOVE)
    try:
        moving = []
        end = time.monotonic() + 3
        while time.monotonic() < end and len(moving) < 20:
            s = _sample(tg)
            if s["vel"] > 0:
                moving.append(s["stat"])
        assert moving, "never saw vel > 0"
        assert set(moving) == {tg.dialect.stat("RUN")}, f"stat while moving: {sorted(set(moving))}"
    finally:
        tg.wait_idle()
        tg.send_gcode("G91 G1 X-2 F600", allow=MOVE)
        tg.wait_idle()


@pytest.mark.motion
@pytest.mark.resets_board
def test_feedhold_while_idle_does_not_hold_next_move(tg):
    tg.send_gcode("G91 G1 X0.5 F300", check=True, allow=MOVE)
    tg.wait_idle()
    tg.send_raw(b"!")
    time.sleep(0.3)
    hold = tg.get("hold")
    try:
        tg.send_gcode("G91 G1 X-0.5 F300", check=True, allow=MOVE)
        tg.wait_idle(timeout=5)
        assert hold == tg.dialect.hold("OFF")
    finally:
        if tg.get("hold") != tg.dialect.hold("OFF") or tg.get("qr") != tg.planner_buffers:
            tg.reset()
