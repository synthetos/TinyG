"""E004: a dwell shorter than one 100 us tick (G4 P0, bare G4) hangs the stepper runtime for ~5 days.

edge stepper.c:1105 `st_pre.dda_ticks = (uint32_t)((microseconds/1000000) * FREQUENCY_DWELL);` gives 0 ticks,
and the dwell ISR at stepper.c:623-627 decrements before testing, so the count wraps to 2^32. _exec_dwell frees
its planner buffer at prep time (planner.c:269-275), so the hang only shows on the next queued item, which is
prepped but never loaded (stepper.c:705-707, :760-762): an M5 behind it keeps one planner buffer forever.
cm_dwell (canonical_machine.c:1220) also never rejects a negative P. Master has the 1-tick floor
(stepper.c:1097) but no negative-P check.
"""
import time

import pytest


def _queue_drains_after(tg, dwell: str) -> bool:
    """Send a dwell, then a queued M5 behind it; True if the planner is empty again 0.5 s later."""
    tg.send_gcode(dwell)
    tg.send_gcode("M5", check=True)          # queued through mp_queue_command (spindle.c:94-98)
    time.sleep(0.5)
    return tg.get("qr") == tg.planner_buffers


def _recover(tg):
    """A wrapped dwell leaves qr at 32 by itself; probe with one more queued item."""
    tg.send_gcode("M5")
    time.sleep(0.3)
    if tg.get("qr") != tg.planner_buffers:
        tg.reset()                           # the stepper runtime stays busy until a real reset


@pytest.mark.resets_board
@pytest.mark.bug("E004", on=["d449"])
@pytest.mark.parametrize("dwell", ["G4 P0", "G4", "G4 P0.00001"])
def test_sub_tick_dwell_does_not_stall_queue(tg, dwell):
    try:
        assert _queue_drains_after(tg, dwell)
        assert tg.stat() != tg.dialect.stat("RUN")
    finally:
        _recover(tg)


@pytest.mark.resets_board
@pytest.mark.bug("E004", on=["d440", "d449"])
def test_negative_dwell_rejected(tg):
    try:
        assert tg.send_gcode("G4 P-1").status == tg.dialect.status("P_WORD_IS_NEGATIVE")
        assert _queue_drains_after(tg, "G4 P0.01")
    finally:
        _recover(tg)


@pytest.mark.resets_board
def test_short_dwell_drains(tg):
    try:
        assert _queue_drains_after(tg, "G4 P0.01")      # 100 ticks
    finally:
        _recover(tg)
