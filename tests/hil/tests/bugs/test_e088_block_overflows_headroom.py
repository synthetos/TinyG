"""E088: one G-code block that queues more than 4 planner buffers can PANIC with 'Buffer full fatal'.

edge controller.c:434-439 _sync_to_planner admits a line once PLANNER_BUFFER_HEADROOM (4, planner.h:84) buffers
are free, but _execute_gcode_block (gcode_parser.c:476-556) queues one buffer each for S, T, M6, M3, M7, M8
and G4. When they run out, planner.c:403 reports 'failed to get planner buffer' and planner.c:217 / :260 call
cm_panic_P(STAT_BUFFER_FULL_FATAL). Master has the same headroom and raises a hard alarm (planner.c:202-203).

30 two-second dwells leave at most 6 buffers free (each dwell frees its buffer when it is prepped, planner.c:271),
so the 7-item block below cannot fit.
"""
import pytest

BLOCK = "T1 M6 S100 M3 M7 M8 G4 P0.1"


@pytest.mark.motion
@pytest.mark.resets_board
@pytest.mark.bug("E088", on=["d440"])
def test_block_with_seven_queued_items(tg):
    bad = {tg.dialect.stat(s) for s in ("ALARM", "SHUTDOWN")} | (
        {tg.dialect.stat("PANIC")} if "PANIC" in tg.dialect.stat_names else set())
    try:
        for _ in range(30):
            tg.send_gcode("G4 P2", check=True)
        resp = tg.send_gcode(BLOCK, timeout=15, allow="spindle/coolant outputs toggle on a bare bench")
        events = resp.events + tg.drain(quiet=0.3)
        assert resp.status == 0, resp.raw
        assert not [e.raw for e in events if e.kind == "exception"]
        assert tg.stat() not in bad
    finally:
        tg.reset()                           # discards ~60 s of queued dwells and any PANIC


@pytest.mark.motion
@pytest.mark.resets_board
def test_m30_behind_a_nearly_full_queue(tg):
    """M2/M30 queue their resets (offsets, coordinate system, M5, M9) and a finalize: five buffers."""
    try:
        for _ in range(30):
            tg.send_gcode("G4 P2", check=True)
        resp = tg.send_gcode("M30", timeout=15)
        events = resp.events + tg.drain(quiet=0.3)
        assert resp.status == 0, resp.raw
        assert not [e.raw for e in events if e.kind == "exception"]
    finally:
        tg.reset()
