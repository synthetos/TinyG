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
            tg.send_gcode("G4 P2", check=True, timeout=10)   # waits for a free buffer once the queue is full
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
            tg.send_gcode("G4 P2", check=True, timeout=10)   # waits for a free buffer once the queue is full
        resp = tg.send_gcode("M30", timeout=15)
        events = resp.events + tg.drain(quiet=0.3)
        assert resp.status == 0, resp.raw
        assert not [e.raw for e in events if e.kind == "exception"]
    finally:
        tg.reset()


@pytest.mark.motion
@pytest.mark.resets_board
def test_densest_block_with_program_end_behind_a_full_queue(tg):
    """Review case: S, M6, M3, M8, G54, a move and M30 in one block (eleven queued items before E069)."""
    try:
        for _ in range(30):
            tg.send_gcode("G4 P2", check=True, timeout=10)   # waits for a free buffer once the queue is full
        resp = tg.send_gcode("S100 M6 M3 M8 G54 G1 X1 F60 M30", timeout=40,
                             allow="spindle/coolant outputs toggle on a bare bench; 1 mm move")
        events = resp.events + tg.drain(quiet=0.3)
        assert resp.status == 0, resp.raw
        assert not [e.raw for e in events if e.kind == "exception"]
    finally:
        tg.reset()


@pytest.mark.motion
@pytest.mark.resets_board
@pytest.mark.dialect('d449')
@pytest.mark.parametrize('permission', ['M48', 'M50 P0'])
def test_permission_fits_densest_legal_block(tg, permission):
    """Permission adds one command; G28's two legs still fit the 12-slot reserve."""
    try:
        tg.send_gcode('G28.1', check=True, allow='save current unloaded machine position')
        for _ in range(30):
            tg.send_gcode('G4 P2', check=True, timeout=10)
        reply = tg.send_gcode(f'S100 T1 M6 M3 M7 M8 {permission} G49 G54 G28 X0 M30',
                              timeout=40, allow='unloaded headroom test with outputs and saved-position motion')
        assert reply.status == 0, reply.raw
        assert not [e.raw for e in reply.events + tg.drain(quiet=.3) if e.kind == 'exception']
    finally:
        tg.reset()
