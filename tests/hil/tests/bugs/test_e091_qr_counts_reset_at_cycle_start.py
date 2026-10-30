"""E091: queue-report qi/qo counts are zeroed at cycle start, losing buffers queued just before the first move.

edge canonical_machine.c:1775-1781 cm_cycle_start() calls qr_init_queue_report(), which zeroes
qr.buffers_added/buffers_removed (report.c:615-621) without reporting them. In the block `M3 G2 ...` the M3
is queued (planner.c:425 +1, gcode_parser.c:489) before cm_arc_feed calls cm_cycle_start (plan_arc.c:254), so
its +1 is lost; the M3 waits behind a dwell, so its -1 is reported later and qo ends one ahead of qi.
Master's cm_cycle_start resets the counts the same way (canonical_machine.c:1379-1385).
"""
import time

import pytest

MOVE = "M3 plus a half-circle G2 to X+2 at F500, then back"


def _counts(events):
    q = [e.data for e in events if e.kind == "queue"]
    return sum(d.get("qi", 0) for d in q), sum(d.get("qo", 0) for d in q)


def _get(tg, token, events):
    """GET that keeps the queue reports seen while waiting for the response."""
    r = tg.send_json({token: None}, check=True)
    events += r.events
    return r.body[token]


@pytest.mark.motion
@pytest.mark.persists
@pytest.mark.bug("E091", on=["d440", "d449"])
def test_queue_report_counts_balance(tg, keep):
    keep("qv")
    tg.set("qv", 2)
    tg.send_gcode("G17 G91", check=True)
    tg.wait_idle()
    tg.drain(quiet=0.3)
    for token in ("qi", "qo"):               # qi/qo GETs return the pending counts and zero them
        _get(tg, token, [])
    events = []
    try:
        for line in ("G4 P0.5", "M3 G2 X2 Y0 I1 J0 F500", "M5"):
            events += tg.send_gcode(line, check=True, allow=MOVE).events
        end = time.monotonic() + 10
        while time.monotonic() < end:
            if _get(tg, "qr", events) == tg.planner_buffers and _get(tg, "stat", events) in tg.dialect.idle_stats:
                break
            time.sleep(0.05)
        time.sleep(0.4)                      # arc QRs are throttled to one per 200 ticks (report.h:37)
        events += tg.drain(quiet=0.3)
        added, removed = _get(tg, "qi", events), _get(tg, "qo", events)    # counts not yet reported
        q_added, q_removed = _counts(events)
        added, removed = added + q_added, removed + q_removed
        assert added == removed, f"qi total {added}, qo total {removed}"
    finally:
        tg.send_gcode("G91 G1 X-2 F600", allow=MOVE)
        tg.wait_idle()
