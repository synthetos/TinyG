"""N02 hypothesis: during a slow arc the board stops sending status reports (and line replies) for seconds.

Streams one 90-second arc (radius 15 mm at F100, about 94 mm of path) in line mode with si=100 sv=1, after
queuing a few short lines behind it so the planner is full the way the soaks have it. Records every status report
arrival time and the longest gap between consecutive reports while the arc runs, plus the reply times of the
lines queued behind the arc. The soaks' stall watchdog fires at 5 s without a report or reply: a longest gap
above 5 s reproduces N02 deterministically, without the generated files.
"""
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(600)]

ARC = "G2 X0 Y0 I15 J0 F100"                 # full circle, 94.2 mm at 100 mm/min: ~57 s plus ramps
GAP_LIMIT = 5.0


def test_slow_arc_reports(tg, keep, record, request):
    keep("ej", "qv", "rxm", "ex", "sv", "si")
    for tok, val in (("ej", 1), ("qv", 1), ("sv", 1), ("si", 100), ("rxm", 1), ("ex", 1)):
        tg.set(tok, val, allow="line mode with 100 ms reports")
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    tr = tg.transport
    tg.guard.check_gcode("G1 X20 Y20", allow="slow arc")
    t0 = time.monotonic()
    tr.write((ARC + "\n").encode())
    for k in range(6):                                  # short lines behind the arc: replies come as slots free
        tr.write((f"G91 G1 X{0.5 if k % 2 else -0.5} F300\n").encode())
    reports, replies, others = [], [], []
    gaps, last_report = [], None
    end = t0 + 150
    done = False
    while time.monotonic() < end and not done:
        ev = tr.next_event(0.05)
        if ev is None:
            continue
        t = ev.t - t0
        if ev.kind == "status":
            reports.append(round(t, 3))
            if last_report is not None:
                gaps.append(round(t - last_report, 3))
            last_report = t
            body = ev.body if isinstance(ev.body, dict) else {}
            if len(reports) > 20 and body.get("stat") in (3, 4) and body.get("vel", 1) == 0:
                done = True
        elif ev.kind == "response":
            replies.append(round(t, 3))
        else:
            others.append((round(t, 3), ev.kind, ev.raw[:60]))
    tg.wait_idle(timeout=60)
    tg.send_gcode("G90", check=True)
    info = {"reports": len(reports), "first_report": reports[:1], "last_report": reports[-1:],
            "longest_gap_s": max(gaps) if gaps else None, "gaps_over_1s": [g for g in gaps if g > 1.0][:20],
            "replies": replies, "others": others[:10], "end_pos": {a: tg.get(f"pos{a}") for a in "xy"}}
    record("slow_arc", info)
    assert gaps, f"no status reports at all: {info}"
    assert max(gaps) < GAP_LIMIT, f"status reports stopped for {max(gaps):.1f} s during the arc: {info}"
