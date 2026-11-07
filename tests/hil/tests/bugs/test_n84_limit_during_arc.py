"""N84: a limit hit while an arc is still being queued must leave the board in ALARM.

cm_arc_callback() (plan_arc.c) ran on after an alarm until the flush aborted the arc, and called cm_cycle_start()
before each segment, which writes MACHINE_CYCLE over the ALARM. The planner is full while the arc is generated, so the
ALARM lasted until the stop freed a buffer. Seen on bench 3 (2026-11-06) with real switches: stat 2, 6, then 3 with
nothing sent, and the same program with a line in place of the arc stayed in ALARM (wiki Homing-and-Limits: a limit hit
alarms until cleared).

A chain of short lines (the stop runs across several, freeing their buffers), then an arc long enough to outlast the
planner. Needs a TEST_HOOKS=1 build and --ice for the soft switch. Pass: ALARM holds until the clear.
"""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.ice, pytest.mark.persists, pytest.mark.resets_board,
              pytest.mark.dialect("d449"), pytest.mark.profile("switches-no"), pytest.mark.timeout(120),
              pytest.mark.bug("N84", on=["d449"])]

WHY = "N84 limit during an arc"
LINES = ["G21 G91"] + ["G1 X0.5 F300"] * 12 + ["G17 G2 X30 Y0 I15 J0 F400"]


@pytest.mark.parametrize("after", [0.7, 1.0])
def test_limit_during_arc_keeps_alarm(tg, keep, switches, ice, record, after):
    if "test_hook" not in ice.symbols.sym:
        pytest.skip("needs a TEST_HOOKS=1 build")
    keep("xsn")
    tg.set("xsn", 2, allow="x min as a limit")
    tg.reset()
    t0 = time.monotonic()
    for ln in LINES:
        tg.guard.check_gcode(ln, allow=WHY)
        tg.send_gcode(ln, timeout=30, allow=WHY)
    time.sleep(max(0.0, after - (time.monotonic() - t0)))
    switches.pulse("x", "min", ms=20)
    stats = []
    for _ in range(15):
        time.sleep(0.2)
        stats.append(tg.stat())
    record("n84", {"after": after, "stats": stats})
    tg.reset()
    tg.send_gcode("G90", allow="absolute")
    assert stats[-1] == tg.dialect.stat("ALARM"), f"the alarm did not hold: {stats}"
