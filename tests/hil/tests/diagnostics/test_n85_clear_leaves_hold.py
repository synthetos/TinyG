"""Diagnostic (bench 1, ICE + hooks): a limit hit during the dwells of a job, then $clear: can new lines run?

Bench 3 seed 144979799: a limit during a run of M codes and dwells, ALARM holds, {"clear":null} reports stat 3, but the
next lines stay queued: stat 6, qr 28 for 60 s, settings refused with status 117. This trips X min at several points
of the same kind of program and reads the hold, flush and queue state before and after the clear, while the ICE can
see them. Pass: after the clear the new lines run and the board is idle within 10 s.
"""
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.ice, pytest.mark.persists,
              pytest.mark.dialect("d449"), pytest.mark.resets_board, pytest.mark.profile("switches-no"),
              pytest.mark.timeout(6000)]   # includes waiting for the board lock

WHY = "N85 limit during dwells"
LINES = ["G21 G91", "G1 X0.5 F600", "M7", "G4 P0.6", "M3 S1000", "G4 P0.6", "M4 S1500", "G4 P0.6", "M5", "G4 P0.6",
         "G0 X1"]
STATE = ["cm.machine_state", "cm.cycle_state", "cm.motion_state", "cm.hold_state", "cm.queue_flush_state",
         "cm.end_hold_requested", "mb.buffers_available", "mr.block_state", "mr.section_state", "st_pre.buffer_state",
         "st_pre.block_type"]


def _snap(ice):
    v = ice.peek_many(STATE)
    v["where"] = ice.where()
    return v


@pytest.mark.parametrize("after", [0.25, 0.5, 0.9, 1.3, 1.8, 2.4])
def test_new_lines_run_after_clear(tg, keep, switches, ice, record, after):
    if "test_hook" not in ice.symbols.sym:
        pytest.skip("needs a TEST_HOOKS=1 build")
    keep("xsn")
    tg.set("xsn", 2, allow="x min as a limit")
    tg.reset()
    for ln in LINES:
        tg.guard.check_gcode(ln, allow=WHY)
    t0 = time.monotonic()
    for ln in LINES:
        tg.send_gcode(ln, timeout=30, allow=WHY)
    time.sleep(max(0.0, after - (time.monotonic() - t0)))
    switches.pulse("x", "min", ms=20)
    time.sleep(1.2)
    alarmed = _snap(ice)
    stat_alarmed = tg.stat()
    tg.clear_alarm()
    time.sleep(0.05)
    cleared = _snap(ice)
    tg.send_gcode("M5 M9", timeout=5, allow=WHY)
    tg.send_gcode("G91 G0 X0.1", timeout=5, allow=WHY)
    t_end, stat, qr = time.monotonic() + 10, None, None
    while time.monotonic() < t_end:
        stat, qr = tg.stat(), tg.get("qr")
        if stat in tg.dialect.idle_stats and qr == 32:
            break
        time.sleep(0.2)
    final = _snap(ice)
    r = {"after": after, "stat_alarmed": stat_alarmed, "alarmed": alarmed, "cleared": cleared, "final": final,
         "stat": stat, "qr": qr}
    record("n85", r)
    print("N85", r)
    stuck = stat not in tg.dialect.idle_stats or qr != 32
    if stuck:
        tg.reset()
    tg.send_gcode("G90", allow="absolute")
    assert not stuck, f"new lines did not run after the clear: {r}"
