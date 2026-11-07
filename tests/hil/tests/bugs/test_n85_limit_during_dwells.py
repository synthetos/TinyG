"""N85: a limit hit between the M codes and dwells of a job must flush the job, and $clear must leave the board usable.

_enter_alarm() requests a hold and a planner flush. Between moves the hold stops in FEEDHOLD_SYNC, which only an aline
ends, while _run_requested_queue_flush() refuses to flush in SYNC and cancels the dwell whose end would have woken the
exec. The queue stays full behind the ALARM, $clear reports stop with the hold still latched, and the next lines queue
behind it: stat 6 until a reset, persisted settings refused with status 117. Seen on bench 3 (2026-11-06, seed
144979799) and read with the ICE on bench 1: hold_state SYNC, queue_flush_state REQUESTED, 1.2 s after the trip
(wiki Homing-and-Limits: a limit hit stops at once and alarms; $clear returns to a usable machine).

Needs a TEST_HOOKS=1 build and --ice for the soft switch. Pass: the job is flushed in ALARM, and after the clear new
lines run.
"""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.ice, pytest.mark.persists, pytest.mark.resets_board,
              pytest.mark.dialect("d449"), pytest.mark.profile("switches-no"), pytest.mark.timeout(120),
              pytest.mark.bug("N85", on=[])]

WHY = "N85 limit during dwells"
LINES = ["G21 G91", "G1 X0.5 F600", "M7", "G4 P0.6", "M3 S1000", "G4 P0.6", "M4 S1500", "G4 P0.6", "M5", "G4 P0.6",
         "G0 X1"]


@pytest.mark.parametrize("after", [0.5, 0.9, 1.3])
def test_limit_during_dwells_flushes_and_clears(tg, keep, switches, ice, record, after):
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
    time.sleep(1.2)
    stat_alarmed, qr_alarmed = tg.stat(), tg.get("qr")
    tg.clear_alarm()
    tg.send_gcode("M5 M9", timeout=5, allow=WHY)
    tg.send_gcode("G91 G0 X0.1", timeout=5, allow=WHY)
    t_end, stat, qr = time.monotonic() + 5, None, None
    while time.monotonic() < t_end:
        stat, qr = tg.stat(), tg.get("qr")
        if stat in tg.dialect.idle_stats and qr == 32:
            break
        time.sleep(0.1)
    r = {"after": after, "stat_alarmed": stat_alarmed, "qr_alarmed": qr_alarmed, "stat": stat, "qr": qr}
    record("n85", r)
    tg.reset()
    tg.send_gcode("G90", allow="absolute")
    assert stat_alarmed == tg.dialect.stat("ALARM"), f"the alarm did not hold: {r}"
    assert qr_alarmed == 32, f"the job was not flushed: {r}"
    assert stat in tg.dialect.idle_stats and qr == 32, f"new lines did not run after the clear: {r}"
