"""E018 (PR #272 repro): a tiny move after M201.3 must not stop the machine with 'Move time is NAN'.

GitHub PR #272 (OpenPnP, master 440.21): after `M201.3 X38884 Y48990` a 0.0005/0.0006 mm G1 at F12.3 from rest
raised a hard alarm, st 31 "Move time is NAN". Head and tail of the move round to zero, and plan_zoid.c then sets the
body-only cruise velocity to the entry velocity, 0 from rest; plan_exec.c divides the body length by it. The PR's
change (keep the computed cruise velocity when the entry velocity is zero) is the E018 fix on edge (6e4cd1b3).
This replays the PR's own lines, relative so any bench travel works, with the PR's axis jerk (540000).
M201.3 is transient, so the jerk it sets ends with the test's reset.
"""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.resets_board, pytest.mark.timeout(120),
              pytest.mark.bug("E018", on=["d440"])]

LINES = ["G21 G91", "G1 X1.0 Y1.0 F60.68", "M400", "M114", "M201.3 X38884 Y48990", "G1 X0.0005 Y0.0006 F12.30"]


def test_tiny_move_after_m2013(tg, keep, record):
    keep("xjm", "yjm")
    tg.set("xjm", 540000, allow="PR #272 axis jerk")
    tg.set("yjm", 540000, allow="PR #272 axis jerk")
    bad = {tg.dialect.stat(s) for s in ("ALARM", "SHUTDOWN")} | (
        {tg.dialect.stat("PANIC")} if "PANIC" in tg.dialect.stat_names else set())
    mark = len(tg.transport.history)
    try:
        for ln in LINES:
            tg.send_gcode(ln, check=True, allow="PR #272 lines")       # every line must be accepted, or nothing is tested
        end, stats = time.monotonic() + 10, []
        while time.monotonic() < end:
            stats.append(tg.stat())
            if stats[-1] in bad or (stats[-1] in tg.dialect.idle_stats and len(stats) > 5):
                break
            time.sleep(0.2)
        exc = [e.raw for e in tg.transport.history[mark:] if e.kind == "exception"]
        record("pr272", {"stats": stats[-5:], "exceptions": exc})
        assert not bad & set(stats), f"stat went to {stats[-1]}: {exc}"
        assert not any('"st":31' in x or "NAN" in x.upper() for x in exc), exc
    finally:
        if tg.stat() in bad:
            tg.reset()
        else:
            tg.wait_idle(timeout=10)
            tg.send_gcode("G91 G1 X-1.0005 Y-1.0006 F600", allow="return")
            tg.wait_idle(timeout=20)
            tg.send_gcode("G90", allow="absolute")
        tg.reset()                                   # drop the transient M201.3 jerk
