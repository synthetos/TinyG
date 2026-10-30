"""E041: arcs are never soft-limit checked, not even their endpoint.

edge plan_arc.c:574-589 `_test_arc_soft_limits()` has its body commented out and returns STAT_OK;
segments go straight to mp_aline() (:95). G0/G1 do check (canonical_machine.c:1101-1104, 1246-1249).
Master has the same code commented out (master plan_arc.c:170-177).
"""
import time

import pytest

MOVE = "arc soft-limit test (at most 2 mm X / 1 mm Y from a G28.3 origin, returns to it)"


def _settle(tg, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if tg.stat() not in (tg.dialect.stat("RUN"), tg.dialect.stat("HOLD")) and \
                tg.get("qr") == tg.planner_buffers:
            return
        time.sleep(0.1)
    raise TimeoutError(f"still moving after {timeout}s")


@pytest.mark.motion
@pytest.mark.persists
@pytest.mark.bug("E041", on=["d440"])
def test_arc_endpoint_beyond_soft_limit_is_rejected(tg, keep):
    keep("sl", "xtn", "xtm")
    try:
        tg.send_gcode("G21 G90 G17", check=True)
        tg.send_gcode("G28.3 X0 Y0", check=True)      # marks X and Y homed
        tg.wait_idle()
        tg.set("xtn", 0)
        tg.set("xtm", 1)
        tg.set("sl", 1)
        resp = tg.send_gcode("G91 G2 X2 Y0 I1 J0 F300", allow=MOVE)
        _settle(tg)
        rejected = tg.status_name(resp.status).startswith("SOFT_LIMIT") or \
            tg.stat() == tg.dialect.stat("ALARM")
        assert rejected
        assert tg.get("mpox") == pytest.approx(0, abs=0.01)
    finally:
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()         # settings writes are ignored while alarmed
        tg.set("sl", 0)              # keep() restores the user's value afterwards
        tg.send_gcode("G90 G53 G0 X0 Y0", allow=MOVE)
        tg.wait_idle()
