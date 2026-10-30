"""Diagnostic: does homing onto a software switch complete reliably? (E114 hold-completion race)

Homes X onto a firmware-mounted switch N times in a row and records, per attempt, whether the cycle
ended, homx, the machine position and the time taken. A hang (no response, cycle never ends) points at
the homing/feedhold race in E114.
"""
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice, pytest.mark.motion]


@pytest.mark.profile("switches-no", "homing-x")
def test_homing_repeat(tg, switches, record):
    switches.mount("x", "min", at=-3.0)
    results = []
    try:
        for i in range(5):
            t0 = time.monotonic()
            try:
                tg.send_gcode("G28.2 X0", check=True, timeout=5, allow="home X onto a software switch")
                tg.wait_idle(timeout=30)
                results.append({"i": i, "ok": True, "homx": tg.get("homx"), "mpox": tg.get("mpox"),
                                "s": round(time.monotonic() - t0, 2)})
            except Exception as e:
                results.append({"i": i, "ok": False, "error": f"{type(e).__name__}: {str(e)[:200]}",
                                "s": round(time.monotonic() - t0, 2)})
                break
            tg.send_gcode("G90 G0 X5", check=True, allow="move off the switch")
            tg.wait_idle()
    finally:
        record("attempts", results)
        switches.unmount_all()
    assert all(r["ok"] and r["homx"] == 1 for r in results), results
