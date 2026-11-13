"""A refused persisted SET must not apply an earlier RAM-only SET on its line."""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(90), pytest.mark.bug("N109", on=["d449"])]


def test_busy_line_refused_before_any_set(tg, keep, record):
    keep("xvm")
    old_vm = tg.get("xvm")
    tg.send_gcode("G21 G90", allow="units")
    tg.send_gcode("G28.3 X0", allow="declare X")
    move = "N101 G91 G1 X20 F100"
    tg.guard.check_gcode(move, allow="N109 move")
    tg.send_line(move)
    time.sleep(1.5)
    before = tg.get("n")
    try:
        response = tg.send_json({"n": 12345, "xvm": old_vm + 1}, allow="N109 refused line")
        after = {"n": tg.get("n"), "xvm": tg.get("xvm")}
        record("n109", {"before_n": before, "before_xvm": old_vm,
                        "status": response.status, "after": after})
    finally:
        tg.wait_idle(timeout=30)
        tg.send_gcode("G90", allow="absolute")
    assert response.status == 117
    assert after == {"n": before, "xvm": old_vm}, "a refused line applied an earlier SET"
