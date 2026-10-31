"""E054: cm_set_model_target() does not seed gm.target from the model position, so a rejected block leaves a stale target.

edge canonical_machine.c:483-498 only overwrites (G90) or adds to (G91) the flagged axes of cm.gm.target.
cm_arc_feed() calls it at plan_arc.c:209 and then rejects an impossible centre at :299 without restoring
it; when idle, the alarm's feedhold and flush do nothing (:1553, :1567), so the next move picks up the
rejected X/Y endpoint. Master has the same pattern (master plan_arc.c:135, :262-266 return an error).
"""
import pytest

MOVE = "stale-target test (Z -0.2 mm; buggy firmware also moves X/Y by 1 mm), returns to start"


def _mpo(tg):
    return {a: tg.get(f"mpo{a}") for a in "xyz"}


@pytest.mark.motion
@pytest.mark.bug("E054", on=["d440"])
def test_rejected_arc_does_not_leak_into_next_move(tg):
    tg.wait_idle()
    assert tg.get("qr") == tg.planner_buffers, "test requires an idle, empty planner"
    start = _mpo(tg)
    try:
        # centre (5,5) is 7.07 from the start but 5.66 from the end: rejected, no motion
        assert tg.send_gcode("G17 G91 G2 X1 Y1 I5 J5 F100", allow=MOVE).status != 0
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
        tg.send_gcode("G91 G1 Z-0.2 F60", check=True, allow=MOVE)
        tg.wait_idle()
        assert tg.get("mpox") == pytest.approx(start["x"], abs=0.01)
        assert tg.get("mpoy") == pytest.approx(start["y"], abs=0.01)
        assert tg.get("mpoz") == pytest.approx(start["z"] - 0.2, abs=0.01)
    finally:
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
        tg.send_gcode("G90 G53 G0 " + " ".join(f"{a.upper()}{v:.4f}" for a, v in start.items()), allow=MOVE)
        tg.wait_idle()
