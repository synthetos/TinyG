"""E018: a slow move from rest whose head and tail round to zero panics with 'Move time is NAN'.

edge plan_zoid.c:294-295 sets `bf->cruise_velocity = bf->entry_velocity` (0 from rest) when head and tail were
zeroed below MIN_HEAD/TAIL_LENGTH (plan_zoid.c:121-122, i.e. 2 < F < J*1.7375e-9). plan_exec.c:541
`mr.body_length / mr.cruise_velocity` is then inf, segment_time NaN, and stepper.c:1002-1003 calls
cm_panic_P(STAT_PREP_LINE_MOVE_TIME_IS_NAN). With xjm=5000 the window is F2..F8.7. Master has the same
plan_zoid code and raises a hard alarm instead (master stepper.c:996).
"""
import time

import pytest

MOVE = "0.2 mm +X at F5 (2.4 s), then back"


@pytest.mark.motion
@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.bug("E018", on=["d440"])
def test_slow_move_from_rest_completes(tg, keep):
    keep("xjm")
    tg.set("xjm", 5000)
    bad = {tg.dialect.stat(s) for s in ("ALARM", "SHUTDOWN")} | (
        {tg.dialect.stat("PANIC")} if "PANIC" in tg.dialect.stat_names else set())
    resp = tg.send_gcode("G91 G1 X0.2 F5", allow=MOVE)
    try:
        end = time.monotonic() + 6
        stats, events = [], list(resp.events)
        while time.monotonic() < end:
            r = tg.send_json({"stat": None}, check=True)
            events += r.events
            stats.append(r.body["stat"])
            if stats[-1] in bad or tg.get("qr") == tg.planner_buffers:
                break
        errors = [e.raw for e in events if e.kind == "exception"]
        assert not errors
        assert not bad & set(stats), f"stat went to {stats[-1]}"
        tg.wait_idle(timeout=5)
    finally:
        if tg.stat() in bad:
            tg.reset()
        else:
            tg.send_gcode("G91 G1 X-0.2 F600", allow=MOVE)
            tg.wait_idle()
