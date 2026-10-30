"""E090: a status report shows the arc endpoint (with vel 0) before the arc starts moving.

edge plan_arc.c:254-256 `cm_cycle_start(); arc.run_state = BLOCK_RUNNING; cm_finalize_move();` moves the model
position to the endpoint at parse time; segments are queued later by cm_arc_callback (controller.c:167), after
sr_status_report_callback (controller.c:162). While motion is STOP, ACTIVE_MODEL = MODEL
(canonical_machine.c:145). JSON-wrapped G-code requests the SR ASAP (json_parser.c:99), so the first SR comes
before the first segment. Issue #140 shows the same on master 440.18.
"""
import pytest

MOVE = "half-circle G2 to X+2 (radius 1 mm) at F300, then back"


@pytest.mark.motion
@pytest.mark.bug("E090", on=["d440", "d449"])
def test_no_endpoint_report_before_arc_moves(tg):
    fields = tg.get("sr") or {}
    if not {"posx", "vel"} <= set(fields):
        pytest.skip(f"status report list lacks posx/vel: {sorted(fields)}")
    tg.send_gcode("G17 G91", check=True)
    start = tg.get("posx")
    end_x = start + 2
    tg.drain(quiet=0.4)
    resp = tg.send_json({"gc": "G2 X2 Y0 I1 J0 F300"}, check=True, allow=MOVE)
    try:
        state, reports = {"vel": 0}, []
        events = iter(resp.events)
        while True:
            ev = next(events, None) or tg.wait_for(lambda e: e.kind == "status", 3.0)
            if ev.kind != "status":
                continue
            state.update(ev.body)
            reports.append(ev.raw)
            if state["vel"] > 0:
                break
            assert "posx" not in ev.body or abs(ev.body["posx"] - end_x) > 0.001, \
                f"endpoint reported before motion: {reports}"
    finally:
        tg.wait_idle()
        tg.send_gcode("G91 G1 X-2 F600", allow=MOVE)
        tg.wait_idle()
