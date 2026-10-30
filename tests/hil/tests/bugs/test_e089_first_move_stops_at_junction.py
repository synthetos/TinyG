"""E089: the first move after idle decelerates to zero even when the next collinear move arrives right away.

edge planner.c:426 mp_commit_write_buffer calls st_request_exec_move() on every commit, so a lone block starts at
once. It was planned as the last block (plan_line.c:470-471 `bp->exit_velocity = 0;`) and is frozen when it
starts (plan_exec.c:183 `bf->replannable = false;`), so a block queued a few ms later cannot raise its exit
velocity. There is no start-up delay like g2core's PLANNER_STARTUP (planner.cpp:545-560). Master is the same.
"""
import time

import pytest

MOVE = "two collinear 2 mm +X moves at F300, then back"
F = 300.0


def _sample(tg):
    if tg.dialect.name == "d440":
        return {"vel": tg.get("vel"), "posx": tg.get("posx")}
    return tg.send_json({"vel": None, "posx": None}, check=True).body


@pytest.mark.motion
@pytest.mark.persists
@pytest.mark.bug("E089", on=["d440", "d449"])
def test_second_move_joins_first_at_speed(tg, keep):
    keep("xjm")
    tg.set("xjm", 50)                        # 5e7 mm/min^3: a stop at the junction lasts ~0.6 s
    tg.send_gcode("G64 G91", check=True)
    start = tg.get("posx")
    lines = [f"G1 X2 F{F:g}", "G1 X2"]
    for line in lines:
        tg.guard.check_gcode(line, MOVE)
    tg.transport.write("".join(line + "\n" for line in lines).encode())   # one write: both arrive together
    try:
        for _ in range(2):
            assert tg.wait_for(lambda e: e.kind == "response", 2.0).data["f"][1] == 0
        samples, end = [], time.monotonic() + 3
        while time.monotonic() < end:
            samples.append(_sample(tg))
        near = [s["vel"] for s in samples if abs(s["posx"] - (start + 2)) < 0.3]
        assert near, f"no samples near the junction: {samples}"
        assert min(near) > 0.8 * F, f"slowed to {min(near):.0f} mm/min at the junction"
    finally:
        tg.wait_idle()
        tg.send_gcode("G91 G1 X-4 F600", allow=MOVE)
        tg.wait_idle()
