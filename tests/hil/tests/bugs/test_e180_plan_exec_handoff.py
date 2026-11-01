"""E180: the exec can start a block while the planner is rewriting it, and the move is lost.

_plan_block_list() (plan_line.c) recomputes queued blocks in place with interrupts on, and
mp_calculate_trapezoid() (plan_zoid.c) zeroes body_length before it computes the new head, body and
tail. When the exec interrupt starts the next block inside that window, it copies a plan whose
lengths do not add up to the move (often all zero) and the move ends without moving: the runtime
position stays behind and every later block is built from it, so the machine finishes off target
while an idle report (which reads the model) shows the target. Seen once in a bench gate as a
constant 0.080 mm offset on the dense goose outline (one 0.08 mm segment lost).

The window is a few hundred microseconds, so on its own the loss is rare. On a TEST_HOOKS=1 build the
debugger holds each trapezoid rewrite for test_plan.replan_delay_us, and the exec counts the blocks it
starts with inconsistent lengths. The program, 300 gently turning 0.08 mm segments and a move back
to the start, is streamed in line mode at F300, where a sweep on the stock planner tore blocks at
every delay tried (0.8, 1.5 and 2.5 ms); it runs twice. The checks are the count and the runtime
position after each run, read with the ICE while the machine is idle.
"""
import math

import pytest

from tinyg_hil import sender

pytestmark = [pytest.mark.ice, pytest.mark.motion, pytest.mark.dialect("d449")]

SEGMENTS = 300
FEED = 300
DELAY_US = 1500


def _program():
    """Gently turning 0.08 mm segments, as dense CAM output, and the straight move back."""
    pts, x, y = [], 0.0, 0.0
    for i in range(SEGMENTS):
        x += 0.08 * math.cos(i * 0.03)
        y += 0.08 * math.sin(i * 0.03)
        pts.append((x, y))
    return ["G21 G90 G17"] + [f"G1 X{x:.4f} Y{y:.4f} F{FEED}" for x, y in pts] + [f"G1 X0 Y0 F{FEED}"]


@pytest.mark.bug("E180", on=[])
def test_exec_never_starts_a_half_replanned_block(tg, ice, keep, record):
    if "test_plan" not in ice.symbols.sym:
        pytest.skip("requires a TEST_HOOKS=1 build with the planner hand-off hook")
    keep("ej", "rxm", "ex", "qv")
    for token, value in {"ej": 1, "qv": 1, **sender.MODES["line"]}.items():
        if tg.get(token) != value:
            tg.set(token, value, allow="E180: line-mode streaming")
    lines = _program()
    for line in lines:
        tg.guard.check_gcode(line, allow="E180: short segments that return to the start")
    runs = []
    try:
        for _ in range(2):
            tg.send_gcode("G21 G90 G17 G92 X0 Y0", check=True)
            start = ice.peek_many(["mr.position[0]", "mr.position[1]"])
            ice.poke("test_plan.starts", 0)
            ice.poke("test_plan.torn_starts", 0)
            ice.poke("test_plan.replan_delay_us", DELAY_US)
            res = sender.stream(tg, lines, "line")
            ice.poke("test_plan.replan_delay_us", 0)
            snap = ice.peek_many(["test_plan.starts", "test_plan.torn_starts", "mr.position[0]", "mr.position[1]"])
            off = math.hypot(snap["mr.position[0]"] - start["mr.position[0]"],
                             snap["mr.position[1]"] - start["mr.position[1]"])
            runs.append({"acked": res.acked, "starts": snap["test_plan.starts"],
                         "torn_starts": snap["test_plan.torn_starts"], "off_mm": round(off, 4)})
            tg.send_gcode("G92.1")
    finally:
        ice.poke("test_plan.replan_delay_us", 0)
        tg.send_gcode("G92.1")
        record("e180", runs)
    for run in runs:
        assert run["starts"] >= SEGMENTS, f"the exec started only {run['starts']} blocks"
        assert run["torn_starts"] == 0, f"the exec started {run['torn_starts']} blocks with lengths that do not add up"
        assert run["off_mm"] < 0.002, f"the program returns to its start but the runtime ended {run['off_mm']} mm away"
