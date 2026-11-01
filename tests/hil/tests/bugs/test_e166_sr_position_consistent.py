"""E166: status reports show positions the machine never occupied.

report.c builds a status report one token at a time; each position token reads
mp_get_runtime_work_position(axis) (plan_line.c), i.e. mr.position[axis], while the exec interrupt rewrites
mr.position at every segment (plan_exec.c copy_vector). posx and posy can come from two different segments,
and on the AVR a 4-byte float can be read while it is being written. Streaming the dense goose outline,
reports landed 0.01-0.10 mm off the commanded path, while the X/Y path rebuilt from ~354,000 step pulses on
the logic analyzer stayed within 0.0012 mm (diagnostics/test_e166_true_path.py). Hosts plotting the
toolpath or showing a DRO see errors the machine never made.

Every status-report position while streaming goose must lie on the commanded path within 0.005 mm
(reports print 3 decimals; segment endpoints are on the path).
"""
import math
from collections import defaultdict

import pytest

from tinyg_hil import gcodeplay, sender

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449")]

TOL = 0.005
CELL = 0.5


def _index(path):
    cells = defaultdict(list)
    for a, b in zip(path, path[1:]):
        if a is None or b is None:
            continue
        for i in range(int(math.floor(min(a[0], b[0]) / CELL)) - 1, int(math.floor(max(a[0], b[0]) / CELL)) + 2):
            for j in range(int(math.floor(min(a[1], b[1]) / CELL)) - 1, int(math.floor(max(a[1], b[1]) / CELL)) + 2):
                cells[(i, j)].append((a, b))
    return cells


def _distance(cells, p):
    best = float("inf")
    for (ax, ay), (bx, by) in cells.get((int(math.floor(p[0] / CELL)), int(math.floor(p[1] / CELL))), ()):
        dx, dy = bx - ax, by - ay
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
        best = min(best, math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy)))
    return best


@pytest.mark.bug("E166", on=[])
def test_status_report_positions_lie_on_the_path(tg, keep, record):
    keep("ej", "rxm", "ex", "qv", "sl", "xtn", "xtm", "ytn", "ytm", "ztn", "ztm")
    lines = gcodeplay.goose()
    path = gcodeplay.path(lines)
    pts = [p for p in path if p is not None]
    box = ((min(p[0] for p in pts) - 2, max(p[0] for p in pts) + 2), (min(p[1] for p in pts) - 2, max(p[1] for p in pts) + 2))
    tg.transport.write(b'{"sr":null}\n')
    original_sr = list(tg.wait_for(lambda e: e.kind == "response" and "sr" in e.body, 3.0).body["sr"])
    try:
        for token, value in {"ej": 1, "qv": 1, **sender.MODES["line"]}.items():
            if tg.get(token) != value:
                tg.set(token, value, allow="E166: line mode")
        tg.send_json({"sr": {k: True for k in ("posx", "posy", "vel", "stat")}}, check=True)
        tg.send_gcode("G21 G90 G28.3 X0 Y0 Z0", check=True)
        for token, value in {"xtn": -2, "xtm": gcodeplay.SIZE + 2, "ytn": -2, "ytm": gcodeplay.SIZE + 2,
                             "ztn": -2, "ztm": 3, "sl": 1}.items():
            tg.set(token, value, allow="soft-limit fence")
        tg.send_gcode("G21 G90 G17 G92 X0 Y0 Z0", check=True)
        tg.guard.check_gcode("G1 X20 Y20", allow="goose inside the fence")
        res = sender.stream(tg, lines, "line", bounds=box)
    finally:
        tg.send_gcode("G92.1")
        tg.send_json({"sr": False})
        tg.send_json({"sr": {k: True for k in original_sr}})
    cells = _index(path)
    samples = [(s["posx"], s["posy"]) for _, s in res.status if "posx" in s and "posy" in s]
    worst = sorted(((_distance(cells, p), p) for p in samples), reverse=True)[:5]
    record("e166", {"samples": len(samples), "worst_mm": [(round(d, 4), p) for d, p in worst]})
    assert res.acked == res.lines and not res.errors
    assert samples, "no status reports with a position"
    assert worst[0][0] <= TOL, f"a status report was {worst[0][0]:.4f} mm off the path at {worst[0][1]}"
