"""Soak (local): stream a program in one mode while the Saleae records the X/Y step and direction pulses.

The toolpath rebuilt from the pulses must stay on the commanded path and end where the program ends
(a lost or extra move shows as an offset), and the status reports must lie on the path too. Program and
mode come from SOAK_PROGRAM / SOAK_MODE (line, rts, count, noflow = line mode with flow control off).
SOAK_SETTINGS="1mi=8,2sa=0.9,..." applies motor geometry first (restored afterwards); steps per mm are then read
from the board, so a geometry the step generation does not honour rebuilds a path of the wrong size and fails.
"""
import math
import os
from collections import defaultdict

import pytest

from tinyg_hil import benches, gcodeplay, saleae, sender

pytestmark = [pytest.mark.timeout(900), pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449")]

CELL = 0.5                      # mm: grid cell for the nearest-segment search


class PathIndex:
    """Nearest distance from a point to the commanded polyline, via a uniform grid of segments."""

    def __init__(self, pts):
        self.cells = defaultdict(list)
        for a, b in zip(pts, pts[1:]):
            if a is None or b is None:
                continue
            x0, x1 = sorted((a[0], b[0]))
            y0, y1 = sorted((a[1], b[1]))
            for i in range(int(math.floor(x0 / CELL)) - 1, int(math.floor(x1 / CELL)) + 2):
                for j in range(int(math.floor(y0 / CELL)) - 1, int(math.floor(y1 / CELL)) + 2):
                    self.cells[(i, j)].append((a, b))

    def distance(self, p):
        best = float("inf")
        i, j = int(math.floor(p[0] / CELL)), int(math.floor(p[1] / CELL))
        for (ax, ay), (bx, by) in self.cells.get((i, j), ()):
            dx, dy = bx - ax, by - ay
            L = dx * dx + dy * dy
            t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
            best = min(best, math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy)))
        return best


def _motor_track(step, dirn, initial_dir, spmm, sign):
    """[(t, position mm)] at every step of one motor; direction sampled at each rising step edge."""
    out, pos, d, k = [], 0.0, initial_dir, 0
    for t in saleae.rising(step):
        while k < len(dirn) and dirn[k][0] <= t:
            d = dirn[k][1]
            k += 1
        pos += (1 if d else -1) * sign / spmm
        out.append((t, pos))
    return out


def _merge(xt, yt):
    """Interleave the two motors' steps in time into X/Y points."""
    pts, x, y, i, j = [], 0.0, 0.0, 0, 0
    while i < len(xt) or j < len(yt):
        if j >= len(yt) or (i < len(xt) and xt[i][0] <= yt[j][0]):
            x = xt[i][1]
            i += 1
        else:
            y = yt[j][1]
            j += 1
        pts.append((x, y))
    return pts


PROGRAM = os.environ.get("SOAK_PROGRAM", "goose")
MODE = os.environ.get("SOAK_MODE", "line")
TRUE_TOL = 0.015                # mm: chordal arcs (ct 0.01) plus one step
SR_TOL = 0.02


def test_soak_true_path(tg, keep, record, request):
    bench = benches.bench(tg.transport.port)
    if not bench.get("saleae"):
        pytest.skip(f"no Saleae for {tg.transport.port}")
    chan = {v: int(k) for k, v in bench["saleae_channels"].items()}
    geo = [kv.split("=") for kv in os.environ.get("SOAK_SETTINGS", "").split(",") if "=" in kv]
    for token, value in geo:
        keep(token)
        tg.set(token, float(value) if "." in value else int(value), allow="soak geometry")
    motors = {tg.get(f"{m}ma"): m for m in (1, 2, 3, 4)}           # axis index -> motor
    mx, my = motors.get(0), motors.get(1)
    if not (mx and my and f"M{mx} step" in chan and f"M{my} step" in chan):
        pytest.skip("X and Y motors must both be on the analyzer")
    spmm = {m: 360.0 / tg.get(f"{m}sa") * tg.get(f"{m}mi") / tg.get(f"{m}tr") for m in (mx, my)}
    keep("ej", "rxm", "ex", "qv", "sl", "xtn", "xtm", "ytn", "ytm", "ztn", "ztm")
    lines = gcodeplay.PROGRAMS[PROGRAM]()
    path = gcodeplay.path(lines)
    pts = [p for p in path if p is not None]
    box = ((min(p[0] for p in pts) - 2, max(p[0] for p in pts) + 2), (min(p[1] for p in pts) - 2, max(p[1] for p in pts) + 2))
    tg.transport.write(b'{"sr":null}\n')
    original_sr = list(tg.wait_for(lambda e: e.kind == "response" and "sr" in e.body, 3.0).body["sr"])
    channels = [chan[f"M{mx} step"], chan[f"M{mx} dir"], chan[f"M{my} step"], chan[f"M{my} dir"]]
    try:
        settings = {"rxm": 1, "ex": 0} if MODE == "noflow" else sender.MODES[MODE]
        for token, value in {"ej": 1, "qv": 1, **settings}.items():
            if tg.get(token) != value:
                tg.set(token, value, allow="soak")
        tg.send_json({"sr": {k: True for k in ("posx", "posy", "vel", "stat")}}, check=True)
        tg.send_gcode("G21 G90 G28.3 X0 Y0 Z0", check=True)
        for token, value in {"xtn": -2, "xtm": gcodeplay.SIZE + 2, "ytn": -2, "ytm": gcodeplay.SIZE + 2,
                             "ztn": -2, "ztm": 3, "sl": 1}.items():
            tg.set(token, value, allow="soft-limit fence")
        tg.send_gcode("G21 G90 G17 G92 X0 Y0 Z0", check=True)
        tg.guard.check_gcode("G1 X20 Y20", allow="goose inside the fence")
        stem = request.config._tinyg_recorder.dir / f"soak-{PROGRAM}-{MODE}"
        with saleae.Capture(bench["saleae"], channels=channels, seconds=None, rate=2_000_000, save_as=stem) as cap:
            res = sender.stream(tg, lines, "line" if MODE == "noflow" else MODE, bounds=box)
        edges = cap.edges()
        initial = cap.initial
    finally:
        tg.send_gcode("G92.1")
        tg.send_json({"sr": False})
        tg.send_json({"sr": {k: True for k in original_sr}})
    index = PathIndex(path)
    sr_points = [(s["posx"], s["posy"]) for _, s in res.status if "posx" in s and "posy" in s]
    sr_dev = sorted(((index.distance(p), p) for p in sr_points), reverse=True)[:5]
    # pick each motor's sign so the rebuilt path fits the commanded one
    best = None
    for sx in (1, -1):
        for sy in (1, -1):
            xt = _motor_track(edges[channels[0]], edges[channels[1]], initial[channels[1]], spmm[mx], sx)
            yt = _motor_track(edges[channels[2]], edges[channels[3]], initial[channels[3]], spmm[my], sy)
            track = _merge(xt, yt)
            sample = track[::max(1, len(track) // 400)]
            score = sum(index.distance(p) for p in sample) / max(1, len(sample))
            if best is None or score < best[0]:
                best = (score, sx, sy, track)
    _, sx, sy, track = best
    true_dev = sorted(((index.distance(p), p) for p in track[::2]), reverse=True)[:5]
    step_mm = max(1 / spmm[mx], 1 / spmm[my])
    end_cmd = pts[-1]
    end_off = math.hypot(track[-1][0] - end_cmd[0], track[-1][1] - end_cmd[1]) if track else None
    record("soak", {"program": PROGRAM, "mode": MODE, "end_off_mm": end_off and round(end_off, 4), "lines": res.lines, "acked": res.acked, "errors": res.errors[:3],
                    "steps_per_mm": spmm, "signs": [sx, sy], "true_points": len(track),
                    "true_worst_mm": [(round(d, 4), (round(p[0], 3), round(p[1], 3))) for d, p in true_dev],
                    "sr_worst_mm": [(round(d, 4), p) for d, p in sr_dev], "step_mm": round(step_mm, 4)})
    assert res.acked == res.lines and not res.errors
    assert true_dev[0][0] <= TRUE_TOL, f"the pulses left the path by {true_dev[0][0]:.4f} mm at {true_dev[0][1]}"
    assert end_off is not None and end_off <= 0.002 + step_mm, f"the pulses ended {end_off:.4f} mm from the program end"
    assert not sr_dev or sr_dev[0][0] <= SR_TOL, f"a status report was {sr_dev[0][0]:.4f} mm off the path"
