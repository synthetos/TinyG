"""Stream whole G-code files with every flow-control scheme and check they run through.

For each mode in tinyg_hil.sender.MODES (packet line mode by free slots, g2core's line-mode protocol,
XON/XOFF in the tty and in the application, RTS/CTS, Grbl-style character counting) and each demo program in tinyg_hil.gcodeplay, the program is streamed from a work origin
set with G92 at the current position. Pass: every line is answered with status 0, no exception
report arrives, the machine stops back at the work origin, and every status-report position lies on
the commanded path (within PATH_TOL). The status-report trace is plotted against the commanded path
in the run directory as stream-<mode>-<program>.png. A stream that stalls (5 s with lines unanswered and
no status report) fails at once; with --ice the board's serial, planner and machine state is read over
the Atmel-ICE into the record first.
"""
import json
import sys
import time

import pytest

from tinyg_hil import gcodeplay, sender

pytestmark = [pytest.mark.streaming, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449")]

PATH_TOL = 0.05             # mm: SR positions print 3 decimals; arcs follow ct = 0.01 mm
SR_INTERVAL = 0.1           # s: si; coverage allows the distance two reports apart can span at top speed
SR_FIELDS = ["posx", "posy", "posz", "line", "stat", "vel"]


def _sr_list(tg):
    tg.transport.write(b'{"sr":null}\n')
    ev = tg.wait_for(lambda e: e.kind == "response" and isinstance(e.body, dict) and "sr" in e.body, 3.0)
    return list(ev.body["sr"])


def _plot(path, samples, title, dest):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    fig, ax = plt.subplots(figsize=(6, 6), layout="constrained")
    ax.plot([p[0] if p else float("nan") for p in path], [p[1] if p else float("nan") for p in path],
            color="#9aa5b1", lw=1, label="commanded path")
    ax.scatter([s[0] for s in samples], [s[1] for s in samples], s=6, color="#087c89", label="status reports")
    ax.set_aspect("equal"); ax.grid(alpha=.2); ax.legend(loc="upper right", fontsize=8)
    ax.set_title(title, fontsize=10); ax.set_xlabel("X (mm, work)"); ax.set_ylabel("Y (mm, work)")
    fig.savefig(dest, dpi=90); plt.close(fig)
    return dest.name


@pytest.mark.timeout(900)
@pytest.mark.parametrize("program", list(gcodeplay.PROGRAMS) + [
    pytest.param(name, marks=pytest.mark.large_travel) for name in gcodeplay.LARGE_PROGRAMS])
@pytest.mark.parametrize("mode", list(sender.MODES))
def test_stream_runs_through(tg, keep, record, request, mode, program):
    recorder = request.config._tinyg_recorder
    keep("ej", "rxm", "ex", "qv", "sl", "xtn", "xtm", "ytn", "ytm", "ztn", "ztm")
    original_sr = _sr_list(tg)
    if mode == "xon" and sys.platform == "darwin":
        pytest.skip("the macOS tty misses the board's XON and stops host output for good (xon-app covers XON/XOFF)")
    if program in gcodeplay.LARGE_PROGRAMS and not request.config.getoption("--large-travel"):
        pytest.skip("needs about 110 mm of free X/Y travel; pass --large-travel")
    lines = {**gcodeplay.PROGRAMS, **gcodeplay.LARGE_PROGRAMS}[program]()
    try:
        for token, value in {"ej": 1, "qv": 1, **sender.MODES[mode]}.items():
            if tg.get(token) != value:
                tg.set(token, value, allow="streaming mode")
        tg.send_json({"sr": {k: True for k in SR_FIELDS}}, check=True)
        if program in gcodeplay.PROGRAMS:
            # Soft limits as a backstop: mark the axes homed here (G28.3) and fence the program's box, so a
            # corrupted line that targets outside it alarms instead of moving (a 583 mm Y runaway was seen).
            tg.send_gcode("G21 G90 G28.3 X0 Y0 Z0", check=True)
            for token, value in {"xtn": -2, "xtm": gcodeplay.SIZE + 2, "ytn": -2, "ytm": gcodeplay.SIZE + 2,
                                 "ztn": -2, "ztm": 3, "sl": 1}.items():
                tg.set(token, value, allow="soft-limit fence for streaming")
        tg.send_gcode("G21 G90 G17 G92 X0 Y0 Z0", check=True)
        tg.guard.check_gcode("G1 X20 Y20", allow=f"stream {program}")
        pts = [p for p in gcodeplay.path(lines) if p is not None]
        box = ((min(p[0] for p in pts) - 2, max(p[0] for p in pts) + 2), (min(p[1] for p in pts) - 2, max(p[1] for p in pts) + 2))
        try:
            res = sender.stream(tg, lines, mode, bounds=box)
        except (sender.StreamStalled, sender.StreamRunaway):
            if request.config.getoption("--ice"):            # read the hung board's state before anything changes it
                record("stall_snapshot", sender.stall_snapshot(request.getfixturevalue("ice")))
            raise
        end = {t: tg.get(t) for t in ("posx", "posy", "posz", "stat")}
    finally:
        if tg.stat() == tg.dialect.stat("ALARM"):       # e.g. lost input (E162): restores are rejected while alarmed
            tg.clear_alarm()
        tg.send_gcode("G92.1")
        tg.send_json({"sr": False})
        tg.send_json({"sr": {k: True for k in original_sr}})
    path = gcodeplay.path(lines)
    samples = [(s["posx"], s["posy"]) for _, s in res.status if "posx" in s and "posy" in s]
    path_pts = [p for p in path if p is not None]
    scored = sorted(((gcodeplay.distance_to_path((st["posx"], st["posy"]), path), t - res.started, st)
                     for t, st in res.status if "posx" in st and "posy" in st), key=lambda r: -r[0])
    worst = scored[0][0] if scored else 0.0
    cover = gcodeplay.coverage(samples, path)
    top_mm_s = max((st.get("vel", 0) for _, st in res.status), default=0) / 60.0
    gap_limit = max(2.0, 3 * SR_INTERVAL * top_mm_s)
    plot = _plot(path, samples, f"{program} via {mode}: {res.acked}/{res.lines} lines, "
                 f"{res.finished - res.started:.1f} s", recorder.dir / f"stream-{mode}-{program}.png") if recorder else None
    result = {"mode": mode, "program": program, "lines": res.lines, "acked": res.acked, "errors": res.errors[:10],
              "exceptions": res.exceptions[:5], "seconds": round(res.finished - res.started, 2),
              "send_seconds": round(res.sent_done - res.started, 2), "max_inflight": res.max_inflight,
              "xoff_count": res.xoff_count, "min_free_slots": res.min_free, "sr_samples": len(samples),
              "lines_per_s": round(res.lines / max(1e-6, res.sent_done - res.started), 1),
              "planner_queue": sender.queue_profile(res),
              "peak_vel_mm_min": max((st.get("vel", 0) for _, st in res.status), default=0),
              "max_path_deviation_mm": round(worst, 4), "end": end, "plot": plot,
              "coverage": cover,
              "worst_samples": [{"dev": round(d, 4), "t": round(t, 3), "sr": st} for d, t, st in scored[:5]]}
    record("stream", result)
    assert res.acked == res.lines and not res.errors, f"{len(res.errors)} errors: {res.errors[:5]}"
    assert not res.exceptions, res.exceptions[:5]
    last = next(p for p in reversed(path) if p is not None)
    assert end["posx"] == pytest.approx(last[0], abs=0.01) and end["posy"] == pytest.approx(last[1], abs=0.01), (end, last)
    assert worst <= PATH_TOL, f"a status report was {worst:.3f} mm off the commanded path"
    assert cover.get("max_gap_mm", 0) <= gap_limit, \
        f"part of the path has no status report on it ({cover}, limit {gap_limit:.1f} mm): it did not run"
