"""Capture feed-limit push frames for host test vectors: one feed-limited job and one starving job.

Both stream the dense perfbench section (0.04 mm segments) in line mode with the push on ({"rt":{"fi":1}}, si=100,
sv=1, qv=1). The feed-limited run sends as fast as the slots allow; the starving run throttles the host to one line
every FEED_STARVE_MS ms (default 80) so the planner runs dry. Every frame the board sends is recorded in order with
its arrival time (push fs/fl, status reports, queue reports, responses, heartbeat) under "frames"; the test also
writes <run dir>/feed-vectors-<case>.jsonl, one frame per line, for replay by a host's fake serial.
"""
import json
import os
import time

import pytest

from tinyg_hil import gcodeplay, sender

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(900)]

STARVE_MS = int(os.environ.get("FEED_STARVE_MS", "80"))


def _lines():
    out = ["G21 G90 G17 G94", "G0 X0 Y0 Z0", "G1 Z0 F1200"]
    out += [l for l in gcodeplay.perfbench() if l.startswith(("G1", "G0", "G2", "G3"))][200:900]
    return out + ["G0 X0 Y0", "G0 Z0"]


def _capture(tg, request, case, throttle_ms):
    tr = tg.transport
    frames = []
    t0 = time.monotonic()

    def collect():
        while True:
            ev = tr.next_event(0.0)
            if ev is None:
                return
            frames.append({"t": round(ev.t - t0, 4), "kind": ev.kind, "raw": ev.raw})

    tg.send_line('{"rt":{"fi":0}}'); tg.send_line('{"rt":{"fi":1}}'); tg.send_line('{"rt":{"hb":1}}')
    collect()
    lines = _lines()
    tg.guard.check_gcode("G1 X20 Y20", allow="feed vectors")
    if throttle_ms == 0:
        start = len(tr.history)                           # the sender consumes events; the transport keeps them all
        res = sender.stream(tg, lines, "line", timeout=600)
        frames.extend({"t": round(ev.t - t0, 4), "kind": ev.kind, "raw": ev.raw} for ev in tr.history[start:])
        acked = res.acked
    else:
        acked = 0
        for l in lines:                                   # starve: one line per throttle period, replies read as they come
            tr.write((l + "\n").encode())
            end = time.monotonic() + throttle_ms / 1000.0
            while time.monotonic() < end:
                ev = tr.next_event(0.01)
                if ev is not None:
                    frames.append({"t": round(ev.t - t0, 4), "kind": ev.kind, "raw": ev.raw})
                    if ev.kind == "response":
                        acked += 1
    tg.wait_idle(timeout=120)
    time.sleep(2.5)                                       # the final summary and a heartbeat or two
    collect()
    tg.send_line('{"rt":{"fi":0}}'); tg.send_line('{"rt":{"hb":0}}')
    tg.send_gcode("G90", check=True)
    rec = request.config._tinyg_recorder
    path = None
    if rec:
        path = str(rec.dir / f"feed-vectors-{case}.jsonl")
        with open(path, "w") as f:
            for fr in frames:
                f.write(json.dumps(fr) + "\n")
    kinds = {}
    for fr in frames:
        kinds[fr["kind"]] = kinds.get(fr["kind"], 0) + 1
    return {"case": case, "lines": len(lines), "acked": acked, "frames": len(frames), "kinds": kinds, "file": path,
            "fs_examples": [fr["raw"][:200] for fr in frames if fr["raw"].startswith('{"fs"')][:3],
            "fl_examples": [fr["raw"][:200] for fr in frames if fr["raw"].startswith('{"fl"')][:3]}


def test_feed_vectors_limited(tg, keep, record, request):
    keep("ej", "qv", "rxm", "ex", "sv", "si")
    for tok, val in (("ej", 1), ("qv", 1), ("sv", 1), ("si", 100), ("rxm", 1), ("ex", 1)):
        tg.set(tok, val, allow="line mode with reports")
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    info = _capture(tg, request, "limited", 0)
    record("feed_vectors", info)
    assert info["frames"] > 0 and info["fs_examples"], f"no feed-limit frames captured: {info}"


def test_feed_vectors_starving(tg, keep, record, request):
    keep("ej", "qv", "rxm", "ex", "sv", "si")
    for tok, val in (("ej", 1), ("qv", 1), ("sv", 1), ("si", 100), ("rxm", 1), ("ex", 1)):
        tg.set(tok, val, allow="line mode with reports")
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    info = _capture(tg, request, "starving", STARVE_MS)
    record("feed_vectors", info)
    assert info["frames"] > 0, f"nothing captured: {info}"
