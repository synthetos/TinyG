"""Throughput benchmark: stream gcodeplay.perfbench() in line mode and record the cutting time.

PERF_JERK (default 20) sets xjm/yjm/zjm for the run; PERF_TAG (default A) and PERF_SLOT (default 1) name the
capture so tools/perf/paired.py can pair A/B runs: <PERF_PREFIX>-<TAG>-j<JERK>-<SLOT>-edges.json in the run
directory (bench 2, Saleae on M1..M3 step/dir). Recorded for every bench: wall time from the first move to idle
(serial side), lines, and the final position error; with the Saleae also net steps per motor (must be 0, the
program ends where it starts) and the shortest step pulse per motor. Smooth, fast motion is the product: a
merge that costs cutting time or shortens pulses below the base build must be explained before it ships.
"""
import os
import time

import pytest

from tinyg_hil import benches, gcodeplay, saleae, sender

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(600)]

JERK = float(os.environ.get("PERF_JERK", "20"))
TAG = os.environ.get("PERF_TAG", "A")
SLOT = os.environ.get("PERF_SLOT", "1")
PREFIX = os.environ.get("PERF_PREFIX", "G")


def test_perfbench(tg, keep, record, request):
    keep("ej", "qv", "rxm", "ex", "sv", "si", "xjm", "yjm", "zjm")
    for tok, val in (("ej", 1), ("qv", 1), ("sv", 1), ("si", 250), ("rxm", 1), ("ex", 1),
                     ("xjm", JERK), ("yjm", JERK), ("zjm", JERK)):
        tg.set(tok, val, allow="perfbench settings")
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    lines = gcodeplay.perfbench()
    tg.guard.check_gcode("G1 X20 Y20", allow="perfbench")
    bench = benches.bench(tg.transport.port)
    chan = {v: int(k) for k, v in (bench.get("saleae_channels") or {}).items()}
    use_saleae = bool(bench.get("saleae")) and all(f"M{m} {k}" in chan for m in (1, 2, 3) for k in ("step", "dir"))
    info = {"jerk": JERK, "tag": TAG, "slot": SLOT, "lines": len(lines), "saleae": use_saleae}
    stem = request.config._tinyg_recorder.dir / f"{PREFIX}-{TAG}-j{int(JERK)}-{SLOT}"

    def run():
        t0 = time.monotonic()
        res = sender.stream(tg, lines, "line", timeout=500)
        tg.wait_idle(timeout=120)
        info["wall_s"] = round(time.monotonic() - t0, 3)
        info["acked"] = res.acked
        info["errors"] = res.errors[:5]
        return res

    if use_saleae:
        channels = [chan[f"M{m} {k}"] for m in (1, 2, 3) for k in ("step", "dir")]
        with saleae.Capture(bench["saleae"], channels=channels, seconds=None, rate=12_000_000, save_as=stem) as cap:
            run()
        edges, initial = cap.edges(), cap.initial
        info["edges_json"] = str(stem) + "-edges.json"
        net, pulse = {}, {}
        for m in (1, 2, 3):
            s, d = chan[f"M{m} step"], chan[f"M{m} dir"]
            lvl = initial.get(d, 0) if isinstance(initial, dict) else 0
            rises = saleae.rising(edges.get(s, []))
            dirs = edges.get(d, [])
            # net steps: +1 or -1 per rising step edge by the direction level at that time
            j, n = 0, 0
            for t in rises:
                while j < len(dirs) and dirs[j][0] <= t:
                    lvl = dirs[j][1]; j += 1
                n += 1 if lvl else -1
            net[f"M{m}"] = n
            highs = [b[0] - a[0] for a, b in zip(edges.get(s, []), edges.get(s, [])[1:]) if a[1] == 1]
            pulse[f"M{m}_min_us"] = round(min(highs) * 1e6, 3) if highs else None
        info["net_steps"], info["pulse"] = net, pulse
    else:
        run()
    info["end"] = {a: tg.get(f"pos{a}") for a in "xyz"}
    record("perfbench", info)
    assert info["acked"] == len(lines) and not info["errors"], f"stream did not complete cleanly: {info}"
    assert all(abs(v) <= 0.002 for v in info["end"].values()), f"did not return to the origin: {info}"
    if use_saleae:
        assert all(abs(v) <= 1 for v in info["net_steps"].values()), f"net steps not zero: {info}"
