"""Diagnostic: are the Saleae channels wired to this board's step/dir pins as config/benches.json says?

One capture (left open in Logic 2 for a person to inspect): each motor in turn moves forward and
back. Its step channel must pulse about the step count its geometry predicts each way, every other
step channel must stay quiet meanwhile, and its dir channel must be at different levels during its
two bursts. Results are recorded per motor; the test fails if any motor is miswired.
"""
import json
import pathlib
import time

import pytest

from tinyg_hil import saleae

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists]

BENCHES = json.loads((pathlib.Path(__file__).resolve().parents[2] / "config" / "benches.json").read_text())
STEPS = {"linear": 400, "rotary": 100}      # target pulses per direction


def test_step_dir_wiring(tg, record, keep):
    bench = BENCHES.get(tg.transport.port) or {}
    if not bench.get("saleae"):
        pytest.skip(f"no Saleae recorded for {tg.transport.port} in config/benches.json")
    names = {int(k): v for k, v in bench["saleae_channels"].items()}
    chan = {v: k for k, v in names.items()}
    tg.send_gcode("G21 G91", check=True)
    plan = []
    for motor in (1, 2, 3, 4):
        if f"M{motor} step" not in chan:          # not wired to the analyzer on this bench
            continue
        axis = "xyzabc"[int(tg.get(f"{motor}ma"))]
        mode = tg.get(f"{axis}am")
        if mode == 0:
            continue
        if axis in "abc" and mode != 1:           # radius/slave modes scale A moves: use plain degrees
            keep(f"{axis}am")
            tg.set(f"{axis}am", 1)
        spu = 360.0 / tg.get(f"{motor}sa") * tg.get(f"{motor}mi") / tg.get(f"{motor}tr")
        steps = STEPS["rotary" if axis in "abc" else "linear"]
        dist = steps / spu
        feed = min(tg.get(f"{axis}fr"), tg.get(f"{axis}vm"), 60.0 * dist / 0.3)
        plan.append((motor, axis, steps, dist, feed))
    with saleae.Capture(bench["saleae"], channels=sorted(names), seconds=None, keep_open=True) as cap:
        time.sleep(0.5)
        for motor, axis, steps, dist, feed in plan:
            for d in (dist, -dist):
                tg.send_gcode(f"G1 {axis.upper()}{d:.4f} F{feed:.1f}", check=True, allow="wiring check")
                tg.wait_idle(timeout=60.0 * abs(d) / feed + 30)
                time.sleep(0.3)
        time.sleep(0.3)
    edges = cap.edges()
    # motors run one after another, so each motor's own step pulses mark its time span
    results, bad = {}, []
    for motor, axis, steps, dist, feed in plan:
        pulses = saleae.rising(edges[chan[f"M{motor} step"]])
        levels, others = None, {}
        dir_unmonitored = f"M{motor} dir" not in chan
        if len(pulses) > 2:
            lo, hi = pulses[0], pulses[-1]
            others = {names[c]: n for c in sorted(names) if names[c].endswith("step") and c != chan[f"M{motor} step"]
                      for n in [len(saleae.rising(edges[c], lo, hi))] if n}
            gap = max(range(1, len(pulses)), key=lambda i: pulses[i] - pulses[i - 1])
            d = chan.get(f"M{motor} dir")
            if d is not None:
                levels = (saleae.level_at(cap.initial[d], edges[d], pulses[gap // 2]),
                          saleae.level_at(cap.initial[d], edges[d], pulses[gap + (len(pulses) - gap) // 2]))
        dir_ok = dir_unmonitored or (levels is not None and levels[0] != levels[1])
        ok = len(pulses) >= 2 * steps - 2 and not others and dir_ok
        results[f"M{motor}"] = {"axis": axis, "expected_each_way": steps, "step_pulses": len(pulses),
                                "dir_levels_fwd_back": levels, "other_steps_during": others, "ok": bool(ok)}
        if not ok:
            bad.append(f"M{motor}")
    record("wiring", results)
    assert not bad, f"miswired or silent: {bad}: {json.dumps({m: results[m] for m in bad})}"
