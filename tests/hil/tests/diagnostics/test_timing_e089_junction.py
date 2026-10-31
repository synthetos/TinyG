"""Timing gate E089: does the first move after idle slow down at its junction with a move queued behind it?

The planner can start executing the first block before the second line is parsed, so the first move
plans to stop at its end even though a collinear move follows within a millisecond or two. Two equal
+X moves are written in one serial write from idle; the Saleae records X (M1) step pulses, and the
step rate across the junction is compared with the cruise rate inside the first move. Read live
geometry; no ICE halts during the capture. The capture (.sal) and its raw edges are kept in the run
directory beside run.json (firmware and harness commits) for review and promotion.
"""
import json
from pathlib import Path
import time

import pytest

from tinyg_hil import saleae

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.dialect("d449")]

LENGTH = 10.0           # mm per move
FEED = 600.0            # mm/min
MOVE = "two collinear 10 mm +X moves from idle, then back"


def _bench(tg):
    config = json.loads((Path(__file__).resolve().parents[2] / "config/benches.json").read_text())
    bench = config.get(tg.transport.port) or {}
    if not bench.get("saleae"):
        pytest.skip(f"no Saleae recorded for {tg.transport.port}")
    chan = {v: int(k) for k, v in bench["saleae_channels"].items()}
    if "M1 step" not in chan:
        pytest.skip("M1 step is not on the analyzer")
    return bench, chan["M1 step"]


def _rate(rises, i, j):
    """Mean step rate (steps/s) over pulses i..j."""
    return (j - i) / (rises[j] - rises[i])


def test_first_move_keeps_speed_through_junction(tg, record, request):
    bench, step = _bench(tg)
    recorder = request.config._tinyg_recorder
    assert recorder, "run records required: the capture is the evidence"
    assert tg.get("1ma") == 0, "X must be motor 1"
    spm = 360.0 / tg.get("1sa") * tg.get("1mi") / tg.get("1tr")
    feed = min(FEED, tg.get("xvm"), tg.get("xfr"))
    tg.send_gcode("G21 G91 G94", check=True)
    tg.wait_idle()
    with saleae.Capture(bench["saleae"], channels=[step], seconds=None,
                        save_as=recorder.dir / "e089-junction") as cap:
        started = time.monotonic()
        time.sleep(0.2)
        tg.guard.check_gcode(f"G1 X{LENGTH:g} F{feed:g}", allow=MOVE)
        written = time.monotonic()
        tg.transport.write(f"G1 X{LENGTH:g} F{feed:g}\nG1 X{LENGTH:g}\n".encode())   # second queued at once
        for _ in range(2):                                   # their replies, before any getter
            tg.wait_for(lambda e: e.kind == "response", 5.0)
        tg.wait_idle(timeout=60)
        time.sleep(0.2)
    try:
        rises = saleae.rising(cap.edges()[step])
    finally:
        tg.send_gcode(f"G1 X{-2 * LENGTH:g} F{feed:g}", allow=MOVE)
        tg.wait_idle(timeout=60)
        tg.send_gcode("G90")
    per_move = round(LENGTH * spm)
    assert abs(len(rises) - 2 * per_move) <= 2, f"{len(rises)} pulses, expected {2 * per_move}"
    span = max(4, round(0.25 * spm))                        # 0.25 mm of pulses around each point
    cruise = _rate(rises, per_move // 2 - span, per_move // 2 + span)
    junction = _rate(rises, per_move - span, per_move + span)
    # narrowest point: the largest gap between consecutive pulses within 1 mm of the junction
    near = range(max(1, per_move - round(spm)), min(len(rises) - 1, per_move + round(spm)))
    worst_gap = max(rises[i] - rises[i - 1] for i in near)
    result = {"capture": "e089-junction.sal", "edges": "e089-junction-edges.json", "sample_rate": cap.rate,
              "step_channel": step,
              # host clock, not instrument time: when the two lines were written after the capture was started
              "host_write_after_capture_start_s": written - started, "first_step_s": rises[0],
              "junction_pulse_s": rises[per_move - 1] if len(rises) >= per_move else None,
              "steps_per_mm": spm, "feed_mm_min": feed, "pulses": len(rises),
              "cruise_mm_s": cruise / spm, "junction_mm_s": junction / spm,
              "min_junction_mm_s": 1 / worst_gap / spm, "junction_over_cruise": junction / cruise}
    record("e089_junction", result)
    assert result["min_junction_mm_s"] >= 0.8 * result["cruise_mm_s"], \
        f"slowed at the junction: {result}"
