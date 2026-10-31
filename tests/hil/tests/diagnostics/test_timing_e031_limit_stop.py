"""Timing gate E031: how fast does X start braking, and how far does it run, after the X-min limit trips?

A firmware switch mount (test-hooks build, --ice) drives the X-min input low when the step count crosses
8 mm; the Saleae records that pin (ch0) with the M1 step pulses, so the trip edge and the steps are on
one instrument clock. The step positions after the trip are fitted with a jerk-limited stop from cruise
(a delay, then braking at a constant jerk); single step intervals are not used, since DDA tick jitter is
far larger than the first percent of braking. Measured: the fitted delay and jerk, trip to last step,
and the steps after the trip as overrun. Agreed behaviour
(maintainer, 2026-10-30): the switch ISR requests the hold and it brakes at the homing jerk (xjh, here
9 xjm, a third of the normal stopping distance). The capture (.sal) and raw edges are kept in the run
directory beside run.json.
"""
import json
import math
from pathlib import Path
import time

import pytest

from tinyg_hil import saleae

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice, pytest.mark.motion, pytest.mark.dialect("d449")]

FEED = 600.0            # mm/min: cruise is reached long before the switch at these jerks
TRIP_MM = 8.0
SEGMENT_S = 0.005       # NOM_SEGMENT_USEC: the running segment, the prepared one and the one the stop is
                        # computed into are already committed, so the stop can start no sooner than this:
PIPELINE_S = 3 * SEGMENT_S


def _stop_position(t, v, jerk):
    """Distance covered t seconds into a jerk-limited stop from v (no acceleration cap)."""
    total = 2 * math.sqrt(v / jerk)
    if t <= 0:
        return 0.0
    if t <= total / 2:
        return v * t - jerk * t ** 3 / 6
    return v * total / 2 - jerk * max(0.0, total - t) ** 3 / 6


def _fit_stop(times, spm, v, xjm_jerk):
    """Least-squares (delay s, jerk as a multiple of xjm) for step positions after the trip."""
    points = [(t, (i + 1) / spm) for i, t in enumerate(times)][::10]
    best = None
    for jm in [x / 10 for x in range(5, 151)]:
        for dm in range(0, 80):
            d, jerk = dm / 1000, xjm_jerk * jm
            err = sum((x - (v * min(t, d) + _stop_position(t - d, v, jerk))) ** 2 for t, x in points)
            if best is None or err < best[0]:
                best = (err, d, jm)
    return best[1], best[2], math.sqrt(best[0] / len(points))


def _bench(tg):
    config = json.loads((Path(__file__).resolve().parents[2] / "config/benches.json").read_text())
    bench = config.get(tg.transport.port) or {}
    if not bench.get("saleae"):
        pytest.skip(f"no Saleae recorded for {tg.transport.port}")
    chan = {v: int(k) for k, v in bench["saleae_channels"].items()}
    if "M1 step" not in chan or "X min" not in chan:
        pytest.skip("M1 step and X min must both be on the analyzer")
    return bench, chan["M1 step"], chan["X min"]


@pytest.mark.profile("switches-no")
@pytest.mark.settings(xsn=2, xsx=0)
def test_limit_trip_to_stop(tg, switches, keep, record, request):
    bench, step, xmin = _bench(tg)
    recorder = request.config._tinyg_recorder
    assert recorder, "run records required: the capture is the evidence"
    assert tg.get("1ma") == 0, "X must be motor 1"
    keep("xjh")
    xjm = tg.get("xjm")
    tg.set("xjh", xjm * 9)
    spm = 360.0 / tg.get("1sa") * tg.get("1mi") / tg.get("1tr")
    feed = min(FEED, tg.get("xvm"), tg.get("xfr"))
    tg.send_gcode("G28.3 X0", check=True)
    switches.mount("x", "min", at=-TRIP_MM)
    try:
        with saleae.Capture(bench["saleae"], channels=[xmin, step], seconds=None,
                            save_as=recorder.dir / "e031-limit-stop") as cap:
            time.sleep(0.2)
            tg.send_gcode(f"G91 G1 X-20 F{feed:g}", check=True, allow="X runs into a software X-min limit at 8 mm")
            end = time.monotonic() + 15
            while time.monotonic() < end and tg.stat() != tg.dialect.stat("ALARM"):
                time.sleep(0.1)
            time.sleep(0.5)                                  # braking has finished well before this
        edges = cap.edges()
        status = switches.mount_status("x", "min")
    finally:
        tg.clear_alarm()
        switches.unmount_all()
        tg.send_gcode("G91 G1 X20 F1200", allow="back off the limit")
        tg.wait_idle(timeout=30)
        tg.send_gcode("G90", check=True)
    falls = [t for t, level in edges[xmin] if level == 0]
    assert falls, "the X-min input never went low on the analyzer"
    trip = falls[0]
    rises = saleae.rising(edges[step])
    before = [t for t in rises if t < trip]
    after = [t for t in rises if t >= trip]
    assert len(before) > 20 and after, f"{len(before)} steps before the trip, {len(after)} after"
    cruise = (before[-1] - before[-11]) / 10                 # mean interval over the last 10 steps before it
    v = feed / 60.0
    jerk = xjm * 1e6 / 216000.0                              # mm/s^3
    normal = v * (v / jerk) ** 0.5                           # jerk-limited stop at the move's jerk
    delay, jerk_ratio, rms = _fit_stop([t - trip for t in after], spm, v, jerk)
    result = {"capture": "e031-limit-stop.sal", "edges": "e031-limit-stop-edges.json", "sample_rate": cap.rate,
              "channels": {"step": step, "x_min": xmin}, "steps_per_mm": spm, "feed_mm_min": feed, "xjm": xjm,
              "trip_s": trip, "cruise_interval_s": cruise, "cruise_mm_s": 1 / cruise / spm,
              "fit_delay_s": delay, "fit_jerk_over_xjm": jerk_ratio, "fit_rms_mm": rms, "xjh_over_xjm": 9,
              "trip_to_last_step_s": after[-1] - trip, "steps_after_trip": len(after),
              "overrun_mm": len(after) / spm, "normal_stop_mm": normal, "fast_stop_mm": normal / 3,
              "firmware_trip_at": status.get("trip_at")}
    record("e031_limit_stop", result)
    assert rms < 0.01, f"the stop does not fit a jerk-limited profile: {result}"
    # the ISR starts the hold: braking starts at the segment pipeline's floor (2 ms margin), at no less than
    # 90% of the homing jerk; the run-out is the fast-stop distance plus that delay at cruise and two steps
    assert delay <= PIPELINE_S + 0.002, f"slow to start braking: {result}"
    assert jerk_ratio >= 0.9 * 9, f"braked softer than xjh: {result}"
    assert result["overrun_mm"] <= normal / 3 + v * (PIPELINE_S + 0.002) + 2 / spm, f"overran: {result}"
