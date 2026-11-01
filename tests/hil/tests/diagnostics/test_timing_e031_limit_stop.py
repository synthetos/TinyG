"""Timing gate E031: how fast does X start braking, and how far does it run, after the X-min limit trips?

A firmware switch mount (test-hooks build, --ice) drives the X-min input low when the step count crosses
8 mm; the Saleae records that pin (ch0) with the M1 step pulses, so the trip edge and the steps are on
one instrument clock. The step positions after the trip are fitted with the firmware's own stop
profile (tinyg_hil.stopfit: segmented quintic tail, fitted cruise velocity, delay to the first tail
segment); single step intervals are not used, since DDA tick jitter is far larger than the first percent
of braking. The estimator's error on synthetic firmware-profile traces with DDA quantization is
STOPFIT_DELAY_ERROR_S / STOPFIT_JERK_REL_ERROR (tests/test_stopfit_offline.py). Measured: the fitted
delay and planned jerk, trip to last step, and the steps after the trip as overrun. Agreed behaviour
(maintainer, 2026-10-30): the switch ISR requests the hold and it brakes at the homing jerk (xjh, here
9 xjm, a third of the normal stopping distance). The capture (.sal) and raw edges are kept in the run
directory beside run.json.
"""
import json
from pathlib import Path
import time

import pytest

from tinyg_hil import benches, saleae, stopfit

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice, pytest.mark.motion, pytest.mark.dialect("d449")]

FEED = 600.0            # mm/min: cruise is reached long before the switch at these jerks
TRIP_MM = 8.0
XJH_OVER_XJM = 9
# When the switch trips, one segment is running and the next is already prepared (prep buffer owned by
# the loader); the stop is computed into the segment after that, so braking starts within the rest of the
# running segment plus one: at most two NOM_SEGMENT_USEC segments.
PIPELINE_S = 2 * stopfit.NOM_SEGMENT_S
STOPFIT_DELAY_ERROR_S = 0.0006          # measured by stopfit.validate (tests/test_stopfit_offline.py)
STOPFIT_JERK_REL_ERROR = 0.01
FIT_RMS_LIMIT_MM = 0.0005               # synthetic firmware-profile traces fit to <= 0.24 um


def _bench(tg):
    config = benches.load()
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
    tg.set("xjh", xjm * XJH_OVER_XJM)
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
    cruise = (before[-1] - before[-101]) / 100               # mean interval over the last 100 steps before it
    v = feed / 60.0
    jerk = xjm * 1e6 / 216000.0                              # mm/s^3
    normal = v * (v / jerk) ** 0.5                           # stop length at the move's jerk (mp_get_target_length)
    fv, delay, tail_s, rms = stopfit.fit([t - trip for t in after], v, spm)
    jerk_ratio = stopfit.jerk_eff(fv, tail_s) / jerk
    result = {"capture": "e031-limit-stop.sal", "edges": "e031-limit-stop-edges.json", "sample_rate": cap.rate,
              "channels": {"step": step, "x_min": xmin}, "steps_per_mm": spm, "feed_mm_min": feed, "xjm": xjm,
              "trip_s": trip, "cruise_interval_s": cruise, "cruise_mm_s": 1 / cruise / spm,
              "fit_cruise_mm_s": fv, "fit_delay_s": delay, "fit_tail_s": tail_s, "fit_jerk_over_xjm": jerk_ratio,
              "fit_rms_mm": rms, "fit_model": "stopfit segmented quintic", "xjh_over_xjm": XJH_OVER_XJM,
              "estimator_error": {"delay_s": STOPFIT_DELAY_ERROR_S, "jerk_rel": STOPFIT_JERK_REL_ERROR},
              "trip_to_last_step_s": after[-1] - trip, "steps_after_trip": len(after),
              "overrun_mm": len(after) / spm, "normal_stop_mm": normal, "fast_stop_mm": normal / XJH_OVER_XJM ** 0.5,
              "firmware_trip_at": status.get("trip_at")}
    record("e031_limit_stop", result)
    assert rms <= FIT_RMS_LIMIT_MM, f"the stop does not follow the firmware's tail profile: {result}"
    # the ISR requests the hold: braking starts within the segment pipeline (plus estimator error), at the
    # homing jerk (within estimator error and 5%), and runs out no further than the fast stop plus that delay
    assert delay <= PIPELINE_S + STOPFIT_DELAY_ERROR_S, f"slow to start braking: {result}"
    assert abs(jerk_ratio / XJH_OVER_XJM - 1) <= 0.05 + STOPFIT_JERK_REL_ERROR, f"braked at the wrong jerk: {result}"
    fast = normal / XJH_OVER_XJM ** 0.5
    assert result["overrun_mm"] <= fast + v * (PIPELINE_S + STOPFIT_DELAY_ERROR_S) + 2 / spm, f"overran: {result}"
