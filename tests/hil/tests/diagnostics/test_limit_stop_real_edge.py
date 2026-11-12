"""Bench 3 (switchbox + MSO): a limit switch closed by a real electrical edge during a move, measured on the pins.

What the wiki promises (Homing and Limits; release notes E031): a limit hit stops the machine fast, keeps position
(no lost steps; the axes are only marked unhomed) and enters ALARM. Until now that was checked with the ICE's software
switch and step counter. Here the switchbox closes the A min input (used as a limit, $asn=2) for 100 ms while Y moves
at F1200 on both Y motors (M2, M3). The MSO records the A min line (analog) and both Y step lines. Recorded:
  - edge-to-first-slowdown latency and edge-to-last-step time;
  - stop distance after the edge, against the jerk-limited distance at the homing jerk ($yjh) and at $yjm;
  - M2 and M3 step counts (one axis, two motors: they must match);
  - the firmware's reported Y travel against the real steps (position kept).
Asserts only what the docs promise: the machine stops, alarms, and the two motors and the report agree with the steps.
"""
import time

import pytest

from tinyg_hil import benches, mso
from tinyg_hil.switchbox import SwitchBox

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(300)]

F = 1200.0
MOVE = "G91 G1 Y30 F1200"


def _rises(edges):
    return [t for t, v in edges if v == 1]


def test_limit_stop_on_a_real_edge(tg, keep, record, request):
    bench = benches.bench(tg.transport.port) or {}
    cfg, sbcfg = bench.get("mso") or {}, bench.get("switchbox") or {}
    if not cfg.get("analog") or not sbcfg.get("inputs", {}).get("amin"):
        pytest.skip("needs the MSO with the A min line on an analog channel and a switchbox driving A min")
    amin_ch = next(k for k, v in cfg["analog"].items() if "min" in v.lower() or "switch" in v.lower())
    spm = 360.0 * tg.get("2mi") / (tg.get("2sa") * tg.get("2tr"))
    keep("asn")
    tg.set("asn", 2, allow="A min as a limit input for this test")
    jm, jh = tg.get("yjm"), tg.get("yjh")
    tg.send_gcode("G21 G90", allow="units")
    tg.send_gcode("G28.3 Y0", allow="declare Y")
    tg.guard.check_gcode(MOVE, allow="limit stop test move")
    y0 = tg.get("mpoy")
    sb = SwitchBox(sbcfg["port"], lines=sbcfg.get("lines", "asserted"), allow_reset=True).connect()
    stem = request.config._tinyg_recorder.dir / "limit-real-edge"
    try:
        sb.release_all()
        with mso.Capture(cfg, digital=["M2 step", "M3 step"], analog=[amin_ch], seconds=10, rate=1_000_000,
                         save_as=stem) as cap:
            time.sleep(3.5)                                   # data starts about 2 s after STARTED
            tg.send_line(MOVE)
            time.sleep(0.7)                                   # past the acceleration
            sent_at = time.time()
            sb.pulse_channel(sbcfg["inputs"]["amin"], ms=100)
            time.sleep(1.5)
        stat = tg.stat()
        y1 = tg.get("mpoy")
    finally:
        sb.release_all()
        sb.disconnect()
    edges = cap.edges()
    m2, m3 = _rises(edges["M2 step"]), _rises(edges["M3 step"])
    import numpy as np                                        # noqa: F401  (only to fail clearly if absent)
    trace = np.load(cap.analog_path(amin_ch)) if hasattr(cap, "analog_path") else None
    rate = cap.result["rate"]
    low = np.flatnonzero(trace < 1.5) if trace is not None else []
    t_edge = (low[0] / rate) if len(low) else None
    summary = {"steps_m2": len(m2), "steps_m3": len(m3), "steps_per_mm": spm, "feed": F, "yjm": jm, "yjh": jh,
               "stat_after": stat, "reported_travel_mm": round(y1 - y0, 4), "edge_s": t_edge}
    assert m2, f"no M2 steps captured: {summary}"
    summary["step_travel_mm"] = round(len(m2) / spm, 4)
    if t_edge is not None:
        before = [b - a for a, b in zip(m2, m2[1:]) if b < t_edge][-200:]
        cruise_iv = sorted(before)[len(before) // 2] if before else None
        after = [t for t in m2 if t >= t_edge]
        slow = next((b for a, b in zip(after, after[1:]) if cruise_iv and (b - a) > 1.10 * cruise_iv), None)
        v = 1.0 / (spm * cruise_iv) if cruise_iv else None    # mm/s
        def dist(j):                                          # jerk-limited stop from v: v * sqrt(v / J), J in mm/s^3
            J = j * 1e6 / 216000.0
            return v * (v / J) ** 0.5 if v else None
        summary.update({"cruise_mm_s": round(v, 3) if v else None, "steps_after_edge": len(after),
                        "stop_distance_mm": round(len(after) / spm, 4),
                        "latency_to_slowdown_ms": round((slow - t_edge) * 1e3, 3) if slow else None,
                        "edge_to_last_step_ms": round((after[-1] - t_edge) * 1e3, 3) if after else 0.0,
                        "jerk_stop_mm_at_yjh": round(dist(jh), 4) if v else None,
                        "jerk_stop_mm_at_yjm": round(dist(jm), 4) if v else None})
    record("limit_real_edge", summary)
    print("LIMIT", summary)
    tg.clear_alarm()
    tg.send_gcode("G90 G0 Y0", allow="return Y")
    tg.wait_idle(timeout=30)
    assert t_edge is not None, f"the A min line never went low on the MSO: {summary}"
    assert stat == tg.dialect.stat("ALARM"), f"a limit hit must alarm: {summary}"
    assert len(m2) == len(m3), f"the two Y motors stepped differently: {summary}"
    assert abs(summary["reported_travel_mm"] - summary["step_travel_mm"]) <= 1.5 / spm, \
        f"reported position differs from the real steps: {summary}"
