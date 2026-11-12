"""E089 on the MSO: two collinear moves queued together after idle join at speed (step-interval evidence).

E089 (fixed on edge): the first move after idle was planned as the last block and frozen when it started, so a
second collinear move arriving a few ms later could not raise its exit velocity and the axis stopped at the
junction. The serial test samples `vel` and stays timing-pending; this one reads every step pulse.

Two +2 mm Y moves at F300 are written in one write after idle, with a low jerk (yjm=50) so that a stop at the
junction would last ~0.6 s and be obvious. Y is used because its motors run at 1280 steps/mm (X is M4 at
360 mm/rev on bench 3). The velocity is computed from step intervals on the first Y motor; around the junction
(2 mm, +/- 0.3 mm) it must stay above 80 % of the feed. Records the profile summary. Bench with an "mso" entry.
"""
import time

import pytest

from tinyg_hil import benches, mso

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.timing, pytest.mark.persists,
              pytest.mark.dialect("d449"), pytest.mark.timeout(300)]

F = 300.0
MOVE = "two collinear 2 mm +Y moves at F300 on the bring-up bench, then back"


def _y_motor(tg, cfg):
    for m in "1234":
        if tg.get(f"{m}ma") == 1 and f"M{m} step" in (cfg.get("digital") or {}).values():
            return m
    pytest.skip("no Y motor with a step pin on the MSO")


def test_e089_junction_velocity(tg, keep, record):
    cfg = (benches.bench(tg.transport.port) or {}).get("mso") or {}
    if not cfg.get("digital"):
        pytest.skip("no MSO on this bench")
    m = _y_motor(tg, cfg)
    spm = 360.0 * tg.get(f"{m}mi") / (tg.get(f"{m}sa") * tg.get(f"{m}tr"))
    keep("yjm")
    tg.set("yjm", 50, allow="low jerk so a junction stop would be long")
    tg.send_gcode("G21 G90 G64", check=True)
    tg.send_gcode("G28.3 Y0", check=True, allow="declare Y")
    tg.wait_idle()
    lines = [f"G91 G1 Y2 F{F:g}", "G1 Y2"]
    for line in lines:
        tg.guard.check_gcode(line, allow=MOVE)
    with mso.Capture(cfg, digital=[f"M{m} step"], seconds=6, rate=2_000_000) as cap:
        time.sleep(1.5)                                     # pre-roll: the capture starts after ARMED
        tg.transport.write("".join(line + "\n" for line in lines).encode())   # one write: both arrive together
        tg.wait_idle(timeout=20)
    rises = [t for t, v in cap.edges()[f"M{m} step"] if v == 1]
    tg.send_gcode("G90 G0 Y0", allow="return Y")
    tg.wait_idle(timeout=20)
    assert len(rises) > 2 * spm * 3.5, f"expected ~{4 * spm:.0f} steps, got {len(rises)}"
    vel = [(i, 60.0 / (spm * (rises[i] - rises[i - 1]))) for i in range(1, len(rises))]   # mm/min per step
    jn = int(2 * spm)
    window = [v for i, v in vel if abs(i - jn) <= 0.3 * spm]
    cruise = sorted(v for i, v in vel if 0.5 * spm < i < 3.5 * spm)
    summary = {"steps": len(rises), "steps_per_mm": spm, "junction_min": round(min(window), 1),
               "junction_mean": round(sum(window) / len(window), 1),
               "cruise_median": round(cruise[len(cruise) // 2], 1), "feed": F,
               "duration_s": round(rises[-1] - rises[0], 3)}
    record("e089_junction", summary)
    print("E089", summary)
    assert min(window) > 0.8 * F, f"slowed to {min(window):.0f} mm/min at the junction: {summary}"
