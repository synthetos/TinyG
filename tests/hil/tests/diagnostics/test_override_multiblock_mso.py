"""Bench 3 (MSO): does live feed override scale a whole multi-move job? (N111, GitHub #316)

Feed-Override wiki page: "The factor scales the programmed feed rate." So mfo=0.5 on a job must run like the same job
with every F word halved at mfo=1: acceleration, jerk, arc and junction limits apply the same way to both. (A plain
"twice as long" target is wrong: acceleration takes less time at half speed, and an arc already held below F by
its own limits at 100 % cannot take twice as long.) Job time is taken from the step pulses (first to last pulse on
the Y and X step lines). Patterns, relative moves, returned afterwards:

  long4      four collinear G1 Y20 F600 lines (blended)
  short200   200 G1 Y0.4 F600 lines (80 mm in 40 ms moves, like dense CAM output)
  arc        G2 X0 Y20 I0 J10 F600, a half circle of radius 10 mm
     each:   time(F600 at mfo 0.5) within 3 % of time(F300 at mfo 1); time at mfo 1 / F600 recorded for reference
  long4_mid  long4 at mfo 1, mfo 0.5 requested 1 s after the first line (all four blocks already queued):
             time ratio against mfo 1 within 5 % of 1.875, the Y speed in the last steady seconds at 300 mm/min
             (2 %), and no speed above 309 mm/min once the change has had 1.5 s to take effect (no surge back).

450.05 (strict xfail N111): each block restarts at the programmed feed, arcs and short blocks are not retimed.
"""
import re
import statistics
import time

import pytest

from tinyg_hil import benches, mso, sender

pytestmark = [pytest.mark.bug("N111", on=["d449"]), pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists,
              pytest.mark.dialect("d449"), pytest.mark.timeout(600)]

# lines, return lines, seconds at 100 %
PATTERNS = {
    "long4": (["G1 Y20 F600"] * 4, ["G0 Y-80"], 8.0),
    "short200": (["G1 Y0.4 F600"] * 200, ["G0 Y-80"], 8.0),
    "arc": (["G17 G2 X0 Y20 I0 J10 F600"], ["G0 Y-20"], 3.2),
}
MATCH = 0.03          # override vs halved-F reference
MID_IDEAL, MID_TOL = 1.875, 0.05
WINDOW = 32           # steps per speed sample


def _cfg(tg):
    cfg = (benches.bench(tg.transport.port) or {}).get("mso") or {}
    if not cfg.get("digital"):
        pytest.skip("needs the MSO (bench 3)")
    return cfg


def _spm(tg, m):
    return 360.0 * tg.get(f"{m}mi") / (tg.get(f"{m}sa") * tg.get(f"{m}tr"))


def halve_feeds(lines):
    return [re.sub(r"F([0-9]*\.?[0-9]+)", lambda m: f"F{float(m.group(1)) / 2:g}", ln) for ln in lines]


def _rises(edges):
    return [t for t, v in edges if v == 1]


def _speeds(rises, spm, k=WINDOW):
    out = []
    for i in range(0, len(rises) - k, max(1, k // 4)):
        dt = rises[i + k] - rises[i]
        if dt > 0:
            out.append(((rises[i] + rises[i + k]) / 2, k / spm / dt * 60.0))
    return out


def _run(tg, cfg, request, tag, lines, back, nominal_s, factor, mid=None):
    tg.set("mfo", factor)
    for ln in lines + back:
        tg.guard.check_gcode(ln, allow="override multi-move pattern")
    capture_s = min(60.0, 3.5 + nominal_s * 1.3 + 4)
    with mso.Capture(cfg, digital=["M2 step", "M4 step"], seconds=capture_s, rate=1_000_000,
                     save_as=request.config._tinyg_recorder.dir / f"multiblock-{tag}") as cap:
        time.sleep(3.5)
        if len(lines) <= 4:
            for ln in lines:                      # each waits for its own reply; the planner takes them at once
                tg.send_line(ln, check=True)
            if mid is not None:
                time.sleep(mid)
                assert tg.send_json({"mfo": 0.5}, check=True).status == 0
        else:
            assert mid is None
            res = sender.stream(tg, lines, "line", timeout=120)
            assert res.ok, f"stream failed: {res}"
        tg.wait_idle(timeout=120)
    tg.set("mfo", 1)
    for ln in back:
        tg.send_gcode(ln, check=True, allow="return to the start")
    tg.wait_idle(timeout=60)
    e = cap.edges()
    m2 = _rises(e["M2 step"])
    both = sorted(m2 + _rises(e["M4 step"]))
    job = (both[-1] - both[0]) if len(both) > 1 else None
    return {"job_s": round(job, 3) if job else None, "pulses": len(both)}, m2


@pytest.mark.parametrize("pattern", list(PATTERNS) + ["long4_mid"])
def test_override_multiblock(tg, keep, record, request, pattern):
    cfg = _cfg(tg)
    keep("ej", "rxm", "ex", "qv", "sv", "si", "xvm", "xfr", "yvm", "yfr")
    for k, v in dict(ej=1, rxm=1, ex=1, qv=1, sv=1, si=100, xvm=1200, xfr=1200, yvm=1200, yfr=1200).items():
        tg.set(k, v, allow="line-mode streaming and speed limits for the multi-move override measurement")
    for tok in ("mfoe", "m48e"):
        tg.set(tok, 1)
    tg.send_gcode("G21 G91 G94 G64 M48", check=True, allow="relative mode, override permission")
    lines, back, nominal = PATTERNS["long4" if pattern == "long4_mid" else pattern]
    try:
        full, _ = _run(tg, cfg, request, f"{pattern}-mfo1", lines, back, nominal, 1.0)
        if pattern == "long4_mid":
            over, m2 = _run(tg, cfg, request, f"{pattern}-mid", lines, back, nominal * 2, 1.0, mid=1.0)
        else:
            ref, _ = _run(tg, cfg, request, f"{pattern}-halfF", halve_feeds(lines), back, nominal * 2, 1.0)
            over, _ = _run(tg, cfg, request, f"{pattern}-mfo05", lines, back, nominal * 2, 0.5)
    finally:
        tg.set("mfo", 1)
        tg.send_gcode("G90", check=True)

    summary = {"pattern": pattern, "mfo1_F600": full, "with_override": over}
    if pattern == "long4_mid":
        spm = _spm(tg, 2)
        s = _speeds(m2, spm)
        t0, t_end = m2[0], m2[-1]
        steady = [v for t, v in s if t_end - 2.0 <= t <= t_end - 0.8]
        after = [v for t, v in s if t0 + 1.0 + 1.5 <= t <= t_end - 0.8]
        ratio = round(over["job_s"] / full["job_s"], 3) if full["job_s"] and over["job_s"] else None
        summary.update({"time_ratio": ratio, "ideal_ratio": MID_IDEAL,
                        "steady_end_mm_min": round(statistics.median(steady), 2) if steady else None,
                        "max_after_change_mm_min": round(max(after), 2) if after else None})
        record("override_multiblock", summary)
        print("MULTIBLOCK", summary)
        assert ratio is not None and abs(ratio - MID_IDEAL) <= MID_TOL * MID_IDEAL, f"mid-job change: {summary}"
        assert summary["steady_end_mm_min"] == pytest.approx(300, rel=0.02), f"not at 300 mm/min: {summary}"
        assert summary["max_after_change_mm_min"] <= 309, f"speed surged after the change: {summary}"
    else:
        match = round(over["job_s"] / ref["job_s"], 4) if ref["job_s"] and over["job_s"] else None
        summary.update({"halved_F_mfo1": ref, "override_vs_halved_F": match,
                        "ratio_vs_mfo1": round(over["job_s"] / full["job_s"], 3) if full["job_s"] else None})
        record("override_multiblock", summary)
        print("MULTIBLOCK", summary)
        assert match is not None and abs(match - 1) <= MATCH, f"mfo 0.5 does not run like halved F: {summary}"
