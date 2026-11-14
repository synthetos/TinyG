"""Bench 3 (MSO): does live feed override slow a whole multi-move job, or only part of it?

Each pattern runs twice, at mfo=1 and at mfo=0.5 (requested while idle, before the first line), streamed in line
mode, and the job time is taken from the step pulses (first to last pulse on the Y and X step lines). An override
that scales the whole job doubles the time (ratio 2.0). Patterns, relative moves, returned afterwards:

  long4      four collinear G1 Y20 F600 lines (blended, no stop between them), 8 s at 100 %
  short200   200 G1 Y0.4 F600 lines (80 mm in 40 ms moves, like dense CAM output), 8 s at 100 %
  arc        G2 X0 Y20 I0 J10 F600, a half circle of radius 10 mm, 3.1 s at 100 %
  long4_mid  long4 at mfo=1 with mfo=0.5 requested 1 s after streaming starts (ideal ratio 1.875)

Reading of plan_exec.c (450.05): each block starts at its planned (unity) speed and the factor is applied only in
the body of a straight G1 with more than OV_BODY_LEAD+1 segments left; arcs and short blocks are not retimed. This
test measures what that means for real jobs. Feed-Override wiki page: "The factor scales the programmed feed rate."
The test asserts the ideal ratio within 5 %; a pattern the implementation does not cover fails with its ratio.
"""
import time

import pytest

from tinyg_hil import benches, mso, sender

pytestmark = [pytest.mark.bug("N111", on=["d449"]), pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(400)]

# lines, return lines, seconds at 100 %
PATTERNS = {
    "long4": (["G1 Y20 F600"] * 4, ["G0 Y-80"], 8.0),
    "short200": (["G1 Y0.4 F600"] * 200, ["G0 Y-80"], 8.0),
    "arc": (["G17 G2 X0 Y20 I0 J10 F600"], ["G0 Y-20"], 3.2),
}


def _cfg(tg):
    cfg = (benches.bench(tg.transport.port) or {}).get("mso") or {}
    if not cfg.get("digital"):
        pytest.skip("needs the MSO (bench 3)")
    return cfg


def _job_seconds(cap):
    e = cap.edges()
    rises = sorted(t for ch in ("M2 step", "M4 step") for t, v in e[ch] if v == 1)
    return ((rises[-1] - rises[0]) if len(rises) > 1 else None), len(rises)


def _run(tg, cfg, request, name, lines, back, nominal_s, factor, mid=None):
    tg.set("mfo", factor)
    for ln in lines + back:
        tg.guard.check_gcode(ln, allow="override multi-move pattern")
    capture_s = min(60.0, 3.5 + nominal_s * 2.6 + 4)
    tag = f"multiblock-{name}-{factor:g}{'-mid' if mid else ''}"
    with mso.Capture(cfg, digital=["M2 step", "M4 step"], seconds=capture_s, rate=1_000_000,
                     save_as=request.config._tinyg_recorder.dir / tag) as cap:
        time.sleep(3.5)
        if len(lines) <= 4:
            # A few lines: each send waits for its own reply (the planner accepts them at once, so they still queue
            # back to back), and a mid-move request is an ordinary command that cannot take a stale reply.
            for ln in lines:
                tg.send_line(ln, check=True)
            if mid is not None:
                time.sleep(mid)
                assert tg.send_json({"mfo": 0.5}, check=True).status == 0
        else:
            assert mid is None, "mid-move requests are only used with the short patterns"
            res = sender.stream(tg, lines, "line", timeout=120)
            assert res.ok, f"stream failed: {res}"
        tg.wait_idle(timeout=120)
    tg.set("mfo", 1)
    for ln in back:
        tg.send_gcode(ln, check=True, allow="return to the start")
    tg.wait_idle(timeout=60)
    job, pulses = _job_seconds(cap)
    return {"job_s": round(job, 3) if job else None, "pulses": pulses, "capture_s": capture_s}


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
        base = _run(tg, cfg, request, pattern, lines, back, nominal, 1.0)
        if pattern == "long4_mid":
            over = _run(tg, cfg, request, pattern, lines, back, nominal, 1.0, mid=1.0)
            ideal = 1.875                          # 1 s at 100 %, the remaining 7 s of travel at 50 %
        else:
            over = _run(tg, cfg, request, pattern, lines, back, nominal, 0.5)
            ideal = 2.0
    finally:
        tg.set("mfo", 1)
        tg.send_gcode("G90", check=True)
    ratio = round(over["job_s"] / base["job_s"], 3) if base["job_s"] and over["job_s"] else None
    summary = {"pattern": pattern, "at_100": base, "with_override": over, "time_ratio": ratio, "ideal_ratio": ideal}
    record("override_multiblock", summary)
    print("MULTIBLOCK", summary)
    assert ratio is not None and abs(ratio - ideal) <= 0.05 * ideal, f"override did not scale the job: {summary}"
