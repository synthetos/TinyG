"""Bench 3 (MSO): live feed override measured on the real step train.

The override HIL suite checks the firmware's own reports (vel, mfc, mpox). This test checks the pulses: a long Y move
at F600, the factor changed mid-move, both Y step lines captured.

  factor 0.5 / 1.5    the cruise step rate after the change is factor x the rate before (1 %), the change itself
                      is a jerk-limited ramp no harder than the move's own acceleration, and each Y motor's signed
                      net travel is within 1 step of the commanded distance (pulses counted by DIR level, the
                      forward level taken from $<m>po and the firmware's DIR convention, tinyg_hil.override_steps).
  none / unity        controls: the same move with no request, and with mfo=1 sent mid-move.
  cap                 factor 2.0 with $yfr/$yvm at 800 mm/min: the rate stops at 800 mm/min and mcp reports the cap.

Feed-Override wiki page: "The factor scales the programmed feed rate. Axis and motor speed limits, acceleration and
jerk still apply"; "mcp: 1 if axis or motor speed limits capped the accepted speed".
Bench preconditions, checked: motors 2 and 3 both mapped to Y with equal scaling.
"""
import statistics
import time

import pytest

from tinyg_hil import benches, mso
from tinyg_hil.override_steps import dir_level, signed_steps, within

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(240)]

FEED = 600.0          # mm/min
DIST = 80.0           # mm
REQUEST_AT = 3.0      # s after the move line is sent; the 100 % cruise is reached well before this
WINDOW = 32           # steps per speed sample


def _cfg(tg):
    cfg = (benches.bench(tg.transport.port) or {}).get("mso") or {}
    if not cfg.get("digital"):
        pytest.skip("needs the MSO (bench 3)")
    return cfg


def _spm(tg, m=2):
    return 360.0 * tg.get(f"{m}mi") / (tg.get(f"{m}sa") * tg.get(f"{m}tr"))


def _rises(edges):
    return [t for t, v in edges if v == 1]


def speed_series(rises, spm, k=WINDOW):
    """(time, mm/min) from the time k steps take, sampled every k/4 steps."""
    out = []
    for i in range(0, len(rises) - k, max(1, k // 4)):
        dt = rises[i + k] - rises[i]
        if dt > 0:
            out.append(((rises[i] + rises[i + k]) / 2, k / spm / dt * 60.0))
    return out


def median_between(series, t0, t1):
    vals = [v for t, v in series if t0 <= t <= t1]
    return statistics.median(vals) if vals else None


def ramp_span(series, v_from, v_to, after=None):
    """Seconds the speed takes from 5 % to 95 % of the way from v_from to v_to (first such ramp after `after`)."""
    dv = v_to - v_from
    lo, hi = v_from + 0.05 * dv, v_from + 0.95 * dv
    up = dv > 0
    t_lo = t_hi = None
    for t, v in series:
        if after is not None and t < after:
            continue
        if t_lo is None and ((v >= lo) if up else (v <= lo)):
            t_lo = t
        if t_lo is not None and ((v >= hi) if up else (v <= hi)):
            t_hi = t
            break
    return (t_lo, t_hi)


def jerk_mm_s3(dv_mm_min, span):
    # A jerk-limited S-curve changes speed by dv in T = 2*sqrt(dv/J); its 5-95 % portion spans 0.684 T.
    if not span or span <= 0:
        return None
    return 4 * abs(dv_mm_min) / 60.0 / (span / 0.684) ** 2


CASES = {"none": dict(factor=None, limit=1200), "unity": dict(factor=1.0, limit=1200),
         "half": dict(factor=0.5, limit=1200), "one_and_half": dict(factor=1.5, limit=1200),
         "cap": dict(factor=2.0, limit=800)}


@pytest.mark.parametrize("case", list(CASES))
def test_override_step_rate(tg, keep, record, request, case):
    c = CASES[case]
    limit = c["limit"]
    send = c["factor"] is not None              # "none": no request at all, the control for step counts
    factor = c["factor"] if send else 1.0
    expected_after = min(FEED * factor, limit)
    cfg = _cfg(tg)
    if not (tg.get("2ma") == tg.get("3ma") == 1):
        pytest.skip("needs motors 2 and 3 both mapped to Y (bench 3 wiring)")
    spm, spm3 = _spm(tg, 2), _spm(tg, 3)
    if spm != spm3:
        pytest.skip(f"needs equal Y motor scaling (M2 {spm}, M3 {spm3} steps/mm)")
    fwd_level = {m: dir_level(True, tg.get(f"{m}po")) for m in (2, 3)}   # positive Y travel
    keep("yvm", "yfr")
    tg.set("yvm", limit, allow="Y speed limit for the override measurement")
    tg.set("yfr", limit, allow="Y feed limit for the override measurement")
    for tok in ("mfoe", "m48e"):
        assert tg.set(tok, 1) == 1
    tg.set("mfo", 1)
    tg.send_gcode("G21 G90 G94 G64 M48", check=True, allow="modes, program permission on")
    tg.send_gcode("G28.3 Y0", check=True, allow="declare Y")
    line = f"G91 G1 Y{DIST:g} F{FEED:g}"
    tg.guard.check_gcode(line, allow="override step-rate move")
    reply = None
    try:
        with mso.Capture(cfg, digital=["M2 step", "M2 dir", "M3 step", "M3 dir"], seconds=24, rate=2_000_000,
                         save_as=request.config._tinyg_recorder.dir / f"override-{case}") as cap:
            time.sleep(3.5)
            tg.send_line(line)
            sent = time.monotonic()
            time.sleep(REQUEST_AT)
            reply = tg.send_json({"mfo": factor}, check=True) if send else None
            requested = time.monotonic() - sent
            time.sleep(0.8)
            mfc, mcp = float(tg.get("mfc")), tg.get("mcp")
            tg.wait_idle(timeout=30)
    finally:
        tg.set("mfo", 1)
        tg.send_gcode("G90", check=True)
    tg.send_gcode("G0 Y0", allow="return Y")
    tg.wait_idle(timeout=60)

    e = cap.edges()
    m2, m3 = _rises(e["M2 step"]), _rises(e["M3 step"])
    # Signed travel: the move can end with a pulse after DIR flips (one reverse step), so total edges are not
    # travel. Each pulse counts by the DIR level at that moment against the forward level of a positive Y move;
    # travel in the wrong direction comes out negative. Speeds use forward pulses only.
    dir_flips = {k: len(e[k]) for k in ("M2 dir", "M3 dir")}
    ini = cap.initial()
    assert "M2 dir" in ini and "M3 dir" in ini, f"capture has no initial DIR level: {ini}"
    net_m2, back_m2, fwd_m2 = signed_steps(m2, e["M2 dir"], ini["M2 dir"], fwd_level[2])
    net_m3, back_m3, _ = signed_steps(m3, e["M3 dir"], ini["M3 dir"], fwd_level[3])
    if not fwd_m2:
        pytest.fail(f"no forward Y pulses on M2 (net {net_m2}, reverse {back_m2})")
    first_gaps_ms = [round((b - a) * 1000, 3) for a, b in zip(fwd_m2[:4], fwd_m2[1:4])]
    expected_steps = round(DIST * spm)
    s = speed_series(fwd_m2, spm)
    t0, t_end = fwd_m2[0], fwd_m2[-1]
    v_before = median_between(s, t0 + 1.2, t0 + REQUEST_AT - 0.3)
    v_after = median_between(s, t_end - 2.0, t_end - 0.8)
    acc = ramp_span(s, 0.0, v_before)
    chg = ramp_span(s, v_before, v_after, after=t0 + 1.2) if abs(v_after - v_before) > 0.02 * v_before else (None, None)
    j_acc = jerk_mm_s3(v_before, acc[1] - acc[0] if None not in acc else None)
    j_chg = jerk_mm_s3(v_after - v_before, chg[1] - chg[0] if None not in chg else None)
    summary = {"case": case, "factor": factor, "limit_mm_min": limit, "steps_m2": len(m2), "steps_m3": len(m3), "net_m2": net_m2, "net_m3": net_m3,
               "reverse_steps": [back_m2, back_m3], "forward_dir_level": fwd_level,
               "expected_steps": expected_steps, "steps_per_mm": round(spm, 3),
               "rate_before_hz": round(v_before / 60 * spm, 1) if v_before else None,
               "rate_after_hz": round(v_after / 60 * spm, 1) if v_after else None,
               "speed_before_mm_min": round(v_before, 2) if v_before else None,
               "speed_after_mm_min": round(v_after, 2) if v_after else None,
               "expected_after_mm_min": expected_after, "ratio": round(v_after / v_before, 4) if v_before else None,
               "change_starts_s_after_move": round(chg[0] - t0, 3) if chg[0] else None,
               "request_sent_s_after_line": round(requested, 3),
               "change_ramp_s": round(chg[1] - chg[0], 4) if None not in chg else None,
               "accel_ramp_s": round(acc[1] - acc[0], 4) if None not in acc else None,
               "jerk_change_mm_s3": round(j_chg, 1) if j_chg else None,
               "jerk_accel_mm_s3": round(j_acc, 1) if j_acc else None,
               "change_vs_accel_jerk": round(j_chg / j_acc, 3) if j_chg and j_acc else None,
               "move_s": round(t_end - t0, 3), "dir_edges_during_capture": dir_flips, "first_step_gaps_ms": first_gaps_ms, "mfc": mfc, "mcp": mcp, "reply_status": reply.status if reply else None}
    record("override_step_rate", summary)
    print("OVERRIDE", summary)

    # Agreed bound: signed net travel within 1 step of the target on each motor (the starting substep phase can
    # move it by one); a move in the wrong direction is about -target and fails.
    assert within(net_m2, expected_steps), f"M2 net travel is wrong: {summary}"
    assert within(net_m3, expected_steps), f"M3 net travel is wrong: {summary}"
    assert v_before == pytest.approx(FEED, rel=0.01), f"cruise before the change is not F{FEED:g}: {summary}"
    assert v_after == pytest.approx(expected_after, rel=0.01), f"cruise after the change is wrong: {summary}"
    assert mfc == pytest.approx(factor), summary
    assert bool(mcp) == (FEED * factor > limit), f"mcp does not match the cap: {summary}"
    if expected_after == FEED:
        return                                   # no speed change to judge
    assert summary["change_vs_accel_jerk"] is not None and summary["change_vs_accel_jerk"] <= 1.3, \
        f"the speed change was harsher than the move's own acceleration: {summary}"
