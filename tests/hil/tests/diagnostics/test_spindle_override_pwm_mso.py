"""Bench 3 (MSO): live spindle speed override measured on the spindle PWM line.

With known PWM settings (speed 0..10000, duty 0.10..0.90, off duty 0) the duty cycle for a speed S is
0.10 + 0.80 * clamp(S, 0, 10000) / 10000 (spindle.c cm_get_spindle_pwm). Sequence, about 2 s each, no motion:

  M3 S6000        sso 1    -> 0.580
  sso 0.5                  -> 0.340   (S x 0.5 = 3000)
  sso 2.0                  -> 0.900   (12000 clamped to 10000; scp reports the clamp)
  M5                       -> 0       (off, whatever the factor)
  sso 1 while off          -> 0       (a factor never turns the spindle on)
  M3                       -> 0.580   (back on at S x 1)

Feed-Override wiki page (spindle section to come) and g2core sso/ssoe: the factor scales the programmed S, clamped
to the configured range; M5 and alarms stop the spindle regardless. Each plateau is the median duty of the
analyser's 0.1 s PWM windows inside the step, compared with the expected duty within 0.01.
"""
import time

import pytest

from tinyg_hil import benches, mso

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(180)]

PWM = dict(p1csl=0, p1csh=10000, p1cpl=0.10, p1cph=0.90, p1pof=0.0)
STEP_S = 2.0


def duty(s):
    s = min(max(s, PWM["p1csl"]), PWM["p1csh"])
    return PWM["p1cpl"] + (PWM["p1cph"] - PWM["p1cpl"]) * (s - PWM["p1csl"]) / (PWM["p1csh"] - PWM["p1csl"])


def test_spindle_override_pwm(tg, keep, record, request):
    cfg = (benches.bench(tg.transport.port) or {}).get("mso") or {}
    pwm_ch = next((k for k, v in (cfg.get("analog") or {}).items() if "pwm" in v.lower()), None)
    if pwm_ch is None:
        pytest.skip("needs the MSO with the spindle PWM line (bench 3)")
    keep(*PWM)
    for k, v in PWM.items():
        tg.set(k, v, allow="known spindle PWM mapping for the override measurement")
    for tok in ("ssoe", "m48e"):
        tg.set(tok, 1)
    tg.set("sso", 1)
    tg.send_gcode("M5", check=True, allow="spindle off before the sequence")
    tg.send_gcode("M48", check=True)
    steps = [("M3 S6000", None, duty(6000)), (None, 0.5, duty(3000)), (None, 2.0, duty(12000)),
             ("M5", None, PWM["p1pof"]), (None, 1.0, PWM["p1pof"]), ("M3", None, duty(6000))]
    marks, reports = [], {}
    try:
        with mso.Capture(cfg, analog=[pwm_ch], seconds=3.5 + STEP_S * len(steps) + 2, rate=500_000,
                         pwm_window=0.1, save_as=request.config._tinyg_recorder.dir / "spindle-override-pwm") as cap:
            time.sleep(3.5)
            for i, (gcode, factor, _) in enumerate(steps):
                marks.append(time.time())
                if gcode:
                    tg.send_gcode(gcode, check=True, allow="spindle state for the override measurement")
                else:
                    assert tg.send_json({"sso": factor}, check=True).status == 0
                time.sleep(0.6)
                reports[i] = {k: tg.get(k) for k in ("sso", "ssc", "sps", "scp", "sse")}
                time.sleep(STEP_S - 0.6)
            marks.append(time.time())
    finally:
        tg.send_gcode("M5", check=True, allow="spindle off")
        tg.set("sso", 1)
    windows = cap.pwm(pwm_ch)
    t0 = cap.started_wall
    rows = []
    for i, (gcode, factor, want) in enumerate(steps):
        a, b = marks[i] - t0 + 0.3, marks[i + 1] - t0 - 0.1
        vals = sorted(w["duty"] for w in windows if a <= w["t"] <= b and w.get("duty") is not None)
        got = vals[len(vals) // 2] if vals else None
        rows.append({"step": gcode or f"sso {factor}", "want": round(want, 3), "got": round(got, 4) if got is not None else None,
                     "windows": len(vals), "report": reports.get(i)})
    summary = {"pwm_settings": PWM, "steps": rows}
    record("spindle_override_pwm", summary)
    print("SPINDLE-OVERRIDE", summary)
    for r in rows:
        assert r["got"] is not None and abs(r["got"] - r["want"]) <= 0.01, f"duty for {r['step']} is wrong: {summary}"
    assert reports[2]["scp"] in (1, True), f"the 12000 request did not report the clamp: {summary}"
    assert reports[1]["scp"] in (0, False), f"the 3000 request reported a clamp: {summary}"
