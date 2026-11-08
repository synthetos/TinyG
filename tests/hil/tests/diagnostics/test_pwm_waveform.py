"""Diagnostic (MSO bench): the spindle PWM output measured, not read back from settings.

Checks the waveforms behind two fixes that were verified only logically: a 0 % phase drives a steady low with no
pulses (N32), and an accepted PWM frequency comes out at that frequency within 1 % (N31). Each case sets the
phase or frequency, waits for the output to settle, captures the MSO's PWM channel and records frequency and duty
per window. As documented, $p1frq takes effect at power up or reset only and the off phase at the next spindle
change, so each case resets the board and sends M5 before measuring. Settings are restored. Needs a bench whose bench-file entry has an "mso" with an analog channel
named in PWM_CHANNEL (default: the first analog channel).
"""
import os
import time

import pytest

from tinyg_hil import benches, mso

pytestmark = [pytest.mark.diagnostic, pytest.mark.persists, pytest.mark.timing, pytest.mark.resets_board, pytest.mark.dialect("d449"),
              pytest.mark.timeout(300)]

CASES = [  # (label, settings, expected frequency Hz or None for steady, expected duty)
    ("off phase 0.10 at 100 Hz", {"p1frq": 100, "p1pof": 0.1}, 100.0, 0.10),
    ("off phase 0 (steady low)", {"p1frq": 100, "p1pof": 0.0}, None, 0.0),
    ("off phase 0.25 at 1000 Hz", {"p1frq": 1000, "p1pof": 0.25}, 1000.0, 0.25),
]


def test_pwm_waveform(tg, keep, record, request):
    bench = benches.bench(tg.transport.port)
    cfg = bench.get("mso")
    if not cfg or not cfg.get("analog"):
        pytest.skip("no MSO analog channel on this bench")
    name = os.environ.get("PWM_CHANNEL") or next(iter(cfg["analog"].values()))
    keep("p1frq", "p1pof")
    tg.send_gcode("M5", allow="spindle off: the PWM shows the off phase")
    results, bad = [], []
    for label, settings, want_hz, want_duty in CASES:
        for tok, val in settings.items():
            tg.set(tok, val, allow="PWM waveform check")
        tg.reset()                                  # $p1frq is applied at reset only
        tg.send_gcode("M5", allow="apply the off phase")
        time.sleep(0.5)
        stem = request.config._tinyg_recorder.dir / ("pwm-" + "".join(c if c.isalnum() else "-" for c in label))
        with mso.Capture(cfg, analog=[name], seconds=1.5, rate=1_000_000, save_as=stem) as cap:
            pass
        rows = cap.pwm(name)[1:]                   # the first window can straddle the capture start
        duty = sum(r["duty"] for r in rows) / len(rows)
        freqs = [r["freq_hz"] for r in rows]
        ok_duty = abs(duty - want_duty) < 0.005
        ok_freq = (max(freqs) == 0.0) if want_hz is None else all(abs(f - want_hz) / want_hz < 0.01 for f in freqs)
        r = {"case": label, "settings": settings, "duty": round(duty, 4), "freq_hz": freqs,
             "expected": {"freq_hz": want_hz, "duty": want_duty}, "ok": ok_duty and ok_freq}
        results.append(r)
        if not r["ok"]:
            bad.append(r)
    record("pwm_waveform", {"channel": name, "results": results})
    print("PWM", results)
    assert not bad, f"PWM output differs from its settings: {bad}"
