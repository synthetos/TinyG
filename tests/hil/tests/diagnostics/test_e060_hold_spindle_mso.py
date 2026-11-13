"""MSO diagnostic: spindle PWM stays on through a motion-only hold and resume."""
import statistics
import time

import pytest

from tinyg_hil import benches, mso

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.timing,
              pytest.mark.dialect("d449"), pytest.mark.timeout(240)]

WHY = "E060 hold/resume spindle timing on the MSO"


def _rises(edges):
    return [t for t, lv in edges if lv]


def _pwm_cycles(initial, edges):
    """[(t_rise, duty)] for every complete PWM period."""
    out, rise, fall = [], None, None
    for t, lv in edges:
        if lv:
            if rise is not None and fall is not None and t > rise:
                out.append((rise, (fall - rise) / (t - rise)))
            rise, fall = t, None
        elif rise is not None:
            fall = t
    return out


def _gap(step_times, min_gap=0.3):
    """(last step before the longest pause, first step after it)."""
    best = None
    for a, b in zip(step_times, step_times[1:]):
        if b - a >= min_gap and (best is None or b - a > best[1] - best[0]):
            best = (a, b)
    return best


def test_e060_hold_spindle_mso(tg, keep, record, request):
    bench = benches.bench(tg.transport.port) or {}
    cfg = bench.get("mso") or {}
    pwm_name = (cfg.get("analog") or {}).get("0")
    step_name = next((v for v in (cfg.get("digital") or {}).values() if v == "M2 step"), None)
    if not pwm_name or not step_name:
        pytest.skip("needs the MSO with the spindle PWM on analog 0 and M2 step (bench 3)")
    pof = tg.get("p1pof")
    y_axis = next(m for m in (1, 2, 3, 4) if tg.get(f"{m}ma") == 1)
    tg.send_gcode("G21 G91 G94", allow=WHY)
    tg.send_gcode("M3 S1500", allow=WHY)
    time.sleep(0.6)
    stem = request.config._tinyg_recorder.dir / "motion-only-hold-pwm"
    try:
        with mso.Capture(cfg, digital=[step_name], analog=[pwm_name], seconds=9.0, rate=250_000,
                         save_as=stem) as cap:
            time.sleep(0.15)                        # the capture is recording ~0.05 s after enter
            tg.send_gcode("G1 Y4 F120", allow=WHY)  # 2 s of motion at 2 mm/s
            time.sleep(0.8)
            tg.send_raw(b"!")
            time.sleep(1.5)
            tg.send_raw(b"~")
            tg.wait_idle(timeout=8)
    finally:
        tg.send_raw(b"~")
        try:
            tg.wait_idle(timeout=15)
        except Exception:  # noqa: BLE001
            pass
        tg.send_gcode("M5", allow=WHY)
        tg.send_gcode("G1 Y-4 F600", allow="return")
        tg.wait_idle(timeout=20)
        tg.send_gcode("G90", allow="absolute")
    steps = _rises(cap.edges().get(step_name, []))
    init, aedges = cap.analog_edges(pwm_name)
    cycles = _pwm_cycles(init, aedges)
    gap = _gap(steps)
    assert len(steps) > 100 and gap, f"no hold gap in {len(steps)} M{y_axis} steps"
    stop, go = gap
    before = [d for t, d in cycles if t < stop - 0.05]
    on_level = statistics.median(before)
    cut = (on_level + pof) / 2
    during = [(t, d) for t, d in cycles if stop + 0.25 < t < go - 0.05]
    off_during = [t for t, d in during if d < cut]
    result = {"pof": pof, "on_duty": round(on_level, 4), "last_step": round(stop, 4),
              "first_step_after": round(go, 4), "hold_gap_s": round(go - stop, 3),
              "cycles_in_hold": len(during), "off_cycles_in_hold": len(off_during)}
    record("e060_hold_spindle", result)
    print("E060", result)
    assert on_level > pof + 0.02, f"spindle PWM not on before the hold: {result}"
    assert during, f"no PWM cycles sampled during hold: {result}"
    assert not off_during, f"PWM dropped during motion-only hold: {result}"
