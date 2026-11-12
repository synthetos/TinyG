"""Diagnostic (N79, bench 3 MSO): changing $Npo between moves knocks that motor out of step phase with its twin.

st_prep_line() (stepper.c) stores each motor's direction with the polarity folded in (DIRECTION_CW ^ polarity), and
the segment loader mirrors the substep accumulator whenever that value differs from the previous segment's, as it
must on a real reversal. After a $Npo change the next move in the same travel direction looks like a reversal, so the
accumulator is mirrored though the motor keeps going the same way: its fractional step phase moves and it can end a
move one step off. Seen in the bench 3 corpus loop (2026-11-06): after a job that ran with 3po=1 and restored 3po=0,
M3 trailed M2 (both on Y, same settings) by up to 3 ms instead of 2.6 us and ended one step ahead at rest.

From a reset (clean accumulators), Y moves by a distance with a fractional step count; between moves $3po flips to 1
and back. M2 and M3 must stay in step: same pulse count, and each M3 pulse within 20 us of its M2 pulse.
"""
import time

import pytest

from tinyg_hil import benches, mso

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.resets_board,
              pytest.mark.dialect("d449"), pytest.mark.timeout(300)]


@pytest.mark.parametrize("frac", [0.25, 0.4, 0.7])
def test_polarity_change_keeps_twin_motors_in_phase(tg, keep, record, request, frac):
    bench = benches.bench(tg.transport.port) or {}
    cfg = bench.get("mso") or {}
    if not cfg.get("digital"):
        pytest.skip("needs the MSO (bench 3)")
    keep("3po")
    tg.set("3po", 0, allow="start from normal polarity")
    tg.reset()                                             # clean substep accumulators
    if tg.get("2ma") != 1 or tg.get("3ma") != 1:
        pytest.skip("needs M2 and M3 both on Y")
    spm = 360.0 * tg.get("2mi") / (tg.get("2sa") * tg.get("2tr"))
    d = round((400 + frac) / spm, 6)                       # 400 + frac steps
    line = f"G1 Y{d:.6f} F300"
    tg.send_gcode("G21 G91", allow="relative")
    tg.guard.check_gcode(line, allow="N79 Y moves")
    with mso.Capture(cfg, digital=["M2 step", "M3 step"], seconds=14, rate=1_000_000,
                     save_as=request.config._tinyg_recorder.dir / f"n79-{frac}") as cap:
        time.sleep(0.3)
        for po in (None, 1, 0, None):
            if po is not None:
                tg.set("3po", po, allow="flip M3 polarity between moves")
            tg.send_gcode(line, allow="N79 Y move")
            tg.wait_idle(timeout=20)
    tg.send_gcode("G90", allow="absolute")
    e = cap.edges()
    a = [t for t, v in e["M2 step"] if v == 1]
    b = [t for t, v in e["M3 step"] if v == 1]
    n = min(len(a), len(b))
    lag = [round((b[i] - a[i]) * 1e6, 1) for i in range(n)]
    worst = max(lag, key=abs) if lag else None
    r = {"frac": frac, "m2": len(a), "m3": len(b), "worst_lag_us": worst, "lag_first": lag[:3], "lag_last": lag[-3:]}
    record("n79", r)
    print("N79", r)
    assert len(a) == len(b), f"M2 stepped {len(a)}, M3 {len(b)} after the $3po flips: {r}"
    assert abs(worst) < 20, f"M3 out of phase with M2 after the $3po flips: {r}"
