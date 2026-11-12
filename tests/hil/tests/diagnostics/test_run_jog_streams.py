"""Bench 3 (MSO): run's jog and typed-move wire streams, judged from the real step train.

run (run.synthetos.com) drives the board with one line in flight, a poll cycle of four GETs (stat, hold, vel, qr)
sent back to back, and feeds long motions as pieces while fewer than 4 are believed queued (queued = planner size
32 - qr of the last complete cycle + pieces written since). These tests replay that protocol on the wire, as run's
own capture shows it (syn-site, 2026-11-09; run commits ecef1cb / 95a9d65), on bench 3's Y axis (motors 2 and 3):
  test_step_jog          a) a 10 mm step is ONE move "G21 G91 G1 Y10 F1000": one smooth trapezoid, exact travel.
  test_hold_jog          b) a hold jog of 2.5 mm pieces at F1000 for ~2 s, released with ! then %: no braking between
                         pieces while held, the hold decelerates once, the flush leaves no motion, steps match the
                         reported position.
  test_typed_move        c) a typed G1 Y40 F600 split by run's sanitiser into 7 absolute pieces fed 4 deep: no
                         braking between pieces; then the same line paused and resumed mid-way (! ~), and stopped
                         mid-way (! %): resume completes at 40, stop leaves the axis where the steps say.
"No braking" is measured: the speed profile (20 ms bins of step rises) between first and last reaching 95% of its
cruise must never fall under 90% of that cruise.
"""
import time

import pytest

from tinyg_hil import benches, mso

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.dialect("d449"), pytest.mark.timeout(300)]

LOOKAHEAD = 4                       # run's pieces believed queued
POOL = 32                           # planner size on edge (qr when empty)
HOLD_DONE = 6                       # hold state: hold complete
NO_DIP = 0.90                       # speed between cruise points stays above this share of cruise


def _cfg(tg):
    cfg = (benches.bench(tg.transport.port) or {}).get("mso") or {}
    if not cfg.get("digital"):
        pytest.skip("needs the MSO (bench 3)")
    return cfg


def _spm(tg, m=2):
    return 360.0 * tg.get(f"{m}mi") / (tg.get(f"{m}sa") * tg.get(f"{m}tr"))


def _rises(edges):
    return [t for t, v in edges if v == 1]


def _speed_profile(rises, spm, bin_s=0.02):
    if len(rises) < 2:
        return []
    t0, n = rises[0], int((rises[-1] - rises[0]) / bin_s) + 1
    counts = [0] * n
    for t in rises:
        counts[min(n - 1, int((t - t0) / bin_s))] += 1
    return [(k * bin_s, c / spm / bin_s) for k, c in enumerate(counts)]


def _dip(prof):
    """(cruise mm/s, lowest share of cruise between the first and last bin at >= 95% of it, its time)."""
    v = max(sp for _, sp in prof)
    idx = [i for i, (_, sp) in enumerate(prof) if sp >= 0.95 * v]
    i0, i1 = idx[0], idx[-1]
    low_i = min(range(i0, i1 + 1), key=lambda i: prof[i][1])
    return v, prof[low_i][1] / v, prof[low_i][0]


def _cycle(tg):
    """run's poll cycle: four GETs, each written when the previous answer arrives. None unless all are numbers."""
    c = {}
    for tok in ("stat", "hold", "vel", "qr"):
        try:
            c[tok] = tg.get(tok)
        except Exception:
            return None
        if not isinstance(c[tok], (int, float)):
            return None
    return c


def _at_rest(c):
    return c and c["stat"] in (1, 3, 4) and c["hold"] == 0 and c["vel"] == 0 and c["qr"] == POOL


def _until(tg, done, count=2, timeout=60):
    """Poll back to back until `done` holds for `count` consecutive complete cycles."""
    end, run_, last = time.monotonic() + timeout, 0, None
    while time.monotonic() < end:
        c = _cycle(tg)
        if c is None:
            continue
        last = c
        run_ = run_ + 1 if done(c) else 0
        if run_ >= count:
            return c
    raise AssertionError(f"poll condition not reached in {timeout}s; last cycle {last}")


def _feed(tg, pieces, until=None):
    """Feed pieces like run: one line in flight, a piece whenever fewer than LOOKAHEAD are believed queued, checked
    after each piece's answer and each complete poll cycle. Stops at the end of the list or when until() is true."""
    pieces, queued, sent = list(pieces), 0, []
    while pieces and not (until and until()):
        if queued < LOOKAHEAD:
            line = pieces.pop(0)
            tg.send_gcode(line, check=True)          # answered before the next line (one in flight)
            sent.append(line)
            queued += 1
            continue
        c = _cycle(tg)
        if c is not None:
            queued = POOL - c["qr"]
    return sent


def _prep(tg):
    for tok in ("unit", "dist"):                     # run reads the modes it will put back
        tg.get(tok)
    tg.send_gcode("G21", check=True)


def _capture(tg, request, name, seconds):
    return mso.Capture(_cfg(tg), digital=["M2 step", "M3 step"], seconds=seconds, rate=1_000_000,
                       save_as=request.config._tinyg_recorder.dir / name)


def _home_y(tg):
    tg.send_gcode("G21 G90", allow="units")
    tg.send_gcode("G28.3 Y0", allow="declare Y")


def _back(tg):
    tg.send_gcode("G90 G0 Y0", allow="return Y")
    tg.wait_idle(timeout=60)


def test_step_jog(tg, record, request):
    spm = _spm(tg)
    _home_y(tg)
    with _capture(tg, request, "run-step-jog", 8) as cap:
        time.sleep(3.5)
        _prep(tg)
        tg.send_gcode("G21 G91 G1 Y10 F1000", check=True)
        _until(tg, _at_rest)
        tg.send_gcode("G90", check=True)
    posy = tg.get("posy")
    _back(tg)
    e = cap.edges()
    m2, m3 = _rises(e["M2 step"]), _rises(e["M3 step"])
    v, low, t_low = _dip(_speed_profile(m2, spm))
    s = {"steps_m2": len(m2), "steps_m3": len(m3), "expected": round(10 * spm), "posy": posy,
         "cruise_mm_min": round(v * 60, 1), "lowest_share": round(low, 3), "at_s": round(t_low, 3)}
    record("run_step_jog", s)
    print("STEP JOG", s)
    assert len(m2) == len(m3) and abs(len(m2) - 10 * spm) <= 1, s
    assert posy == pytest.approx(10.0, abs=0.001), s
    assert low >= NO_DIP, f"the 10 mm step braked mid-move: {s}"


def test_hold_jog(tg, record, request):
    spm = _spm(tg)
    _home_y(tg)
    with _capture(tg, request, "run-hold-jog", 12) as cap:
        time.sleep(3.5)
        _prep(tg)
        t_end = time.monotonic() + 2.0                # held about 2 s
        sent = _feed(tg, ["G21 G91 G1 Y2.5 F1000"] * 40, until=lambda: time.monotonic() > t_end)
        tg.send_raw(b"!")                             # release
        _until(tg, lambda c: c["hold"] == HOLD_DONE or _at_rest(c))
        c = _cycle(tg)
        if c and (c["hold"] or c["qr"] != POOL):
            tg.send_raw(b"%")
        _until(tg, lambda c: c["hold"] == 0 and c["qr"] == POOL)
        time.sleep(0.5)                               # anything still stepping after the flush shows in the capture
        tg.send_gcode("G90", check=True)
    posy = tg.get("posy")
    _back(tg)
    e = cap.edges()
    m2, m3 = _rises(e["M2 step"]), _rises(e["M3 step"])
    v, low, t_low = _dip(_speed_profile(m2, spm))
    s = {"pieces": len(sent), "steps_m2": len(m2), "steps_m3": len(m3), "travel_from_steps": round(len(m2) / spm, 4),
         "posy": posy, "cruise_mm_min": round(v * 60, 1), "lowest_share": round(low, 3), "at_s": round(t_low, 3)}
    record("run_hold_jog", s)
    print("HOLD JOG", s)
    assert len(m2) == len(m3), s
    assert posy == pytest.approx(len(m2) / spm, abs=1.5 / spm), f"reported position disagrees with the steps: {s}"
    assert posy < len(sent) * 2.5, f"the flush did not drop the queued pieces: {s}"
    assert low >= NO_DIP, f"the hold jog braked between pieces: {s}"


def _typed_pieces():
    tgt = [round(40 * k / 7, 4) for k in range(1, 8)]
    return [f"F600 G1 Y{tgt[0]:g}"] + [f"G1 Y{t:g}" for t in tgt[1:]]


@pytest.mark.parametrize("variant", ["through", "pause-resume", "stop"])
def test_typed_move(tg, record, request, variant):
    spm = _spm(tg)
    _home_y(tg)
    with _capture(tg, request, f"run-typed-{variant}", 16) as cap:
        time.sleep(3.5)
        _prep(tg)
        tg.send_gcode("G90", check=True)
        pieces = _typed_pieces()
        if variant == "through":
            _feed(tg, pieces)
            _until(tg, _at_rest)
        else:
            t_mid = time.monotonic() + 2.0            # about 20 mm in at the planned speed
            sent = _feed(tg, pieces, until=lambda: time.monotonic() > t_mid)
            while time.monotonic() < t_mid:
                _cycle(tg)
            tg.send_raw(b"!")
            _until(tg, lambda c: c["hold"] == HOLD_DONE or _at_rest(c))
            if variant == "pause-resume":
                time.sleep(0.5)
                tg.send_raw(b"~")
                _feed(tg, pieces[len(sent):])
                _until(tg, _at_rest)
            else:
                tg.send_raw(b"%")
                _until(tg, lambda c: c["hold"] == 0 and c["qr"] == POOL)
                time.sleep(0.5)
    posy = tg.get("posy")
    _back(tg)
    e = cap.edges()
    m2, m3 = _rises(e["M2 step"]), _rises(e["M3 step"])
    prof = _speed_profile(m2, spm)
    v, low, t_low = _dip(prof)
    s = {"variant": variant, "steps_m2": len(m2), "steps_m3": len(m3), "travel_from_steps": round(len(m2) / spm, 4),
         "posy": posy, "cruise_mm_min": round(v * 60, 1), "lowest_share": round(low, 3), "at_s": round(t_low, 3)}
    record("run_typed_move", s)
    print("TYPED", s)
    assert len(m2) == len(m3), s
    assert posy == pytest.approx(len(m2) / spm, abs=1.5 / spm), f"reported position disagrees with the steps: {s}"
    if variant == "stop":
        assert posy < 39.0, f"the stop did not end the typed move: {s}"
    else:
        assert posy == pytest.approx(40.0, abs=0.001), s
    if variant == "through":
        assert low >= NO_DIP, f"the typed move braked between pieces: {s}"
