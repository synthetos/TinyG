"""N80: a long slow move must end on its target without a PANIC.

plan_exec.c _exec_aline_segment() built each segment's target by adding unit * segment_length onto the last one, in
32-bit floats, and snapped the section's last segment onto its waypoint. Over the ~21,900 segments of a 1,460 mm
move at 800 mm/min the board's running sum fell about 0.3 mm behind, and the snap asked for about 400 steps in one
5 ms segment of 250 DDA ticks: st_prep_line() panicked "step rate exceeds DDA" (st 20) and the board needed a reset. Found by the
bench 3 corpus pass (2026-11-06) and reproduced there on every run, with G0 Y1460 alone. Wiki: a G0/G1 ends at its
programmed point (Gcode-Support); nothing limits move length.

The Y motor gets 1280 steps/mm (1.8 deg, 8 microsteps, 1.25 mm/rev) and Y a 800 mm/min velocity cap, as on bench 3;
the move runs from Y0 and must end in a ready state at Y1460 exactly.
"""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.resets_board, pytest.mark.dialect("d449"),
              pytest.mark.timeout(400), pytest.mark.bug("N80", on=[])]


def test_n80_long_slow_move_ends_on_target(tg, keep, record):
    ymotors = [m for m in (1, 2, 3, 4) if tg.get(f"{m}ma") == 1]
    if not ymotors:
        pytest.skip("no motor mapped to Y")
    toks = ["yvm", "yfr", "yjm", "sl"] + [f"{m}{t}" for m in ymotors for t in ("sa", "mi", "tr")]
    keep(*toks)
    for m in ymotors:
        for tok, val in (("sa", 1.8), ("mi", 8), ("tr", 1.25)):
            tg.set(f"{m}{tok}", val, allow="1280 steps/mm on Y")
    for tok, val in (("yvm", 800), ("yfr", 800), ("yjm", 20), ("sl", 0)):
        tg.set(tok, val, allow="bench 3's Y limits, soft limits off for the long move")
    tg.reset()
    tg.send_gcode("G21 G90 G17 G94 G54", check=True)
    tg.send_gcode("G28.3 Y0", check=True, allow="start at Y0")
    mark = len(tg.transport.history)
    t0 = time.monotonic()
    tg.send_gcode("G0 Y1460", check=True, allow="N80 long slow move")
    dead = set(tg.dialect.stats("SHUTDOWN", "PANIC"))
    stat = None
    while time.monotonic() - t0 < 300:
        stat = tg.stat()
        if stat in dead or (stat in tg.dialect.idle_stats and tg.get("qr") == 32):
            break
        time.sleep(0.25)
    exc = [e.raw for e in tg.transport.history[mark:] if e.kind == "exception"]
    posy = tg.get("posy")
    r = {"stat": stat, "seconds": round(time.monotonic() - t0, 1), "posy": posy, "exceptions": exc}
    record("n80", r)
    if stat in dead:
        tg.reset()
    else:
        tg.send_gcode("G28.3 Y0", allow="back to zero without moving")
    assert stat not in dead, f"the 1460 mm move ended in stat {stat}: {exc}"
    assert posy == pytest.approx(1460, abs=0.001), f"the move ended at Y{posy}: {r}"
