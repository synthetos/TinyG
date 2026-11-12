"""Diagnostic (N80): a long four-axis G0 from the origin PANICked "step rate exceeds DDA" (st 20) near its end.

Seen in the bench 3 round-2 corpus pass (450.03, 2026-11-06): the file-window preamble for the G93 file
example_5axis.gcode rapided from X0 Y0 Z0 A0 to X0 Y1460 Z485 A37.623; 109 s in, about 10 mm before the end
(deceleration), the board reported st 20 and went to PANIC. No feedhold was involved. This replays the rapid alone,
and variants without A, without Z, and Y alone, to find which part of the move matters. A PANIC is the finding; the
board is reset after each case.
"""
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.resets_board, pytest.mark.dialect("d449"),
              pytest.mark.timeout(6000)]   # includes waiting for the board lock

CASES = {"full": "G0 X0 Y1460 Z485 A37.623", "no_a": "G0 X0 Y1460 Z485", "no_z": "G0 X0 Y1460 A37.623",
         "y_only": "G0 Y1460"}


@pytest.mark.parametrize("case", list(CASES))
def test_long_rapid_does_not_panic(tg, record, case):
    tg.reset()
    tg.send_gcode("G21 G90 G17 G94 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0 A0", check=True, allow="start at the origin")
    line = CASES[case]
    tg.guard.check_gcode(line, allow="N80 replay")
    mark = len(tg.transport.history)
    t0 = time.monotonic()
    tg.send_gcode(line, check=True, allow="N80 replay")
    dead = set(tg.dialect.stats("SHUTDOWN", "PANIC"))
    stat, last = None, None
    while time.monotonic() - t0 < 400:
        stat = tg.stat()
        if stat in dead or (stat in tg.dialect.idle_stats and tg.get("qr") == 32):
            break
        time.sleep(0.25)
    exc = [e.raw for e in tg.transport.history[mark:] if e.kind == "exception"]
    pos = {a: tg.get(f"pos{a}") for a in "xyza"}
    r = {"case": case, "line": line, "stat": stat, "seconds": round(time.monotonic() - t0, 1), "pos": pos,
         "exceptions": exc, "vm": {a: tg.get(f"{a}vm") for a in "yza"}}
    record("n80", r)
    print("N80", r)
    if stat in dead:
        tg.reset()
    else:
        tg.send_gcode("G28.3 X0 Y0 Z0 A0", allow="back to zero without moving")
    assert stat not in dead, f"{case}: stat {stat} {exc}"
