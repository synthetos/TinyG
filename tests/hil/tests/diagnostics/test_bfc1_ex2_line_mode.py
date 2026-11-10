"""B-FC-1: can a host that does no hardware flow control (run: Web Serial flowControl "none") drive a board set to
$ex=2 (RTS/CTS) in line mode?

With $ex=2 the board raises its RTS output when its receive buffer fills, and holds its own output while its CTS input
(the host's RTS line) is released. A host with flow control off ignores the first; this checks the second, and
whether line mode alone (rxm=1, one answer per line) keeps a stream intact:
  - host RTS asserted (pyserial's default at open): a 300-line stream in line mode runs through, every line answered,
    no errors, and the machine ends at the program's end point;
  - host RTS released: does a GET still get its answer? If the board holds output, a host that never drives RTS
    must leave it asserted (recorded: answered or held, and what arrives once RTS is asserted again).
Settings are put back ($ex, $rxm) and RTS is asserted again whatever happens.
"""
import math
import time

import pytest

from tinyg_hil import sender

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(300)]


def _program():
    """A 296-segment circle of radius 5 mm through the start point: gentle corners, so the stream runs at speed."""
    lines = ["G21 G90 G17 G94", "G92 X0 Y0 Z0", "F1200"]
    n = 296
    for i in range(1, n + 1):
        a = 2 * math.pi * i / n
        lines.append(f"G1 X{5 * math.cos(a) - 5:.4f} Y{5 * math.sin(a):.4f}")
    return lines


def test_ex2_line_mode_without_host_flow_control(tg, keep, record):
    ser = tg.transport.ser
    keep("ex", "rxm", "ej", "qv")
    out = {}
    try:
        ser.rts = True
        for tok, val in {"ej": 1, "qv": 1, "rxm": 1, "ex": 2}.items():
            if tg.get(tok) != val:
                tg.set(tok, val, allow="B-FC-1: RTS/CTS board, line-mode host")
        out["ex"], out["rxm"] = tg.get("ex"), tg.get("rxm")
        lines = _program()
        tg.guard.check_gcode("G1 X-10 Y5", allow="B-FC-1 circle")
        res = sender.stream(tg, lines, "line", timeout=240)
        out["stream"] = {"lines": res.lines, "acked": res.acked, "errors": res.errors[:5],
                         "exceptions": res.exceptions[:5], "completion": res.completion,
                         "seconds": round(res.finished - res.started, 2)}
        out["end"] = {t: tg.get(t) for t in ("posx", "posy", "stat")}

        ser.rts = False                                      # host stops asserting RTS: the board's CTS released
        time.sleep(0.05)
        t0 = time.monotonic()
        try:
            resp = tg.send_json({"stat": None}, timeout=2.0)
            out["rts_released"] = {"answered": True, "after_s": round(time.monotonic() - t0, 3),
                                   "body": getattr(resp, "body", None)}
        except Exception as e:                               # noqa: BLE001
            out["rts_released"] = {"answered": False, "error": type(e).__name__}
            ser.rts = True
            late = tg.drain(quiet=0.3, limit=3.0)
            out["rts_released"]["after_reassert"] = [ev.raw for ev in late][:5]
    finally:
        ser.rts = True
        time.sleep(0.05)
        tg.drain(quiet=0.2, limit=2.0)
        if tg.stat() not in tg.dialect.idle_stats:          # an aborted stream: stop and flush before any restore
            tg.send_raw(b"!")
            time.sleep(1.0)
            tg.send_raw(b"%")
        tg.wait_idle(timeout=60)                            # settings are refused while moves are queued (117)
        tg.send_gcode("G92.1")
    record("bfc1", out)
    print("BFC1", out)
    s = out["stream"]
    assert s["acked"] == s["lines"] and not s["errors"] and not s["exceptions"], out
    assert s["completion"] == "idle", out
    assert abs(out["end"]["posx"]) < 0.001 and abs(out["end"]["posy"]) < 0.001, out
