"""Diagnosis (E162): what does the firmware do with lines damaged by an RX ring overrun?

The dense goose program is streamed with no flow control and no pacing (rxm=0, ex=0), so the 250-byte
RX ring overruns. Inside a soft-limit fence and the runaway guard, each response's footer length is
matched against the sent lines: a mismatch is a spliced line, and status 0 on it means the board
executed a corrupted line. Record only.
"""
import pytest

from tinyg_hil import gcodeplay, sender

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449")]


def test_rx_overrun_damage(tg, keep, record, request):
    keep("ej", "rxm", "ex", "qv", "sl", "xtn", "xtm", "ytn", "ytm", "ztn", "ztm")
    lines = gcodeplay.goose()
    for token, value in {"ej": 1, "qv": 0, **sender.BLAST}.items():
        if tg.get(token) != value:
            tg.set(token, value, allow="overrun diagnosis")
    tg.send_gcode("G21 G90 G28.3 X0 Y0 Z0", check=True)
    for token, value in {"xtn": -2, "xtm": 22, "ytn": -2, "ytm": 22, "ztn": -2, "ztm": 3, "sl": 1}.items():
        tg.set(token, value, allow="soft-limit fence")
    tg.send_gcode("G92 X0 Y0 Z0", check=True)
    tg.guard.check_gcode("G1 X20 Y20", allow="goose inside a soft-limit fence")
    res, outcome = None, "completed"
    try:
        res = sender.stream(tg, lines, "blast", bounds=((-2, 22), (-2, 22)))
    except sender.StreamFailed as e:
        res, outcome = e.result, f"{type(e).__name__}: {str(e)[:200]}"
    finally:
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
        tg.send_gcode("G92.1")
    bad = sender.splices(res, lines)
    record("rx_overrun", {"outcome": outcome, "completion": res.completion, "lines": res.lines, "responses": len(res.footers),
                          "errors": res.errors[:10], "exceptions": res.exceptions[:5],
                          "spliced": len(bad), "spliced_executed": sum(1 for b in bad if b["status"] == 0),
                          "examples": bad[:8]})
