"""Diagnostics after the 450.03 tag: old GitHub reports and N83 (#298). Any bench, no extra instruments.

  test_issue_240_arcs_after_g1   the lines around the reported "Arc specification error" on a G1 (Aspire via cncjs)
  test_issue_163_inch_arcs       inch-mode arcs and a G1 after them, with the same moves in mm as the control
  test_n83_g10_then_lines        lines sent right after a G10 at rest, and the same lines after a settle pause: which
                                 loses input tells whether the EEPROM write behind the G10 is what collides. The
                                 offset is restored after each case.
"""
import time

import pytest

from tinyg_hil import sender

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.resets_board,
              pytest.mark.dialect("d449"), pytest.mark.timeout(6000)]   # includes waiting for the board lock

WHY = "follow-up diagnostic on an unloaded bench"
ISSUE_240 = ["G21 G90", "G92 X-93.980 Y0.712 Z0", "G1Z-1.500F914.4", "G2X-93.268Y-0.001I0.000J-0.713F400.0",
             "G2X-93.980Y-0.713I-0.713J0.000", "G2X-94.693Y-0.001I0.000J0.713", "G2X-93.980Y0.712I0.713J0.000",
             "G1Z0F914.4"]
ARCS = ["G90", "G92 X0 Y0", "G2 X{a} Y0 I{b} J0 F{f}", "G1 Y{b} F{f}", "G2 X0 Y{b} I{mb} J0 F{f}"]


def _stream(tg, keep, lines, mode="line"):
    keep("ej", "rxm", "ex", "qv", "sv")
    for token, value in {"ej": 1, "qv": 1, "sv": 1, **sender.MODES[mode]}.items():
        if tg.get(token) != value:
            tg.set(token, value, allow="stream settings")
    for ln in lines:
        tg.guard.check_gcode(ln, allow=WHY)
    res = sender.stream(tg, lines, mode, timeout=120)
    return [(s, t) for _, s, t in res.errors]


def test_issue_240_arcs_after_g1(tg, keep, record):
    try:
        errs = _stream(tg, keep, ISSUE_240)
    finally:
        tg.send_gcode("G92.1", allow="clear the origin offset")
    record("issue_240", {"errors": errs})
    print("I240", errs)
    assert not errs, f"lines refused: {errs}"


@pytest.mark.parametrize("units,a,b,f", [("G20", 0.1001, 0.05, 20), ("G21", 2.5425, 1.27, 508)], ids=["inch", "mm"])
def test_issue_163_inch_arcs(tg, keep, record, units, a, b, f):
    lines = [units] + [ln.format(a=a, b=b, mb=-b, f=f) for ln in ARCS]
    try:
        errs = _stream(tg, keep, lines)
    finally:
        tg.send_gcode("G92.1 G21", allow="clear the origin offset, back to mm")
    record("issue_163", {"units": units, "errors": errs})
    print("I163", units, errs)
    assert not errs, f"lines refused: {errs}"


@pytest.mark.parametrize("pause", [0.0, 0.25], ids=["at-once", "after-pause"])
def test_n83_g10_then_lines(tg, keep, record, pause):
    keep("g54z", "rxm", "ex")
    for token, value in sender.MODES["line"].items():
        if tg.get(token) != value:
            tg.set(token, value, allow="line mode")
    tg.send_gcode("G21 G90", check=True)
    z = tg.get("g54z")
    follow = [f"N{n} M5" for n in range(30, 110, 10)]                 # no motion, nothing to wait for
    mark = len(tg.transport.history)
    tg.guard.check_gcode("G10 L2 P1 Z-1.25", allow=WHY)
    if pause:
        tg.send_gcode(f"G10 L2 P1 Z{z - 1.25:.3f}", check=True, allow=WHY)
        time.sleep(pause)
        tg.transport.write("".join(ln + "\n" for ln in follow).encode())
    else:
        tg.transport.write((f"G10 L2 P1 Z{z - 1.25:.3f}\n" + "".join(ln + "\n" for ln in follow)).encode())
    time.sleep(1.5)
    tg.drain()
    h = tg.transport.history[mark:]
    exc = [e.raw for e in h if e.kind == "exception"]
    stat = tg.stat()
    r = {"pause": pause, "stat": stat, "exceptions": exc[:3], "replies": sum(1 for e in h if e.kind == "response")}
    record("n83", r)
    print("N83", r)
    if stat == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    time.sleep(0.3)
    tg.send_gcode(f"G10 L2 P1 Z{z:.3f}", allow="restore the offset")
    time.sleep(0.3)
    assert not exc, f"input lost: {r}"
