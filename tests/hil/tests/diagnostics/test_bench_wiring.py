"""Diagnostic: which homing switches on this bench are wired and working (bench bring-up, no instruments).

For each axis with a homing switch configured ($xsn or $xsx in a homing mode), run that axis's homing cycle alone
(G28.2 <axis>0) and record how it ended: homed (the switch was found and released), or a homing error after the
full search travel (no switch, or it never closed). The machine position is declared zero first so soft limits
cannot refuse the search. Records only; nothing is asserted beyond the board still answering.
"""
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.dialect("d449"), pytest.mark.timeout(900)]

AXES = "xyza"
HOMING_MODES = {1, 3}           # 1 homing only, 3 homing and limit


def _homing_axes(tg):
    out = {}
    for a in AXES:
        sn, sx = tg.get(f"{a}sn"), tg.get(f"{a}sx")
        if sn in HOMING_MODES or sx in HOMING_MODES:
            out[a] = {"sn": sn, "sx": sx, "sv": tg.get(f"{a}sv"), "lv": tg.get(f"{a}lv"),
                      "tn": tg.get(f"{a}tn"), "tm": tg.get(f"{a}tm")}
    return out


def test_bench_wiring(tg, record):
    info = {"switch_type": tg.get("st"), "motor_map": {m: tg.get(f"{m}ma") for m in "1234"}}
    axes = _homing_axes(tg)
    info["homing_axes"] = axes
    results = {}
    for a, cfg in axes.items():
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
        tg.send_gcode("G21 G90", check=True)
        tg.send_gcode("G28.3 X0 Y0 Z0 A0", check=True, allow="declare the position before a homing probe")
        mark = len(tg.transport.history)
        t0 = time.monotonic()
        line = f"G28.2 {a.upper()}0"
        tg.guard.check_gcode(line, allow="bench bring-up homing probe, one axis")
        status = tg.send_line(line, timeout=5.0).status
        end = time.monotonic() + 240
        while time.monotonic() < end:
            time.sleep(0.5)
            try:
                s = tg.stat()
            except Exception:  # noqa: BLE001
                continue
            if s in tg.dialect.stats("READY", "PROGRAM_STOP", "PROGRAM_END", "ALARM"):
                break
        exc = [e.raw[:140] for e in tg.transport.history[mark:] if e.kind == "exception"]
        homed = tg.get(f"hom{a}") if a != "a" else tg.get("homa")
        results[a] = {"status": status, "seconds": round(time.monotonic() - t0, 1), "stat": tg.stat(),
                      "homed": homed, "position": tg.get(f"mpo{a}"), "exceptions": exc[:3]}
        record(f"homing_{a}", results[a])
    info["results"] = results
    record("bench_wiring", info)
    if tg.stat() == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    assert tg.get("fb") is not None
