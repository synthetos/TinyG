"""Diagnostic (bench 1, ICE): does a whole file's pulse count match the runtime position it reports?

Bench 3's MSO windows of round-2 corpus files compare net pulses with reported positions, which have 0.001 mm
resolution (1.3 steps at 1280 steps/mm). Skull2.tap (inch, G20 then G71, 320 arcs) came out 2 steps short in a window,
identically on a rerun: at the edge of what the report resolution can hide. Here the ICE reads the exact runtime
position (mr.position, float) and the firmware's own pulse count (en.en[m].encoder_steps + steps_run) before and
after streaming the whole file with every linear motor at 1280 steps/mm (as bench 3's Y and Z).

ACCT_FILE (required), ACCT_MODE (line). Pass: every motor's pulse count change is within 1 step of its axis's position
change times steps per unit.
"""
import os

import pytest

from tinyg_hil import sender

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.ice, pytest.mark.persists,
              pytest.mark.dialect("d449"), pytest.mark.timeout(6000)]   # includes waiting for the board lock

FILE = os.environ.get("ACCT_FILE", "")
MODE = os.environ.get("ACCT_MODE", "line")


def _snap(ice, motors):
    exprs = [f"en.en[{m - 1}].{f}" for m in motors for f in ("encoder_steps", "steps_run")]
    exprs += [f"mr.position[{a}]" for a in range(4)]
    v = ice.peek_many(exprs)
    pulses = {m: v[f"en.en[{m - 1}].encoder_steps"] + v[f"en.en[{m - 1}].steps_run"] for m in motors}
    return pulses, [v[f"mr.position[{a}]"] for a in range(4)]


def test_file_pulses_match_runtime_position(tg, keep, record, ice):
    if not FILE:
        pytest.skip("ACCT_FILE not set")
    lines = [ln.rstrip("\r") for ln in open(FILE, "rb").read().decode("latin-1").split("\n") if ln.strip()]
    for ln in lines:
        tg.guard.check_gcode(ln, allow="file accounting on an unloaded bench")
    motors = {m: tg.get(f"{m}ma") for m in (1, 2, 3, 4)}
    linear = [m for m, ax in motors.items() if ax in (0, 1, 2)]
    keep("ej", "rxm", "ex", "qv", *[f"{m}{t}" for m in linear for t in ("sa", "mi", "tr")])
    for token, value in {"ej": 1, "qv": 1, **sender.MODES[MODE]}.items():
        if tg.get(token) != value:
            tg.set(token, value, allow="stream settings")
    for m in linear:
        for tok, val in (("sa", 1.8), ("mi", 8), ("tr", 1.25)):
            tg.set(f"{m}{tok}", val, allow="1280 steps/mm, as bench 3")
    spu = {m: ice.peek(f"st_cfg.mot[{m - 1}].steps_per_unit") for m in linear}
    tg.send_gcode("G21 G90 G17 G94 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="start at zero")
    p0, x0 = _snap(ice, linear)
    res = sender.stream(tg, lines, MODE, timeout=3000)
    tg.wait_idle(timeout=120)
    p1, x1 = _snap(ice, linear)
    tg.send_gcode("M5 M9 G92.1 G90 G21 G94", allow="cleanup")
    rows = {}
    bad = []
    for m in linear:
        ax = motors[m]
        want = (x1[ax] - x0[ax]) * spu[m]
        got = p1[m] - p0[m]
        rows[m] = {"axis": "xyz"[ax], "pulses": got, "position_steps": round(want, 3), "diff": round(got - want, 3)}
        if abs(got - want) > 1.0:
            bad.append(m)
    r = {"file": FILE, "mode": MODE, "lines": len(lines), "errors": len(getattr(res, "errors", [])), "motors": rows}
    record("accounting", r)
    print("ACCT", r)
    assert not bad, f"pulses and runtime position disagree by more than a step: {rows}"
