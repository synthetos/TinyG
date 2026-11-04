"""Overnight soak: coordinate systems, G92 and tool length offsets against a host model, many random steps.

Sets distinct G10 L2 offsets for G54..G59 (restored at the end), declares the machine position with G28.3
(as after homing), then runs COORDS_STEPS random steps: select a system, G0/G1 to a work target, G92 to a
value, G92.1, G43.1 Z tool offset, G49, G53 machine moves, inch blocks. After each move the reported work
position (posx..) and machine position (mpox..) must match the model:
    work = machine - coord_offset[system] - g92 - tool (Z only), in mm.
"""
import json
import os
import random
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(3600)]

STEPS = int(os.environ.get("COORDS_STEPS", "300"))
SEED = int(os.environ.get("COORDS_SEED", "0")) or random.randrange(1, 1 << 30)
TOL = 0.002
SYS = ["G54", "G55", "G56", "G57", "G58", "G59"]


def test_coords_soak(tg, keep, record):
    r = random.Random(SEED)
    offs = {}
    g10_refusals = []
    tokens = [f"g{c}{a}" for c in (54, 55, 56, 57, 58, 59) for a in "xyz"]
    keep(*tokens)
    for i, s in enumerate(SYS):
        offs[s] = {a: (0.0 if s == "G54" else round(r.uniform(-20, 20), 3)) for a in "xyz"}
        line = f"G10 L2 P{i + 1} X{offs[s]['x']} Y{offs[s]['y']} Z{offs[s]['z']}"
        for attempt in range(20):                        # finding N01: back-to-back G10 can answer 13 (buffer full)
            resp = tg.send_line(line, timeout=5.0)
            if resp.status == 0:
                break
            g10_refusals.append({"line": line, "status": resp.status, "attempt": attempt})
            tg.wait_idle(timeout=10)
            time.sleep(0.5)
        assert resp.status == 0, f"G10 never accepted: {line}"
    tg.send_gcode("G21 G90 G17 G94 G92.1 G49 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position (as homed)")
    machine = {"x": 0.0, "y": 0.0, "z": 0.0}
    sysname, g92, tool = "G54", {"x": 0.0, "y": 0.0, "z": 0.0}, 0.0
    failures, log = [], []

    def work_of(mach):
        return {a: mach[a] - offs[sysname][a] - g92[a] - (tool if a == "z" else 0.0) for a in "xyz"}

    def check(step):
        tg.wait_idle(timeout=60)
        w = {a: tg.get(f"pos{a}") for a in "xyz"}
        m = {a: tg.get(f"mpo{a}") for a in "xyz"}
        exp_w = work_of(machine)
        bad = {a: (w[a], exp_w[a], m[a], machine[a]) for a in "xyz"
               if abs(w[a] - exp_w[a]) > TOL or abs(m[a] - machine[a]) > TOL}
        if bad:
            failures.append({"step": step, "sys": sysname, "g92": dict(g92), "tool": tool, "bad": bad, "last": log[-6:]})

    try:
        for step in range(STEPS):
            op = r.random()
            if op < 0.15:
                sysname = r.choice(SYS)
                tg.send_gcode(sysname, check=True); log.append(sysname)
            elif op < 0.55:
                target = {a: round(r.uniform(-30, 30), 3) for a in "xy"}
                target["z"] = round(r.uniform(-5, 5), 3)
                cmd = f"{r.choice(['G0', 'G1 F2000'])} X{target['x']} Y{target['y']} Z{target['z']}"
                tg.send_gcode(cmd, check=True, allow="coords soak"); log.append(cmd)
                for a in "xyz":
                    machine[a] = target[a] + offs[sysname][a] + g92[a] + (tool if a == "z" else 0.0)
                check(step)
            elif op < 0.65:
                cur = work_of(machine)
                val = {a: round(r.uniform(-10, 10), 3) for a in "xyz"}
                tg.send_gcode(f"G92 X{val['x']} Y{val['y']} Z{val['z']}", check=True); log.append(f"G92 {val}")
                for a in "xyz":
                    g92[a] += cur[a] - val[a]
                check(step)
            elif op < 0.70:
                tg.send_gcode("G92.1", check=True); log.append("G92.1")
                g92 = {"x": 0.0, "y": 0.0, "z": 0.0}
                check(step)
            elif op < 0.78:
                tool = round(r.uniform(-15, 15), 3)
                tg.send_gcode(f"G43.1 Z{tool}", check=True, allow="tool offset"); log.append(f"G43.1 Z{tool}")
                check(step)
            elif op < 0.82:
                tg.send_gcode("G49", check=True); log.append("G49"); tool = 0.0
                check(step)
            elif op < 0.92:
                target = {a: round(r.uniform(-25, 25), 3) for a in "xy"}
                cmd = f"G53 G0 X{target['x']} Y{target['y']}"
                tg.send_gcode(cmd, check=True, allow="machine move"); log.append(cmd)
                machine["x"], machine["y"] = target["x"], target["y"]
                check(step)
            else:
                target = {a: round(r.uniform(-1, 1), 4) for a in "xy"}
                # G20 on its own line: F executes before G20 in a block (NIST 3.8, items 3 and 12), so
                # "G20 G1 F50" sets 50 mm/min and a 1.7-inch move takes a minute
                tg.send_gcode("G20", check=True); log.append("G20")
                cmd = f"G1 F50 X{target['x']} Y{target['y']}"
                tg.send_gcode(cmd, check=True, allow="inch move"); log.append(cmd)
                tg.send_gcode("G21", check=True)
                for a in "xy":
                    machine[a] = target[a] * 25.4 + offs[sysname][a] + g92[a]
                check(step)
            if len(failures) >= 5:
                break
    finally:
        tg.send_gcode("G21 G90 G92.1 G49 G54", allow="cleanup")
        tg.send_gcode("G53 G0 X0 Y0 Z0", allow="return")
        tg.wait_idle(timeout=60)
        record("coords", {"seed": SEED, "steps": STEPS, "offsets": offs, "failures": failures, "g10_refusals": g10_refusals})
    assert not failures, f"seed {SEED}: " + json.dumps(failures[:3])
