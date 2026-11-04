"""Overnight soak: soft limits against the wiki's rules (Homing-and-Limits-Setup-and-Troubleshooting, "Soft Limits").

Spec: with {sl:1}, each new block is checked against the travel limits (xtn..ztm) of homed axes: the endpoint
of traverses and feeds, and for arcs the endpoint and any part of the arc beyond the envelope. A violation is
refused with an alarm (status 220, "Soft limit exceeded"); {clear:n} clears it. Axes with min == max, or a limit
of -1 (disabled), are not tested. Limits are machine coordinates, so work offsets (G54..G59, G92) shift them.

The test fences X/Y to +/-20 and Z to +/-5 mm, declares the position with G28.3 (as homed), and runs
SOFTLIM_STEPS random cases: lines/rapids inside, outside and exactly on the limit; with G92 and G55 offsets; G53;
G91 relative; inch targets; G17/G18/G19 arcs whose endpoints are inside but whose body may leave the fence; and
blocks with {sl:0}. Each case checks the reply, the alarm state, and that a refused block did not move the machine
(an accepted one must arrive at its target). Seeded (SOFTLIM_SEED) for reproduction.
"""
import json
import math
import os
import random

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(3600)]

STEPS = int(os.environ.get("SOFTLIM_STEPS", "250"))
SEED = int(os.environ.get("SOFTLIM_SEED", "0")) or random.randrange(1, 1 << 30)
LIM = {"x": (-20.0, 20.0), "y": (-20.0, 20.0), "z": (-5.0, 5.0)}
TOL = 0.002
SOFT_LIMIT = 220


def _inside(p):
    return all(LIM[a][0] - 1e-6 <= p[a] <= LIM[a][1] + 1e-6 for a in "xyz")


def _arc_extremes(c1, c2, r, a0, a1, cw):
    """Points of an arc (plane axes 1, 2) at its start, end and every quadrant angle it passes."""
    pts = [(c1 + r * math.cos(a1), c2 + r * math.sin(a1))]
    span = (a0 - a1) % (2 * math.pi) if cw else (a1 - a0) % (2 * math.pi)
    for q in (0, math.pi / 2, math.pi, 3 * math.pi / 2):
        d = (a0 - q) % (2 * math.pi) if cw else (q - a0) % (2 * math.pi)
        if d <= span:
            pts.append((c1 + r * math.cos(q), c2 + r * math.sin(q)))
    return pts


def test_soft_limits_soak(tg, keep, record):
    r = random.Random(SEED)
    keep("sl", "xtn", "xtm", "ytn", "ytm", "ztn", "ztm", "g55x", "g55y", "g55z")
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position (as homed)")
    for a in "xyz":
        tg.set(f"{a}tn", LIM[a][0], allow="soft-limit fence")
        tg.set(f"{a}tm", LIM[a][1], allow="soft-limit fence")
    tg.set("sl", 1, allow="soft limits on")
    g55 = {"x": round(r.uniform(-8, 8), 3), "y": round(r.uniform(-8, 8), 3), "z": 0.0}
    tg.send_gcode(f"G10 L2 P2 X{g55['x']} Y{g55['y']} Z0", check=True, allow="G55 offset")
    machine = {"x": 0.0, "y": 0.0, "z": 0.0}
    failures, cases = [], 0

    def mpo():
        tg.wait_idle(timeout=60)
        return {a: tg.get(f"mpo{a}") for a in "xyz"}

    def run(line, expect_ok, target_machine, note):
        nonlocal machine
        before = mpo()
        resp = tg.send_line(line, timeout=5.0)
        alarm = tg.stat() == tg.dialect.stat("ALARM")    # a refused block alarms: clear before waiting for idle
        if alarm:
            tg.clear_alarm()
        after = mpo()
        moved = any(abs(after[a] - before[a]) > TOL for a in "xyz")
        ok = resp.status == 0 and not alarm
        bad = None
        if expect_ok and not ok:
            bad = "a block inside the limits was refused"
        elif not expect_ok and ok:
            bad = "a block outside the limits was accepted"
        elif not expect_ok and moved:
            bad = "a refused block moved the machine"
        elif not expect_ok and resp.status != SOFT_LIMIT:
            bad = f"refused with status {resp.status}, not 220"
        elif expect_ok and target_machine and any(abs(after[a] - target_machine[a]) > TOL for a in "xyz"):
            bad = "an accepted block did not arrive at its target"
        if bad:
            failures.append({"line": line, "why": bad, "note": note, "status": resp.status, "alarm": alarm,
                             "before": before, "after": after, "expected": target_machine})
        machine = after
        return ok

    try:
        for step in range(STEPS):
            cases += 1
            op = r.random()
            if op < 0.35:                                # line or rapid, G54, maybe on the edge
                t = {a: round(r.choice([r.uniform(*LIM[a]), r.uniform(LIM[a][0] - 6, LIM[a][1] + 6),
                                        LIM[a][r.randint(0, 1)]]), 3) for a in "xyz"}
                g = r.choice(["G0", "G1 F3000"])
                run(f"G54 G90 {g} X{t['x']} Y{t['y']} Z{t['z']}", _inside(t), t, "G54 line")
            elif op < 0.45:                              # G55: limits are machine coordinates
                w = {a: round(r.uniform(LIM[a][0] - 6, LIM[a][1] + 6), 3) for a in "xy"}
                m = {"x": w["x"] + g55["x"], "y": w["y"] + g55["y"], "z": machine["z"]}
                run(f"G55 G90 G1 F3000 X{w['x']} Y{w['y']}", _inside(m), m, "G55 work offset")
                tg.send_gcode("G54", check=True)
            elif op < 0.52:                              # G53 machine move
                t = {a: round(r.uniform(LIM[a][0] - 5, LIM[a][1] + 5), 3) for a in "xy"}
                m = {"x": t["x"], "y": t["y"], "z": machine["z"]}
                run(f"G53 G0 X{t['x']} Y{t['y']}", _inside(m), m, "G53")
            elif op < 0.62:                              # G91 relative
                d = {a: round(r.uniform(-12, 12), 3) for a in "xy"}
                m = {"x": machine["x"] + d["x"], "y": machine["y"] + d["y"], "z": machine["z"]}
                run(f"G91 G1 F3000 X{d['x']} Y{d['y']}", _inside(m), m, "G91 relative")
                tg.send_gcode("G90", check=True)
            elif op < 0.68:                              # inch target
                t = {a: round(r.uniform(-1.0, 1.0), 4) for a in "xy"}
                m = {"x": t["x"] * 25.4, "y": t["y"] * 25.4, "z": machine["z"]}
                run(f"G20 G90 G1 F100 X{t['x']} Y{t['y']}", _inside(m), m, "inch")
                tg.send_gcode("G21", check=True)
            elif op < 0.92:                              # arc in a random plane; endpoint inside, body maybe not
                plane, (p1, p2), i1, i2 = r.choice([("G17", ("x", "y"), "I", "J"), ("G18", ("z", "x"), "K", "I"),
                                                    ("G19", ("y", "z"), "J", "K")])
                rad = r.choice([2.0, 6.0, 12.0, 18.0])
                a0 = r.uniform(0, 2 * math.pi)
                c1, c2 = machine[p1] - rad * math.cos(a0), machine[p2] - rad * math.sin(a0)
                a1 = a0 + r.choice([-1, 1]) * r.uniform(0.3, 5.5)
                cw = r.random() < 0.5
                e1, e2 = c1 + rad * math.cos(a1), c2 + rad * math.sin(a1)
                end = dict(machine); end[p1], end[p2] = round(e1, 4), round(e2, 4)
                if not _inside(end):
                    cases -= 1
                    continue                             # endpoint cases are covered above; this tests the body
                body = _arc_extremes(c1, c2, rad, a0, math.atan2(end[p2] - c2, end[p1] - c1), cw)
                ok_body = all(LIM[p1][0] - 1e-3 <= u <= LIM[p1][1] + 1e-3 and LIM[p2][0] - 1e-3 <= v <= LIM[p2][1] + 1e-3
                              for u, v in body)
                g = "G2" if cw else "G3"
                line = (f"{plane} G90 {g} F2000 {p1.upper()}{end[p1]} {p2.upper()}{end[p2]} "
                        f"{i1}{round(c1 - machine[p1], 4)} {i2}{round(c2 - machine[p2], 4)}")
                run(line, ok_body, end, f"arc {plane} r{rad} body {'inside' if ok_body else 'crosses the fence'}")
                tg.send_gcode("G17", check=True)
            else:                                        # soft limits off: anything goes
                tg.set("sl", 0, allow="soft limits off for one block")
                t = {"x": round(r.uniform(25, 30), 3) * r.choice([-1, 1]), "y": machine["y"], "z": machine["z"]}
                run(f"G53 G0 X{t['x']}", True, t, "sl=0 outside the fence")
                tg.send_gcode("G53 G0 X0 Y0 Z0", check=True, allow="back inside")
                tg.wait_idle(timeout=60)                 # a persisted SET is refused while moving (106)
                machine = {"x": 0.0, "y": 0.0, "z": 0.0}
                tg.set("sl", 1, allow="soft limits on")
            if len(failures) >= 8:
                break
    finally:
        try:
            if tg.stat() == tg.dialect.stat("ALARM"):
                tg.clear_alarm()
            tg.wait_idle(timeout=60)                     # cleanup SETs are refused while moving (106)
        except Exception:  # noqa: BLE001
            pass
        tg.set("sl", 0, allow="soft limits off")
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
        tg.send_gcode("G21 G90 G17 G54 G92.1", allow="cleanup")
        tg.send_gcode("G53 G0 X0 Y0 Z0", allow="return")
        tg.wait_idle(timeout=60)
        record("soft_limits", {"seed": SEED, "cases": cases, "failures": failures})
    assert not failures, f"seed {SEED}: " + json.dumps(failures[:3])
