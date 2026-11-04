"""Overnight soak: interrupting motion the ways hosts and operators do, then checking the board still knows where it is.

INT_STEPS random scenarios (seed INT_SEED), each followed by an absolute G0 to a random target that must land
exactly (mpo == target) with the queue empty:
  flush     queue several moves, '!' at a random moment, wait for the hold, '%' (queue flush)
  kill      queue several moves, ^D (job kill) at a random moment
  reset     queue several moves, ^X mid-motion; afterwards no stale line may run (position stays put for 1.5 s)
  dwell     G4 P2, '!' during the dwell, '~' after a while; the dwell and the next move complete
  last      one long move, '!' near its end, '~'; it must arrive exactly
  jog       run's jog pattern: G91 G1 pieces of at most ~150 ms, at most two queued, released with '!', '%', G90
  settings  a setting written while moving (its current value): record the reply; the value must not change
"""
import json
import os
import random
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.dialect("d449"), pytest.mark.timeout(3600)]

STEPS = int(os.environ.get("INT_STEPS", "40"))
SEED = int(os.environ.get("INT_SEED", "0")) or random.randrange(1, 1 << 30)
TOL = 0.002
BOX = 40.0


def test_interrupt_soak(tg, record):
    r = random.Random(SEED)
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    failures, notes = [], []

    def mpo():
        return {a: tg.get(f"mpo{a}") for a in "xy"}

    def wait_stat(want, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if tg.stat() in want:
                return True
            time.sleep(0.05)
        return False

    def queue_moves(n):
        for _ in range(n):
            tg.send_line(f"G1 F{r.choice([600, 1500, 3000])} X{r.uniform(-BOX, BOX):.3f} Y{r.uniform(-BOX, BOX):.3f}")

    def land(scenario):
        if tg.stat() == tg.dialect.stat("ALARM"):          # a job kill alarms by design (E096): clear before idling
            tg.clear_alarm()
            notes.append({"scenario": scenario, "alarm": True})
        tg.wait_idle(timeout=60)
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
            notes.append({"scenario": scenario, "alarm": True})
        t = {a: round(r.uniform(-BOX, BOX), 3) for a in "xy"}
        resp = tg.send_line(f"G90 G0 X{t['x']} Y{t['y']}")
        tg.wait_idle(timeout=60)
        got, qr = mpo(), tg.get("qr")
        if resp.status != 0 or any(abs(got[a] - t[a]) > TOL for a in "xy"):
            failures.append({"scenario": scenario, "why": "the move after the interruption did not land exactly",
                             "status": resp.status, "target": t, "got": got})
        elif qr != 32:
            failures.append({"scenario": scenario, "why": f"planner not empty after landing (qr {qr})"})

    hold = tg.dialect.stat("HOLD")
    for step in range(STEPS):
        sc = r.choice(["flush", "kill", "reset", "dwell", "last", "jog", "settings"])
        try:
            if sc == "flush":
                queue_moves(r.randint(3, 10))
                time.sleep(r.uniform(0.1, 2.0))
                tg.send_raw(b"!")
                if not wait_stat({hold}, 8):
                    failures.append({"scenario": sc, "why": "feedhold never reached HOLD"})
                tg.send_raw(b"%")
            elif sc == "kill":
                queue_moves(r.randint(3, 10))
                time.sleep(r.uniform(0.1, 2.0))
                tg.send_raw(b"\x04")
            elif sc == "reset":
                queue_moves(r.randint(3, 8))
                time.sleep(r.uniform(0.2, 1.5))
                tg.reset()
                before = mpo()
                time.sleep(1.5)
                after = mpo()
                if any(abs(after[a] - before[a]) > TOL for a in "xy"):
                    failures.append({"scenario": sc, "why": "motion after a reset (a stale line ran?)",
                                     "before": before, "after": after})
                tg.send_gcode("G21 G90 G17 G94 G54", check=True)
                tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position after the reset")
            elif sc == "dwell":
                tg.send_line("G4 P2")
                time.sleep(r.uniform(0.3, 1.5))
                tg.send_raw(b"!")
                time.sleep(r.uniform(0.3, 1.5))
                tg.send_raw(b"~")
            elif sc == "last":
                t = {a: round(r.uniform(-BOX, BOX), 3) for a in "xy"}
                tg.send_line(f"G1 F1200 X{t['x']} Y{t['y']}")
                d = max(abs(t[a] - mpo()[a]) for a in "xy")
                time.sleep(max(0.1, d / 20.0 * r.uniform(0.6, 0.95)))
                tg.send_raw(b"!")
                time.sleep(r.uniform(0.3, 1.0))
                tg.send_raw(b"~")
                tg.wait_idle(timeout=60)
                got = mpo()
                if any(abs(got[a] - t[a]) > TOL for a in "xy"):
                    failures.append({"scenario": sc, "why": "a held and resumed move did not arrive", "target": t, "got": got})
            elif sc == "jog":
                for _ in range(r.randint(5, 40)):
                    tg.send_line(f"G91 G1 F1000 X{r.choice([-1, 1]) * 2.0:.3f}")
                    while tg.get("qr") < 30:                 # at most two queued
                        time.sleep(0.02)
                tg.send_raw(b"!")
                wait_stat({hold, tg.dialect.stat("PROGRAM_STOP"), tg.dialect.stat("PROGRAM_END")}, 5)
                tg.send_raw(b"%")
                tg.wait_idle(timeout=20)
                tg.send_line("G90")
            elif sc == "settings":
                tg.send_line(f"G1 F600 X{r.uniform(-BOX, BOX):.3f}")
                time.sleep(0.3)
                tok = r.choice(["xjm", "xvm", "si", "ja"])
                val = tg.get(tok)
                resp = tg.send_line(json.dumps({tok: val}))
                after = tg.get(tok)
                notes.append({"scenario": sc, "token": tok, "status": resp.status})
                if after != val:
                    failures.append({"scenario": sc, "why": "a setting write while moving changed the value",
                                     "token": tok, "was": val, "now": after, "status": resp.status})
            land(sc)
        except Exception as e:                           # a lost board or a harness error mid-scenario is a finding too
            failures.append({"scenario": sc, "why": f"{type(e).__name__}: {str(e)[:160]}"})
            try:
                tg.reset()
                tg.send_gcode("G21 G90 G54", allow="recover")
                tg.send_gcode("G28.3 X0 Y0 Z0", allow="recover")
            except Exception:
                break
        if len(failures) >= 6:
            break
    record("interrupt", {"seed": SEED, "failures": failures, "notes": notes[:50]})
    assert not failures, f"seed {SEED}: " + json.dumps(failures[:3])
