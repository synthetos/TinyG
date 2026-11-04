"""Soak: the serial port is closed and reopened the way UI users do, at every awkward moment, and the board must
carry on exactly. No ^X on reconnect (a UI does not reset); the host just reopens, drains whatever arrived while
nobody was listening, asks for status and continues.

RECONNECT_STEPS random scenarios (seed RECONNECT_SEED), each ending with an exact landing check (G0 to a random
target, mpo == target, queue empty):
  midjob      queue 6-12 moves, close the port mid-motion, wait 0.5-4 s, reopen: the queued moves must have kept
              running (position advanced), then the rest of a short job is streamed and must land
  hold        queue moves, '!', close during the hold, reopen, '~': the job resumes and lands
  alarm       cause an alarm (job kill ^D while moving), close, reopen, {clear}: the board answers and lands
  flap        open/close the port 8 times in 2 s during motion (DTR/RTS toggling): no reset, no lost motion
  backlog     leave hb=1 and sv=1 reporting with the port closed for 5 s, reopen: the first query's reply must be
              found after the backlog (possibly with one torn frame), then normal operation
  idlegap     close while idle for 10-30 s, reopen, {"stat":null} answers at once
Records per scenario: what was seen after reopen (backlog size, torn lines, first reply latency) and failures.
"""
import json
import os
import random
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(3600)]

STEPS = int(os.environ.get("RECONNECT_STEPS", "24"))
SEED = int(os.environ.get("RECONNECT_SEED", "0")) or random.randrange(1, 1 << 30)
TOL = 0.002
BOX = 30.0


def test_reconnect_soak(tg, keep, record):
    r = random.Random(SEED)
    keep("ej", "qv", "rxm", "ex", "sv", "si")
    for tok, val in (("ej", 1), ("qv", 1), ("sv", 1), ("si", 100), ("rxm", 1), ("ex", 1)):
        tg.set(tok, val, allow="line mode with reports")
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    tr = tg.transport
    failures, notes = [], []

    def mpo():
        return {a: tg.get(f"mpo{a}") for a in "xy"}

    def queue_moves(n, feed=600):
        for _ in range(n):
            tg.send_line(f"G90 G1 F{feed} X{r.uniform(-BOX, BOX):.3f} Y{r.uniform(-BOX, BOX):.3f}")

    def reopen(wait):
        """Close the port, wait, reopen; drain the backlog; return what the reopen saw."""
        t_close = time.monotonic()
        tr.close()
        time.sleep(wait)
        tr.open()
        t_open = time.monotonic()
        backlog, torn = 0, 0
        end = time.monotonic() + 0.4
        while time.monotonic() < end:                       # whatever the board sent while nobody listened
            ev = tr.next_event(0.05)
            if ev is None:
                continue
            backlog += 1
            if ev.kind == "line" or getattr(ev, "damaged", False):
                torn += 1
        t0 = time.monotonic()
        try:
            resp = tg.send_line('{"stat":null}', timeout=3.0)
            first = round(time.monotonic() - t0, 3)
            ok = resp.status == 0
        except Exception as e:  # noqa: BLE001
            first, ok = None, f"no reply: {type(e).__name__}"
        return {"closed_s": round(wait, 2), "backlog": backlog, "torn": torn, "first_reply_s": first,
                "answered": ok}

    def land(sc):
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
        tg.wait_idle(timeout=90)
        t = {a: round(r.uniform(-BOX, BOX), 3) for a in "xy"}
        resp = tg.send_line(f"G90 G0 X{t['x']} Y{t['y']}")
        tg.wait_idle(timeout=90)
        got, qr = mpo(), tg.get("qr")
        if resp.status != 0 or any(abs(got[a] - t[a]) > TOL for a in "xy"):
            failures.append({"scenario": sc, "why": "did not land exactly after the reconnect", "status": resp.status,
                             "target": t, "got": got})
        elif qr != 32:
            failures.append({"scenario": sc, "why": f"planner not empty after landing (qr {qr})"})

    hold = tg.dialect.stat("HOLD")
    for step in range(STEPS):
        sc = r.choice(["midjob", "hold", "alarm", "flap", "backlog", "idlegap"])
        info = {"scenario": sc}
        try:
            if sc == "midjob":
                queue_moves(r.randint(6, 12))
                time.sleep(r.uniform(0.2, 1.0))
                before = mpo()
                info["reopen"] = reopen(r.uniform(0.5, 4.0))
                after = mpo()
                info["moved_while_closed"] = round(max(abs(after[a] - before[a]) for a in "xy"), 3)
                if info["moved_while_closed"] < 0.5 and tg.stat() != tg.dialect.stat("ALARM"):
                    failures.append({"scenario": sc, "why": "motion stopped while the port was closed", **info})
                queue_moves(3)
            elif sc == "hold":
                queue_moves(r.randint(4, 8))
                time.sleep(r.uniform(0.3, 1.0))
                tr.write(b"!", drain_timeout=None)
                end = time.monotonic() + 8
                while time.monotonic() < end and tg.stat() != hold:
                    time.sleep(0.1)
                info["reopen"] = reopen(r.uniform(0.5, 3.0))
                info["stat_after_reopen"] = tg.stat()
                tr.write(b"~", drain_timeout=None)
            elif sc == "alarm":
                queue_moves(r.randint(3, 6))
                time.sleep(r.uniform(0.2, 0.8))
                tr.write(b"\x04", drain_timeout=None)        # job kill: alarms by design
                time.sleep(0.5)
                info["reopen"] = reopen(r.uniform(0.5, 2.0))
                info["stat_after_reopen"] = tg.stat()
                if info["stat_after_reopen"] != tg.dialect.stat("ALARM"):
                    failures.append({"scenario": sc, "why": "alarm not preserved across the reconnect", **info})
                tg.clear_alarm()
            elif sc == "flap":
                queue_moves(r.randint(6, 10), feed=300)
                time.sleep(0.3)
                before = mpo()
                for _ in range(8):
                    tr.close(); time.sleep(0.12); tr.open(); time.sleep(0.13)
                while tr.next_event(0.05) is not None:
                    pass
                tg.send_line('{"stat":null}', timeout=3.0)
                after = mpo()
                info["moved_during_flaps"] = round(max(abs(after[a] - before[a]) for a in "xy"), 3)
                if info["moved_during_flaps"] < 0.3:
                    failures.append({"scenario": sc, "why": "motion stopped during port flapping (reset?)", **info})
            elif sc == "backlog":
                tg.send_line('{"rt":{"hb":1}}')
                queue_moves(2, feed=300)
                info["reopen"] = reopen(5.0)
                tg.send_line('{"rt":{"hb":0}}')
                if info["reopen"]["answered"] is not True:
                    failures.append({"scenario": sc, "why": "no usable reply after a reporting backlog", **info})
            elif sc == "idlegap":
                tg.wait_idle(timeout=60)
                info["reopen"] = reopen(r.uniform(10, 30))
                if info["reopen"]["first_reply_s"] is None or info["reopen"]["first_reply_s"] > 1.0:
                    failures.append({"scenario": sc, "why": "slow or no reply after an idle gap", **info})
            notes.append(info)
            land(sc)
        except Exception as e:  # noqa: BLE001
            failures.append({"scenario": sc, "why": f"{type(e).__name__}: {str(e)[:160]}", **info})
            try:
                if not tr.is_open():
                    tr.open()
                tg.reset()
                tg.send_gcode("G21 G90 G54", allow="recover")
                tg.send_gcode("G28.3 X0 Y0 Z0", allow="recover")
            except Exception:  # noqa: BLE001
                break
        if len(failures) >= 6:
            break
    tg.send_line('{"rt":{"hb":0}}')
    record("reconnect", {"seed": SEED, "steps": step + 1, "failures": failures, "notes": notes[:40]})
    assert not failures, f"seed {SEED}: " + json.dumps(failures[:3])
