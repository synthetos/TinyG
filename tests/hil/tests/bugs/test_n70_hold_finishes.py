"""N70: a feedhold must finish its stop promptly; a short block entered slowly could crawl in HOLD for hours.

Seen in the 450.02 soak (bench 3) and reproduced on bench 1: after '!' the board reports stat 6, but the hold state
stays FEEDHOLD_DECEL_TO_ZERO and the '~' waits for FEEDHOLD_HOLD, so the board looks frozen until reset. The ICE
snapshot showed the cause: the running block (99% A, length 0.101) was planned tail-only over its whole length from
an entry of 0.145 mm/min (once 0.00116), because MIN_BODY_LENGTH scales with cruise_vmax (plan_zoid.c tail-only case),
and mp_exec_aline() keeps a planned tail to zero as "already decelerating" (plan_exec.c): the stop would take 83 s
(once 2.9 h) at an average of half that speed. Needs an axis without a motor and fractional steps: A in degrees with
no motor mapped (aam=1, ara=10, 4ma=0), X/Y 0.01, Z 0.001, A 0.1 per line (corpus/n70). Feedhold wiki: a feedhold
decelerates to a stop and '~' resumes.

The stream runs in line mode with feed accounting on while '!' and, 0.4-1.5 s later, '~' are sent; after each '~' a
status report other than stat 6 must arrive within 3 s. With --ice a stuck hold is snapshotted.
"""

import os
from pathlib import Path
import random
import threading
import time

import pytest

from tinyg_hil import sender

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"), pytest.mark.timeout(1800),
              pytest.mark.bug("N70", on=[])]

FILE = str(Path(__file__).resolve().parents[2] / "corpus" / "n70" / "rotary-plus-linear-tiny-steps.nc")
REPEATS = int(os.environ.get("N70_REPEATS", "3"))
SEED = int(os.environ.get("N70_SEED", "9000"))
SHAPE = {"aam": 1, "ara": 10.0, "4ma": 0}                 # A in degrees, no motor on A (bench 3)
SNAP = ["cm.hold_state", "cm.end_hold_requested", "cm.motion_state", "cm.cycle_state", "cm.machine_state",
        "cm.queue_flush_state", "cm.program_paused", "mr.block_state", "st_pre.buffer_state", "mb.buffers_available",
        "us[0].rx_buf_count", "rx_in_line", "mr.section", "mr.section_state", "mr.segment_count", "mr.segments",
        "mr.segment_time", "mr.segment_velocity", "mr.tail_length", "mr.cruise_velocity", "mr.exit_velocity",
        "st_run.dda_ticks_downcount", "st_pre.block_type", "st_pre.dda_ticks", "mb.r", "mb.w", "mb.q"]


class _Holder(threading.Thread):
    def __init__(self, tg, ice, seed):
        super().__init__(daemon=True)
        self.tg, self.ice, self.r = tg, ice, random.Random(seed)
        self.stop, self.holds, self.stuck = threading.Event(), 0, []

    def _left_hold(self, since, wait):
        end = time.monotonic() + wait
        while time.monotonic() < end and not self.stop.is_set():
            for e in list(self.tg.transport.history)[-40:]:
                if e.t >= since and e.kind == "status" and isinstance(e.data, dict):
                    st = (e.data.get("sr") or {}).get("stat")
                    if st is not None and st != 6:
                        return True
            time.sleep(0.05)
        return False

    def run(self):
        while not self.stop.wait(self.r.uniform(1.0, 4.0)):
            self.tg.transport.write(b"!")
            if self.stop.wait(self.r.uniform(0.4, 1.5)):
                return
            t = time.monotonic()
            self.tg.transport.write(b"~")
            self.holds += 1
            if self._left_hold(t, 3.0) or self.stop.is_set():
                continue
            snap = {}
            try:
                if self.ice is not None:
                    for sym in SNAP:                         # one by one: a name the build lacks loses only itself
                        try:
                            snap[sym] = self.ice.peek(sym)
                        except Exception as exc:
                            snap[sym] = f"? {exc.__class__.__name__}"
                    snap["pc"] = [self.ice.where() for _ in range(3)]
                    for sym in ("mr.entry_velocity", "mr.head_length", "mr.body_length", "mr.forward_diff_5",
                                "mr.gm.move_time", "mr.unit[3]"):
                        try:
                            snap[sym] = self.ice.peek(sym)
                        except Exception as exc:
                            snap[sym] = f"? {exc.__class__.__name__}"
                    try:                                     # the running planner block: mb.r points into mb.bf[]
                        a0 = self.ice.symbols.resolve("mb.bf[0]")[0]
                        a1 = self.ice.symbols.resolve("mb.bf[1]")[0]
                        r = snap.get("mb.r")
                        sram = a0 if r is None or r >= 0x2000 else a0 - 0x800000   # pointer vs SRAM address base
                        idx = (int(r) - (a0 & 0xFFFF)) // (a1 - a0)
                        snap["run_buffer_index"] = idx
                        for f in ("block_state", "length", "head_length", "body_length", "tail_length",
                                  "entry_velocity", "cruise_velocity", "exit_velocity", "entry_vmax",
                                  "cruise_vmax", "exit_vmax", "jerk", "recip_jerk", "unit[0]", "unit[3]"):
                            snap["bf." + f] = self.ice.peek(f"mb.bf[{idx}].{f}")
                    except Exception as exc:
                        snap["bf_error"] = repr(exc)[:200]
                else:                                    # no queries: their replies would count as line replies
                    snap = {"note": "no ICE, not queried"}
            except Exception as exc:                     # a symbol the build lacks must not lose the event
                snap = {"error": repr(exc)}
            t2 = time.monotonic()
            self.tg.transport.write(b"~")
            second = self._left_hold(t2, 3.0)
            self.stuck.append({"hold_no": self.holds, "t": round(t, 3), "snapshot": snap, "second_tilde_resumed": second})
            if not second:
                self.stop.set()
                return


def test_hold_finishes_and_resumes(tg, keep, record, request):
    ice = request.getfixturevalue("ice") if request.config.getoption("--ice") else None
    lines = [l.rstrip("\r") for l in open(FILE, "rb").read().decode("latin-1").split("\n") if l.strip()]
    for line in lines:
        tg.guard.check_gcode(line, allow="N70 probe on an unloaded bench")
    keep("ej", "rxm", "ex", "qv", "sv", "si")
    for token, value in {"ej": 1, "qv": 1, "sv": 1, **sender.MODES["line"]}.items():
        if tg.get(token) != value:
            tg.set(token, value, allow="N70 settings")
    for tok, val in SHAPE.items():
        keep(tok)
        tg.set(tok, val, allow="N70 machine shape")
    original_sr = _sr_list(tg)                           # an earlier test may have left stat out of the list
    tg.send_json({"sr": {"stat": True, "vel": True, "posx": True, "posa": True}}, check=True)
    rows = []
    try:
        _passes(tg, lines, ice, rows)
    finally:
        tg.send_json({"sr": False})
        tg.send_json({"sr": {k: True for k in original_sr}})
    record("n70_probe", rows)
    stuck = [s for r in rows for s in r["stuck"]]
    assert not stuck, f"a '~' left the board in HOLD: {stuck}"


def _sr_list(tg):
    tg.transport.write(b'{"sr":null}\n')
    ev = tg.wait_for(lambda e: e.kind == "response" and isinstance(e.body, dict) and "sr" in e.body, 3.0)
    return list(ev.body["sr"])


def _passes(tg, lines, ice, rows):
    for k in range(REPEATS):
        tg.send_gcode("G21 G90 G28.3 X0 Y0 Z0 A0", check=True, allow="declare the start")
        tg.send_line('{"rt":{"fi":0}}')
        assert tg.send_line('{"rt":{"fi":1}}').status == 0
        holder = _Holder(tg, ice, SEED + k)
        holder.start()
        err = None
        try:
            sender.stream(tg, lines, "line", timeout=120)   # ~20 s when holds finish
        except Exception as exc:
            err = repr(exc)[:300]
        holder.stop.set(); holder.join(timeout=10)
        if holder.stuck:
            tg.reset()                                   # a stuck hold refuses settings (117): clear it first
        tg.transport.write(b"~")
        tg.send_line('{"rt":{"fi":0}}')
        rows.append({"pass": k, "holds": holder.holds, "stuck": holder.stuck, "stream_error": err})
        print("N70", rows[-1])
        if holder.stuck:
            break
        tg.wait_idle(timeout=60)
