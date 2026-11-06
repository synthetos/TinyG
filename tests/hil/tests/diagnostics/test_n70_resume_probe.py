"""Diagnostic (N70): a feedhold reached HOLD and the '~' sent 450 ms later was ignored (450.02 soak, bench 3).

Streams a file in line mode with feed accounting on (as the soak did) while a holder thread sends '!' and, 0.4-1.5 s
later, '~'. After each '~' it waits up to 3 s for a status report that is not stat 6 (HOLD). If none comes, it records
one ICE snapshot of the hold state and the receive side, then sends a second '~' and records whether that resumes.
The snapshot tells the candidate causes apart:
- cm.end_hold_requested set and hold_state short of FEEDHOLD_HOLD (6): the '~' was received and waits for a hold that
  never finishes;
- end_hold_requested clear, hold_state HOLD: the '~' never reached cm_request_end_hold() (lost on the wire, or taken as
  line text because the receiver thought a line was in progress, rx_in_line);
- end_hold_requested clear after a "held move geometry" exception: the replan refused the resume.
N70_FILE (default corpus/n70/rotary-plus-linear-tiny-steps.nc), N70_REPEATS passes, N70_SEED. With --ice the
snapshot is one ICE halt; without, the holder writes {"hold":null}, {"mots":null} and {"cycs":null} and reads the
replies from the transport history (the stream is stalled by then, so they cannot be taken for line replies).
"""
import os
from pathlib import Path
import random
import threading
import time

import pytest

from tinyg_hil import sender

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.dialect("d449"),
              pytest.mark.timeout(3600)]

FILE = os.environ.get("N70_FILE", str(Path(__file__).resolve().parents[2] / "corpus" / "n70" / "rotary-plus-linear-tiny-steps.nc"))
REPEATS = int(os.environ.get("N70_REPEATS", "10"))
SEED = int(os.environ.get("N70_SEED", "7000"))
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
            if self._crawling():
                self.stuck.append({"crawl": True, "after_hold_no": self.holds, "snapshot": self._snapshot()})
                self.stop.set()
                return
            self.tg.transport.write(b"!")
            if self.stop.wait(self.r.uniform(0.4, 1.5)):
                return
            t = time.monotonic()
            self.tg.transport.write(b"~")
            self.holds += 1
            if self._left_hold(t, 3.0) or self.stop.is_set():
                continue
            snap = self._snapshot()
            t2 = time.monotonic()
            self.tg.transport.write(b"~")
            second = self._left_hold(t2, 3.0)
            self.stuck.append({"hold_no": self.holds, "t": round(t, 3), "snapshot": snap, "second_tilde_resumed": second})
            if not second:
                self.stop.set()
                return

    def _crawling(self):
        """stat 5 with every reported velocity under 1 mm/min for the last 5 s: running, but crawling."""
        now = time.monotonic()
        recent = [e for e in list(self.tg.transport.history)[-200:] if e.kind == "status" and isinstance(e.data, dict)]
        stat, vel, since = None, None, None
        for e in recent:
            sr = e.data.get("sr") or {}
            if "stat" in sr:
                stat = sr["stat"]
            if "vel" in sr:
                if vel is None or sr["vel"] >= 1.0 or vel >= 1.0:
                    since = e.t
                vel = sr["vel"]
        return stat == 5 and vel is not None and vel < 1.0 and since is not None and now - since > 5.0

    def _snapshot(self):
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
            return snap


def test_n70_resume_probe(tg, keep, record, request):
    ice = request.getfixturevalue("ice") if request.config.getoption("--ice") else None
    lines = [l.rstrip("\r") for l in open(FILE, "rb").read().decode("latin-1").split("\n") if l.strip()]
    for line in lines:
        tg.guard.check_gcode(line, allow="N70 probe on an unloaded bench")
    keep("ej", "rxm", "ex", "qv", "sv", "si")
    for token, value in {"ej": 1, "qv": 1, "sv": 1, **sender.MODES["line"]}.items():
        if tg.get(token) != value:
            tg.set(token, value, allow="N70 settings")
    for kv in [x for x in os.environ.get("N70_SETTINGS", "").split(",") if "=" in x]:   # e.g. aam=1,ara=10,4ma=0
        tok, val = kv.split("=")
        keep(tok)
        tg.set(tok, float(val) if "." in val else int(val), allow="N70 machine shape")
    rows = []
    for k in range(REPEATS):
        tg.send_gcode("G21 G90 G28.3 X0 Y0 Z0 A0", check=True, allow="declare the start")
        tg.send_line('{"rt":{"fi":0}}')
        assert tg.send_line('{"rt":{"fi":1}}').status == 0
        holder = _Holder(tg, ice, SEED + k)
        holder.start()
        err = None
        try:
            sender.stream(tg, lines, "line", timeout=600)
        except Exception as exc:
            err = repr(exc)[:300]
        holder.stop.set(); holder.join(timeout=10)
        tg.transport.write(b"~")
        tg.send_line('{"rt":{"fi":0}}')
        rows.append({"pass": k, "holds": holder.holds, "stuck": holder.stuck, "stream_error": err})
        print("N70", rows[-1])
        if holder.stuck:
            break
        tg.wait_idle(timeout=60)
    record("n70_probe", rows)
    stuck = [s for r in rows for s in r["stuck"]]
    assert not stuck, f"a '~' left the board in HOLD: {stuck}"
    errors = [(r["pass"], r["stream_error"]) for r in rows if r["stream_error"]]
    assert not errors, f"a stream failed: {errors}"
