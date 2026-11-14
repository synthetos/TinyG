"""Soak (overnight): stream any G-code file in one flow-control mode, return to the start, check the board.

SOAK_M0=resume resumes reported M0/M1 program stops after a 0.5 s settle and records every start.
It does not resume feedhold or extend stall deadlines. Default off. Master 440 defaults to count mode;
edge defaults to line mode. Explicit line/g2 mode is refused on master, which has no rxm setting.
SOAK_FILE: the file (lines as given, blank lines dropped); SOAK_MODE: line, g2, rts or count;
SOAK_SETTINGS="1mi=8,1sa=0.9,3ma=3,aam=3,ara=10" applies those settings first (restored afterwards): motor
geometry (step angle, microsteps, travel per rev) and axis mapping or rotary mode, so the same checks cover other
machines. SOAK_HOLDS=N sends N feedholds ('!', then '~' 0.4-1.5 s later) at random moments during the stream (unset: one
stream in three gets 3, chosen at random and recorded); the end checks are the same, so a hold or resume that
loses, repeats or misplaces motion fails. SOAK_FI=1 also turns on feed-limit push ({"rt":{"fi":1}}) and the heartbeat during the stream: every fl/fs line
must parse, summaries must count up from q 1 in one window, and the final summary's cutting distance is kept.
Pass: the stream runs through without a stall (sender.StreamStalled) or a lost board, the board answers
afterwards, and after a G53 G0 back to the starting machine position it reports that position. On a bench
with a Saleae (bench 2) the X/Y/Z step and direction pulses are captured as well, and every motor's net
step count must come back to zero (no step lost or added). Per-line errors, exceptions and any alarm are
recorded for review but do not fail the test: ugly files are expected to have refused lines.
"""
import bisect
import os
import random
import re
import threading
import time

import pytest

from tinyg_hil import benches, hostlines, saleae, sender, soak, snapshot

pytestmark = [pytest.mark.timeout(5400), pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.dialect("d440", "d449")]

FILE = os.environ.get("SOAK_FILE", "")
MODE = os.environ.get("SOAK_MODE")
POS_TOL = 0.002                                          # mm
CAPTURE_MAX_LINES = int(os.environ.get("SOAK_CAPTURE_MAX", "40000"))
HOLDS = int(os.environ["SOAK_HOLDS"]) if "SOAK_HOLDS" in os.environ else (3 if random.random() < 1 / 3 else 0)


class _Holder(threading.Thread):
    """Feedholds at random moments while the stream runs: '!' then '~' 0.4-1.5 s later."""
    def __init__(self, tg, count, seed, lock):
        super().__init__(daemon=True)
        self.lock = lock
        self.tg, self.count, self.r, self.stop, self.done = tg, count, random.Random(seed), threading.Event(), []

    def run(self):
        for _ in range(self.count):
            if self.stop.wait(self.r.uniform(2.0, 8.0)):
                return
            with self.lock:
                if self.stop.is_set():return
                t = time.monotonic()
                self.tg.transport.write(b"!")
                self.stop.wait(self.r.uniform(0.4, 1.5))
                self.tg.transport.write(b"~")
                self.done.append(round(t, 2))


def _net_steps(edges, initial, step_ch, dir_ch):
    dirs = edges.get(dir_ch, [])
    times = [t for t, _ in dirs]
    net = 0
    for t, level in edges.get(step_ch, []):
        if not level:
            continue
        i = bisect.bisect_right(times, t) - 1
        d = dirs[i][1] if i >= 0 else initial.get(dir_ch, 0)
        net += 1 if d else -1
    return net


AXES = "xyza"


def _pulses(edges, step_ch):
    return sum(1 for _, level in edges.get(step_ch, []) if level)


def _mpo(tg):
    return {a: tg.get(f"mpo{a}") for a in AXES}


def test_soak_file(tg, keep, record, request):
    if not FILE:
        pytest.skip("SOAK_FILE not set")
    mode=soak.mode_for(tg.dialect,MODE)
    resume=soak.resume_policy(os.environ.get("SOAK_M0", ""))
    lock=threading.Lock()
    resumer=soak.ProgramStopResumer(tg,lock) if resume else None
    if os.environ.get("SOAK_FI")=="1" and tg.dialect.name=="d440":
        pytest.skip("SOAK_FI requires the edge runtime report family")
    ALARMED.clear()
    raw = open(FILE, "rb").read().decode("latin-1")
    lines = [l.rstrip("\r") for l in raw.split("\n") if l.strip()]
    lines, stripped = hostlines.host_safe(lines, raw=os.environ.get("SOAK_RAW") == "1")   # N67, N101
    for line in lines:
        tg.guard.check_gcode(line, allow="soak file on an unloaded bench")
    keep(*[k for k in ("ej", "rxm", "ex", "qv", "sv", "si") if k in tg.dialect.tokens])
    geo = [kv.split("=") for kv in os.environ.get("SOAK_SETTINGS", "").split(",") if "=" in kv]
    for token, value in geo:
        keep(token)
    for token, value in {"ej": 1, "qv": 1, "sv": 1, **sender.MODES[mode]}.items():
        if token not in tg.dialect.tokens:continue
        if tg.get(token) != value:
            tg.set(token, value, allow="soak settings")
    for token, value in geo:                             # last, so a combination can override sv/qv too
        tg.set(token, float(value) if "." in value else int(value), allow="soak geometry")
    if resumer:
        original_sr=list(tg.get("sr"))
        request.addfinalizer(lambda: snapshot.set_status_report(tg,original_sr))
        snapshot.set_status_report(tg,[*dict.fromkeys([*original_sr,"stat"])])
        tg.set("sv",1,allow="program-stop observations")
        tg.set("si",100,allow="program-stop observations")
    tg.send_gcode("G21 G90", check=True)
    start = _mpo(tg)
    fi = os.environ.get("SOAK_FI") == "1"
    mark = len(tg.transport.history)
    if fi:
        tg.send_line('{"rt":{"fi":0}}')                  # a fresh window: fi 1 while already on only changes the interval
        assert tg.send_line('{"rt":{"fi":1}}').status == 0
        tg.send_line('{"rt":{"hb":1}}')
    bench = benches.bench(tg.transport.port)
    chan = {v: int(k) for k, v in (bench.get("saleae_channels") or {}).items()}
    use_saleae = bool(bench.get("saleae")) and len(lines) <= CAPTURE_MAX_LINES and all(
        f"M{m} step" in chan for m in (1, 2, 3))
    if re.search(r"(^|[^A-Za-z])A-?[0-9.]", raw, re.M) and tg.get("aam") == 3 and float(tg.get("ara")) < 1.0:
        pytest.skip(f"the board's A axis is in radius mode with ara {tg.get('ara')}: A words would mean thousands "
                    "of degrees per unit (N48); set aam=1 or a sane ara before soaking A files")
    seed = int(os.environ.get("SOAK_HOLD_SEED", "0")) or random.randrange(1, 1 << 30)
    info = {"file": FILE, "mode": mode, "lines": len(lines), "host_safe": stripped, "start": start, "saleae": use_saleae, "holds": HOLDS,
            "settings": os.environ.get("SOAK_SETTINGS", ""),
            "hold_seed": seed}
    holder = _Holder(tg, HOLDS, seed, lock) if HOLDS else None
    t0 = time.monotonic()
    res = None
    try:
        if use_saleae:
            channels = [chan[f"M{m} {k}"] for m in (1, 2, 3) for k in ("step", "dir")]
            stem = request.config._tinyg_recorder.dir / f"soak-file-{os.path.basename(FILE)}-{mode}"
            with saleae.Capture(bench["saleae"], channels=channels, seconds=None, rate=2_000_000, save_as=stem) as cap:
                if holder:
                    holder.start()
                res = _stream(tg, lines, request, record, mode, resumer)
                if holder:
                    holder.stop.set(); holder.join(); tg.transport.write(b"~")
                _settle(tg, 600, resumer)
                _return(tg, start)
            edges, initial = cap.edges(), cap.initial
            axis_of = {m: tg.get(f"{m}ma") for m in (1, 2, 3)}
            info["net_steps"] = {f"M{m}(axis {axis_of[m]})": _net_steps(edges, initial, chan[f"M{m} step"], chan[f"M{m} dir"])
                                 for m in (1, 2, 3)}
            info["pulses"] = {m: _pulses(edges, chan[f"M{m} step"]) for m in (1, 2, 3)}
            info["shared_axes"] = {}
            spu = {m: 360.0 * tg.get(f"{m}mi") / (tg.get(f"{m}sa") * tg.get(f"{m}tr")) for m in (1, 2, 3)}
            info["steps_per_unit"] = spu
            for a in set(axis_of.values()):              # two motors on one axis (a gantry) must travel identically
                ms = [m for m in (1, 2, 3) if axis_of[m] == a]
                if len(ms) > 1:                          # compare travel, not raw pulses: their geometry may differ
                    info["shared_axes"][a] = {m: {"pulses": info["pulses"][m], "travel": info["pulses"][m] / spu[m],
                                                  "step": 1.0 / spu[m]} for m in ms}
        else:
            if holder:
                holder.start()
            res = _stream(tg, lines, request, record, mode, resumer)
            if holder:
                holder.stop.set(); holder.join(); tg.transport.write(b"~")
            _settle(tg, 600, resumer)
            _return(tg, start)
    finally:
        info["m0_policy"] = "resume" if resumer else "off"
        info["program_stop_resumes"] = list(resumer.events) if resumer else []
        info["program_stop_resume_count"] = len(info["program_stop_resumes"])
        info["seconds"] = round(time.monotonic() - t0, 1)
        if holder:
            holder.stop.set()
            holder.join(timeout=10)                      # never let a sleeping holder resume after a stop
            info["holds_sent"] = len(holder.done)
        if fi:
            time.sleep(1.5)                              # the final summary follows the drain
            fs = [e.data["fs"] for e in tg.transport.history[mark:] if e.kind == "feedsummary"]
            fl = [e.data["fl"] for e in tg.transport.history[mark:] if e.kind == "feedlimit"]
            bad = [e.raw for e in tg.transport.history[mark:] if e.kind == "malformed"]
            info["push"] = {"fs": len(fs), "fl": len(fl), "malformed": bad[:5], "last_fs": fs[-1] if fs else None}
            tg.send_line('{"rt":{"fi":0}}')
            tg.send_line('{"rt":{"hb":0}}')
            if fs:
                assert [s["q"] for s in fs] == list(range(1, len(fs) + 1)), f"fs q not consecutive: {[s['q'] for s in fs]}"
            assert not bad, f"malformed lines during the stream: {bad[:5]}"
        if res is not None:
            statuses = {}
            for _, status, _ in res.errors:
                statuses[status] = statuses.get(status, 0) + 1
            info.update({"acked": res.acked, "error_statuses": statuses, "errors": res.errors[:20],
                         "exceptions": res.exceptions[:10]})
        record("soak_file", info)
    end = _mpo(tg)
    info["end"] = end
    record("soak_file_end", {"end": end, "start": start})
    assert tg.stat() not in tg.dialect.stats("PANIC", "SHUTDOWN"), "board panicked or shut down"
    if ALARMED:
        first = (info.get("exceptions") or [None])[0]
        raise AssertionError(f"ALARM during the stream of {os.path.basename(FILE)} ({mode}): {str(first)[:200]}")
    assert all(abs(end[a] - start[a]) <= POS_TOL for a in AXES), f"not back at the start: {start} -> {end}"
    if use_saleae:
        bad = {k: v for k, v in info["net_steps"].items() if abs(v) > 1}
        assert not bad, f"net steps after returning to the start: {info['net_steps']}"
        # A gantry's motors are judged by net steps (above). Their total pulse counts are recorded only: two motors
        # with different DDA phase history (N37: 0.1 and 0.9 of a period) legitimately differ by several pulses
        # in total over a reversing path while both net to zero, and different resolutions round differently.


ALARMED = []


def _return(tg, start):
    if tg.stat() == tg.dialect.stat("ALARM"):
        ALARMED.append(True)
        tg.clear_alarm()
    tg.send_gcode("M5 M9 G92.1 G90 G21 G94 G64", allow="soak cleanup")
    tg.send_gcode(f"G53 G0 X{start['x']:.4f} Y{start['y']:.4f} Z{start['z']:.4f} A{start['a']:.4f}", check=True,
                  allow="return to the start")
    _settle(tg, 600)

def _stream(tg, lines, request, record, mode, resumer):
    """The stream; on a stall with --ice, the board's state is read before anything else touches it."""
    if resumer:resumer.begin()
    try:
        return sender.stream(tg, lines, mode, timeout=5000, on_status=resumer)
    except (sender.StreamStalled, sender.StreamRunaway):
        if request.config.getoption("--ice"):
            record("stall_snapshot", sender.stall_snapshot(request.getfixturevalue("ice")))
        raise
    except TimeoutError:
        # the job is still running (a slow laser file, N75): stop it, or every restore after this is refused
        # with status 117 because a job is running (N60) and the board is left on the test's settings
        tg.reset()
        raise


def _settle(tg, timeout, resumer=None):
    """Wait until idle, or return at once when the board is in ALARM (it never becomes idle then)."""
    end = time.monotonic() + timeout
    alarm, idle = tg.dialect.stat("ALARM"), tg.dialect.stats("READY", "PROGRAM_STOP", "PROGRAM_END")
    while time.monotonic() < end:
        s = tg.stat()
        empty = s in idle and tg.get("qr") == 32
        if resumer and s == tg.dialect.stat("PROGRAM_STOP"):
            resumer(time.monotonic(), {"stat": s})
        if s == alarm or empty:
            return s
        time.sleep(0.1)
    raise AssertionError(f"not idle and no alarm after {timeout} s (stat {tg.stat()})")
