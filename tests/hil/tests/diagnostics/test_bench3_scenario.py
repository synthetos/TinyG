"""Bench 3 (switchbox + MSO): seeded random scenarios that use every line the bench can see, checked against the real
pulses and output lines.

Each scenario queues one random program in G91: lines, rapids and XY arcs on X (M4), Y (M2 + M3) and Z (M1), with
M3/M4 S..., M5, M7/M8/M9 changes between them (each change is followed by a 0.6 s dwell so the line holds still long
enough to sample). One event lands at a random point: none, a feedhold then resume, a feedhold then queue flush (%),
or a limit trip on a random wired switch input. One MSO capture records the step and direction lines of all four
motors and the spindle PWM (plus the A min line when that is the input tripped); the switchbox's WiFi API polls the
spindle enable, spindle direction and coolant lines meanwhile.

Checks (Homing and Limits wiki: a limit hit stops at once, alarms and keeps position; E019: an alarm turns spindle
and coolant off; pwm.c cm_get_spindle_pwm: duty is the clamped S mapped onto the phase range, phase_off when off):
  - the board reports what the motors did: net signed steps of every motor equal its reported position change
  - the two Y motors step together
  - none / hold-resume: the move ends at the commanded total, the PWM duty plateaus follow the commanded spindle
    states in order, the output lines visit the commanded states in order and finish in the last one
  - hold-flush: what ran is a prefix of the program (PWM plateaus and line states)
  - limit: ALARM, spindle and coolant off, PWM at phase_off, the states before the trip a prefix of the program

SCENARIO_SEED picks the programs (default: random, printed and recorded); SCENARIO_RUNS how many (default 3).
"""
import json
import math
import os
import random
import time
import urllib.request

import pytest

from tinyg_hil import benches, mso

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.resets_board, pytest.mark.timeout(400)]

RUNS = int(os.environ.get("SCENARIO_RUNS", "3"))
BASE_SEED = int(os.environ.get("SCENARIO_SEED", "0")) or random.SystemRandom().randrange(1, 2**31)
EVENTS = ["none", "none", "hold", "hold", "flush", "limit", "limit", "limit"]
# A queued line is answered only when the planner has room for it: behind dwells (G4) or a long arc that can take
# seconds, so the 2 s default made correct flow control look like a lost line (most of the pre-2026-11-08 failures).
LINE_REPLY_S = 60.0
# Two duty levels closer than this are one level, both in the program and on the MSO. Merging the measured plateaus at
# 2 x 0.004 but the program's at 0.006 turned a 0.007 step (S2034 clamped to 0.200, then S1907 = 0.193) into a
# "missing" plateau.
DUTY_SAME = 0.006
SWITCH_TOKENS = {"xmin": "xsn", "xmax": "xsx", "ymin": "ysn", "ymax": "ysx", "zmin": "zsn", "amin": "asn"}
DWELL = 0.6
WHY = "bench 3 random scenario"


class _WebBox:
    """The switchbox over its WiFi REST API (its USB serial link stalls when requests interleave with MSO captures)."""

    def __init__(self, host):
        self.base = f"http://{host}"

    def _call(self, method, path, body=None, tries=4):
        for attempt in range(tries):
            try:
                req = urllib.request.Request(self.base + path, method=method,
                                             data=None if body is None else json.dumps(body).encode(),
                                             headers={"Content-Type": "application/json"})
                with urllib.request.urlopen(req, timeout=6) as r:
                    return json.load(r)
            except Exception:  # noqa: BLE001
                if attempt == tries - 1:
                    raise
                time.sleep(0.3)

    def lines(self, outputs):
        lv = {c["name"]: c["level"] for c in self._call("GET", "/api/state")["channels"]}
        return {k: lv[v] for k, v in outputs.items()}

    def pulse_channel(self, name, ms):
        return self._call("POST", f"/api/channels/{name}/pulse", {"ms": ms})

    def release_all(self):
        return self._call("POST", "/api/release_all", {})


def _rises(edges):
    return [t for t, v in edges if v == 1]


def _dedupe(seq, same=lambda a, b: a == b):
    out = []
    for x in seq:
        if not out or not same(out[-1], x):
            out.append(x)
    return out


def _is_subsequence(sub, seq, same=lambda a, b: a == b):
    it = iter(seq)
    return all(any(same(x, y) for y in it) for x in sub)


def _is_prefix(sub, seq, same=lambda a, b: a == b):
    return len(sub) <= len(seq) and all(same(a, b) for a, b in zip(sub, seq))


class _Program:
    """A random G91 program and what it should do."""

    def __init__(self, rng, caps, pwm):
        self.lines, self.total = [], {"x": 0.0, "y": 0.0, "z": 0.0}
        self.spindle = [("off", 0.0)]                 # (mode, S) after each spindle change, starting off
        self.coolant = [0]
        self.est = 0.0
        self.caps, self.pwm, self.rng = caps, pwm, rng
        self._move("G1", {"x": 0.5, "y": 0.5, "z": 0.5}, 600)     # learns each motor's + direction
        for _ in range(rng.randint(6, 11)):
            k = rng.random()
            if k < 0.45:
                self._random_move()
            elif k < 0.6:
                self._random_arc()
            elif k < 0.82:
                self._random_spindle()
            else:
                self._random_coolant()
            if self.est > 5.5:
                break

    def _room(self, ax, d, lim):
        return -lim <= self.total[ax] + d <= lim

    def _move(self, g, d, f=None):
        words = " ".join(f"{a.upper()}{v:.3f}" for a, v in d.items())
        self.lines.append(f"{g} {words}" + (f" F{f:g}" if f else ""))
        for a, v in d.items():
            self.total[a] += round(v, 3)
        dist = math.sqrt(sum(v * v for v in d.values()))
        feed = f if f else min(self.caps[a] for a in d)
        self.est += dist / (feed / 60.0) + 0.15

    def _random_move(self):
        rng, d = self.rng, {}
        for ax, lim, span in (("x", 25, 8), ("y", 25, 8), ("z", 6, 3)):
            if rng.random() < 0.6:
                v = round(rng.uniform(-span, span), 3)
                if abs(v) > 0.05 and self._room(ax, v, lim):
                    d[ax] = v
        if not d:
            return
        if rng.random() < 0.3:
            self._move("G0", d)
        else:
            cap = min(self.caps[a] for a in d)
            self._move("G1", d, round(rng.uniform(200, min(3000, cap)), 0))

    def _random_arc(self):
        rng = self.rng
        r = rng.uniform(1.0, 6.0)
        a0 = rng.uniform(0, 2 * math.pi)
        i, j = round(r * math.cos(a0), 3), round(r * math.sin(a0), 3)       # centre, relative to the start
        sweep = rng.uniform(0.3, 1.6 * math.pi) * (1 if rng.random() < 0.5 else -1)
        s = math.atan2(-j, -i)                                               # start angle seen from the centre
        ex, ey = round(i + r * math.cos(s + sweep), 3), round(j + r * math.sin(s + sweep), 3)
        if not (self._room("x", ex, 25) and self._room("y", ey, 25)):
            return
        f = round(rng.uniform(300, min(2000, self.caps["x"], self.caps["y"])), 0)
        g = "G3" if sweep > 0 else "G2"
        self.lines.append(f"G17 {g} X{ex:.3f} Y{ey:.3f} I{i:.3f} J{j:.3f} F{f:g}")
        self.total["x"] += ex
        self.total["y"] += ey
        self.est += abs(sweep) * r / (f / 60.0) + 0.15

    def _random_spindle(self):
        rng, p = self.rng, self.pwm
        mode = rng.choice(["cw", "cw", "ccw", "off"])
        if mode == "off":
            self.lines.append("M5")
            s = self.spindle[-1][1]
        else:
            lo, hi = (p["csl"], p["csh"]) if mode == "cw" else (p["wsl"], p["wsh"])
            s = round(rng.uniform(lo - 0.15 * (hi - lo), hi + 0.15 * (hi - lo)), 0)
            s = max(s, 0.0)
            self.lines.append(f"{'M3' if mode == 'cw' else 'M4'} S{s:g}")
        self.spindle.append((mode, s))
        self.lines.append(f"G4 P{DWELL}")
        self.est += DWELL + 0.1

    def _random_coolant(self):
        code = self.rng.choice(["M7", "M8", "M9"])
        self.lines.append(code)
        self.coolant.append(0 if code == "M9" else 1)
        self.lines.append(f"G4 P{DWELL}")
        self.est += DWELL + 0.1


def _duty(pwm, mode, s):
    if mode == "off":
        return pwm["pof"]
    lo, hi, plo, phi = ((pwm["csl"], pwm["csh"], pwm["cpl"], pwm["cph"]) if mode == "cw" else
                        (pwm["wsl"], pwm["wsh"], pwm["wpl"], pwm["wph"]))
    s = min(max(s, lo), hi)
    return phi if hi == lo else (s - lo) / (hi - lo) * (phi - plo) + plo


def _plateaus(rows, tol=0.004):
    """Duty levels held for at least two consecutive windows (a window that straddles a change stands alone)."""
    out = []
    for a, b in zip(rows, rows[1:]):
        if abs(a["duty"] - b["duty"]) <= tol:
            out.append(round((a["duty"] + b["duty"]) / 2, 4))
    return _dedupe(out, lambda x, y: abs(x - y) <= DUTY_SAME)   # the same tolerance as the program's levels


def _signed(step_edges, dir_edges, dir_initial, plus_level):
    level, di, net = dir_initial, 0, 0
    for t in _rises(step_edges):
        while di < len(dir_edges) and dir_edges[di][0] <= t:
            level = dir_edges[di][1]
            di += 1
        net += 1 if level == plus_level else -1
    return net


def _dir_at_first_step(step_edges, dir_edges, dir_initial):
    r = _rises(step_edges)
    if not r:
        return None
    level = dir_initial
    for t, v in dir_edges:
        if t > r[0]:
            break
        level = v
    return level


def _bench(tg):
    bench = benches.bench(tg.transport.port) or {}
    cfg, sbcfg = bench.get("mso") or {}, bench.get("switchbox") or {}
    if not cfg.get("digital") or not sbcfg.get("inputs") or not sbcfg.get("outputs") or not sbcfg.get("host"):
        pytest.skip("needs the MSO and the switchbox with outputs and a network address (bench 3)")
    pwm_name = (cfg.get("analog") or {}).get("0")
    amin_name = next((v for v in (cfg.get("analog") or {}).values() if "min" in v.lower()), None)
    return cfg, sbcfg, pwm_name, amin_name


def _motors(tg):
    """{motor: (axis, steps per mm)} for the motors mapped to X, Y or Z."""
    out = {}
    for m in (1, 2, 3, 4):
        ax = tg.get(f"{m}ma")
        if ax in (0, 1, 2):
            out[m] = ("xyz"[ax], 360.0 * tg.get(f"{m}mi") / (tg.get(f"{m}sa") * tg.get(f"{m}tr")))
    return out


def _pos(tg):
    return {a: tg.get(f"mpo{a}") for a in "xyz"}


def _recover(tg):
    try:
        st = tg.stat()
    except Exception:  # noqa: BLE001
        st = None
    if st == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    elif st is None or st not in tg.dialect.idle_stats:
        tg.send_raw(b"!")                       # % flushes only from a hold
        time.sleep(0.5)
        tg.send_raw(b"%")
        time.sleep(0.5)
        try:
            tg.wait_idle(timeout=20)
        except Exception:  # noqa: BLE001
            tg.reset()


@pytest.mark.parametrize("k", range(RUNS))
def test_bench3_scenario(tg, keep, record, request, k):
    cfg, sbcfg, pwm_name, amin_name = _bench(tg)
    seed = BASE_SEED + k
    rng = random.Random(seed)
    event = rng.choice(EVENTS)
    trip = rng.choice(sorted(SWITCH_TOKENS)) if event == "limit" else None
    keep(*SWITCH_TOKENS.values())
    # registered after keep's restore, so it runs first: a job still running at teardown refuses the switch settings
    # (status 117, N60) and left the bench on the trip's switch modes
    request.addfinalizer(lambda: _recover(tg))
    if trip:
        for name, tok in SWITCH_TOKENS.items():
            tg.set(tok, 2 if name == trip else 0, allow="one limit input for the trip")
        tg.reset()                                         # the switch modes take effect from a reset
    motors = _motors(tg)
    caps = {a: tg.get(f"{a}vm") for a in "xyz"}
    pwm = {t: tg.get(f"p1{t}") for t in ("csl", "csh", "cpl", "cph", "wsl", "wsh", "wpl", "wph", "pof")}
    prog = _Program(rng, caps, pwm)
    lines = ["G21 G91"] + prog.lines
    print("SCENARIO", seed, event, trip, f"est {prog.est:.1f}s", lines)
    for ln in lines:
        tg.guard.check_gcode(ln, allow=WHY)
    sb = _WebBox(sbcfg["host"])
    tg.send_gcode("M5 M9", allow=WHY)
    p0 = _pos(tg)
    digital = []
    for m, (ax, _) in sorted(motors.items()):
        digital += [f"M{m} step", f"M{m} dir"]
    analog = [pwm_name] + ([amin_name] if trip == "amin" and amin_name else [])
    secs = min(20.0, 3.5 + prog.est * 1.8 + 4.0)
    polled, exc_mark = [], len(tg.transport.history)
    t_event, stat_at_event, hold_stat = None, None, None
    try:
        sb.release_all()
        with mso.Capture(cfg, digital=digital, analog=analog, seconds=secs, rate=1_000_000, pwm_window=0.1,
                         save_as=request.config._tinyg_recorder.dir / f"scenario-{seed}") as cap:
            time.sleep(3.5)                                # the MSO starts recording late (test_mso_start_lag)
            t0 = time.monotonic()
            for ln in lines:
                tg.send_gcode(ln, allow=WHY, timeout=LINE_REPLY_S)   # a line waits for planner room behind dwells/arcs
            at = rng.uniform(0.25, 0.8) * prog.est
            end = t0 + secs - 4.0
            while time.monotonic() < end:
                now = time.monotonic()
                if t_event is None and event != "none" and now - t0 >= at:
                    t_event = now - t0
                    stat_at_event = tg.stat()
                    if event == "limit":
                        sb.pulse_channel(sbcfg["inputs"][trip], ms=100)
                    else:
                        tg.send_raw(b"!")
                        time.sleep(1.5)
                        hold_stat = tg.stat()
                        tg.send_raw(b"~" if event == "hold" else b"%")
                try:
                    polled.append(sb.lines(sbcfg["outputs"]))
                except Exception:  # noqa: BLE001
                    pass
                st = tg.stat()
                if (event == "none" or t_event is not None) and (
                        st == tg.dialect.stat("ALARM") or (st in tg.dialect.idle_stats and tg.get("qr") == 32)):
                    break
                time.sleep(0.12)
            finished_s = time.monotonic() - t0
            for _ in range(3):
                polled.append(sb.lines(sbcfg["outputs"]))
                time.sleep(0.1)
    finally:
        try:
            sb.release_all()
        except Exception:  # noqa: BLE001
            pass
    tg.drain()
    stat = tg.stat()
    p1 = _pos(tg)
    exc = [e.raw for e in tg.transport.history[exc_mark:] if e.kind == "exception"]
    e, init = cap.edges(), cap.initial()
    plus, net, counts = {}, {}, {}
    for m, (ax, spm) in motors.items():
        st_e, d_e = e.get(f"M{m} step", []), e.get(f"M{m} dir", [])
        plus[m] = _dir_at_first_step(st_e, d_e, init.get(f"M{m} dir", 0))
        net[m] = _signed(st_e, d_e, init.get(f"M{m} dir", 0), plus[m])
        counts[m] = len(_rises(st_e))
    rows = cap.pwm(pwm_name)
    seen_pwm = _plateaus(rows)
    want_pwm = _dedupe([round(_duty(pwm, md, s), 4) for md, s in prog.spindle], lambda a, b: abs(a - b) <= DUTY_SAME)
    enable_seq = _dedupe([p["spindle_enable"] for p in polled])
    cool_seq = _dedupe([p["coolant"] for p in polled])
    dir_seq = _dedupe([p["spindle_dir"] for p in polled if p["spindle_enable"] == 1])
    want_enable = _dedupe([0 if md == "off" else 1 for md, _ in prog.spindle])
    want_cool = _dedupe(prog.coolant)
    want_dir = _dedupe([1 if md == "ccw" else 0 for md, _ in prog.spindle if md != "off"])
    rec = {"seed": seed, "event": event, "trip": trip, "lines": lines, "est_s": round(prog.est, 2),
           "event_s": t_event, "stat_at_event": stat_at_event, "hold_stat": hold_stat, "finished_s": round(finished_s, 2),
           "stat": stat, "exceptions": exc, "start": p0, "end": p1, "total": prog.total,
           "motors": {m: {"axis": ax, "spm": spm, "steps": counts[m], "net": net[m], "plus_level": plus[m],
                          "reported_steps": round((p1[ax] - p0[ax]) * spm, 2)} for m, (ax, spm) in motors.items()},
           "pwm_seen": seen_pwm, "pwm_want": want_pwm, "enable": [enable_seq, want_enable],
           "coolant": [cool_seq, want_cool], "dir": [dir_seq, want_dir], "capture_s": secs}
    record("scenario", rec)
    print("RESULT", json.dumps(rec, default=str)[:3000])

    bad = []
    if finished_s > secs - 4.5:
        bad.append(f"the run did not finish inside the capture ({finished_s:.1f}s of {secs:.1f}s)")
    for m, (ax, spm) in motors.items():
        if abs(net[m] - (p1[ax] - p0[ax]) * spm) > 2.0 + 0.0011 * spm:   # a step at each end + 0.001 mm reports
            bad.append(f"M{m} ({ax}): {net[m]} net steps, reported {(p1[ax] - p0[ax]) * spm:.1f}")
    ys = [m for m, (ax, _) in motors.items() if ax == "y"]
    if len(ys) == 2 and counts[ys[0]] != counts[ys[1]]:
        bad.append(f"the Y motors stepped {counts[ys[0]]} and {counts[ys[1]]}")
    near = lambda a, b: abs(a - b) <= DUTY_SAME  # noqa: E731
    if event == "hold":
        # E060: with $spph (and $coph for coolant) a feedhold pauses the spindle and resumes it on ~, so the hold can
        # add one off stretch: between two equal on stretches, or where it landed on a speed change (it resumes at
        # the next commanded state). The sequence as seen is tried first, so a real M5 between close speeds counts.
        pwm_ok = lambda q: len(q) == len(want_pwm) and all(near(a, b) for a, b in zip(q, want_pwm))  # noqa: E731
        line_ok = lambda q, w: _is_subsequence(q, w) and not (q and w and q[-1] != w[-1])  # noqa: E731
        if _get0(tg, "spph"):
            seen_pwm = _first(_hold_variants(seen_pwm, pwm["pof"], near) + _averaged_pause(seen_pwm, pwm["pof"]), pwm_ok)
            enable_seq = _first(_hold_variants(enable_seq, 0), lambda q: line_ok(q, want_enable))
        if _get0(tg, "coph"):
            cool_seq = _first(_hold_variants(cool_seq, 0), lambda q: line_ok(q, want_cool))
    if event in ("none", "hold") and not bad:
        if stat not in tg.dialect.idle_stats:
            bad.append(f"stat {stat} after the program")
        for a in "xyz":
            if abs((p1[a] - p0[a]) - prog.total[a]) > 0.002:
                bad.append(f"{a} moved {p1[a] - p0[a]:.4f}, program {prog.total[a]:.4f}")
        if not (len(seen_pwm) == len(want_pwm) and all(near(a, b) for a, b in zip(seen_pwm, want_pwm))):
            bad.append(f"PWM plateaus {seen_pwm}, program {want_pwm}")
        for name, got, want in (("enable", enable_seq, want_enable), ("coolant", cool_seq, want_cool),
                                ("dir", dir_seq, want_dir)):
            if not _is_subsequence(got, want) or (got and want and got[-1] != want[-1]):
                bad.append(f"{name} line went {got}, program {want}")
    elif event == "flush":
        if stat not in tg.dialect.idle_stats:
            bad.append(f"stat {stat} after the flush")
        ran = seen_pwm[:-1] if len(seen_pwm) > 1 and near(seen_pwm[-1], pwm["pof"]) else seen_pwm
        if not _is_prefix(ran, want_pwm, near):
            bad.append(f"PWM plateaus {seen_pwm} are not a start of the program's {want_pwm}")
        for name, got, want in (("enable", enable_seq, want_enable), ("coolant", cool_seq, want_cool)):
            if not _is_subsequence(got[:-1] if got and got[-1] == 0 else got, want):
                bad.append(f"{name} line went {got}, program {want}")
    elif event == "limit":
        if stat != tg.dialect.stat("ALARM"):
            bad.append(f"stat {stat} after the {trip} limit (want ALARM)")
        last = polled[-1] if polled else {}
        if last.get("spindle_enable") != 0 or last.get("coolant") != 0:
            bad.append(f"outputs after the limit: {last}")
        if not seen_pwm or not near(seen_pwm[-1], pwm["pof"]):
            bad.append(f"PWM after the limit {seen_pwm[-1:]} (want phase_off {pwm['pof']})")
        ran = seen_pwm[:-1] if len(seen_pwm) > 1 else seen_pwm
        if not _is_prefix(ran, want_pwm, near) and not _is_prefix(ran, want_pwm + [pwm["pof"]], near):
            bad.append(f"PWM plateaus {seen_pwm} before the limit are not a start of the program's {want_pwm}")
    try:
        _recover(tg)
        tg.send_gcode("M5 M9", allow=WHY)
        back = {a: round(p0[a] - tg.get(f"mpo{a}"), 3) for a in "xyz"}
        if any(abs(v) > 0.001 for v in back.values()):
            tg.send_gcode("G91 G0 " + " ".join(f"{a.upper()}{v:.3f}" for a, v in back.items()), allow="return")
            tg.wait_idle(timeout=60)
        tg.send_gcode("G90", allow="absolute")
    finally:
        pass
    assert not bad, f"seed {seed} {event} {trip}: {bad}"


def _hold_variants(seq, off, same=lambda a, b: a == b):
    """The sequence as seen, then with one hold pause taken out: an off entry between two equal entries (both
    dropped as one), then any single off entry that is not at either end."""
    out = [list(seq)]
    for i in range(1, len(seq) - 1):
        if same(seq[i], off) and same(seq[i - 1], seq[i + 1]) and not same(seq[i - 1], off):
            out.append(list(seq[:i]) + list(seq[i + 2:]))
    for i in range(1, len(seq) - 1):
        if same(seq[i], off):
            out.append(list(seq[:i]) + list(seq[i + 1:]))
    return out


def _averaged_pause(seq, off, tol=3 * DUTY_SAME):
    """A pause too short for its own plateau: the duty window across it reads a little below the speed around it.
    Variants with one such entry (between its predecessor and off, within tol of the predecessor) dropped."""
    out = []
    for i in range(1, len(seq)):
        lo, hi = sorted((seq[i - 1], off))
        if lo <= seq[i] <= hi and abs(seq[i] - seq[i - 1]) <= tol and abs(seq[i - 1] - off) > tol:
            out.append(list(seq[:i]) + list(seq[i + 1:]))
    return out

def _first(variants, ok):
    """The first variant that passes ok (the seen sequence when none does, so the failure shows what was seen)."""
    return next((v for v in variants if ok(v)), variants[0])


def _get0(tg, token):
    """A setting's value, or 0 on firmware without it (before E060 there is no $spph/$coph)."""
    try:
        return tg.get(token) or 0
    except Exception:  # noqa: BLE001
        return 0
