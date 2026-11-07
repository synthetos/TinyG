"""Bench 3 (MSO + switchbox): a real G-code file streamed in windows, every window checked against the real pulses.

MSO captures are time-bounded, so the file is cut into windows of about MSO_WINDOW_S seconds of estimated motion at
random places (seeded). Before each window a preamble puts the board in the file's modal state at that point (units,
distance mode, plane, feed mode, work offset, feed, spindle, coolant, motion mode) and rapids to the file's position
there; the window's own lines then stream unchanged (line mode by default) inside one MSO capture of the step and
direction lines of every motor and the spindle PWM, and the board settles. Checks per window:
  - net signed steps of every motor equal its reported position change (the reports are the real motion)
  - the two Y motors step together
  - once the window has settled: PWM duty = the file's spindle state there (pwm.c cm_get_spindle_pwm: S clamped to
    the range and mapped onto the phase range; phase_off when off) and the spindle enable, direction and coolant
    lines (switchbox) match it
Offsets set by the file (G92, G10) are not replayed: the checks are relative, so absolute positions do not matter.
Windows stop before M0/M1/M60/M6 and program ends (they wait for an operator). Lines the board refuses (codes it
does not support) are recorded, not failed, as in test_soak_file.

MSO_FILE (required), MSO_MODE (line), MSO_SEED, MSO_WINDOWS (6), MSO_WINDOW_S (7), MSO_SETTINGS (tok=val,...).
MSO_FIT_S=1 (default) sets the spindle speed ranges ($p1csl/csh, $p1wsl/wsh) to 0..the file's largest S for the
test, so the file's own S values land inside the PWM range instead of all clamping to one end.
"""
import json
import math
import os
import random
import re
import time
import urllib.request

import pytest

from tinyg_hil import benches, mso, sender

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(3600)]

FILE = os.environ.get("MSO_FILE", "")
FIT_S = os.environ.get("MSO_FIT_S", "1") == "1"
MODE = os.environ.get("MSO_MODE", "line")
SEED = int(os.environ.get("MSO_SEED", "0")) or random.SystemRandom().randrange(1, 2**31)
WINDOWS = int(os.environ.get("MSO_WINDOWS", "6"))
WINDOW_S = float(os.environ.get("MSO_WINDOW_S", "7"))
WHY = "MSO file windows on an unloaded bench"
STOPPERS = re.compile(r"M0*(0|1|2|30|60|6)(?![0-9.])")
WORD = re.compile(r"([A-Z])\s*([-+]?(?:[0-9]+\.?[0-9]*|\.[0-9]+))")


class _WebBox:
    """The switchbox over its WiFi REST API (its USB serial link stalls when requests interleave with MSO captures)."""

    def __init__(self, host):
        self.base = f"http://{host}"

    def lines(self, outputs, tries=4):
        for attempt in range(tries):
            try:
                with urllib.request.urlopen(self.base + "/api/state", timeout=6) as r:
                    lv = {c["name"]: c["level"] for c in json.load(r)["channels"]}
                return {k: lv[v] for k, v in outputs.items()}
            except Exception:  # noqa: BLE001
                if attempt == tries - 1:
                    raise
                time.sleep(0.3)


def _code(raw):
    s = re.sub(r"\([^)]*\)", "", raw).split(";")[0].strip().upper()
    return s


class _Modal:
    """The file's modal state line by line (positions in mm, work coordinates, endpoints only)."""

    def __init__(self):
        self.units, self.dist, self.plane, self.fmode, self.wcs, self.motion = "G21", "G90", "G17", "G94", "G54", "G0"
        self.f, self.s, self.spindle, self.cool = None, 0.0, "off", 0
        self.pos = {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0}
        self.est = 0.0

    def step(self, raw, caps):
        s = _code(raw)
        if not s or s[0] in "{$%!~":
            return 0.0
        words = WORD.findall(s)
        gs = [v for k, v in words if k == "G"]
        ms = [float(v) for k, v in words if k == "M"]
        nonmodal_axes = False
        for g in gs:
            gv = float(g)
            if gv in (20, 21):
                self.units = f"G{int(gv)}"
            elif gv in (90, 91):
                self.dist = f"G{int(gv)}"
            elif gv in (17, 18, 19):
                self.plane = f"G{int(gv)}"
            elif gv in (93, 94):
                self.fmode = f"G{int(gv)}"
            elif 54 <= gv <= 59 and gv == int(gv):
                self.wcs = f"G{int(gv)}"
            elif gv in (0, 1, 2, 3):
                self.motion = f"G{int(gv)}"
            elif gv == 80:
                self.motion = "G80"
            elif gv in (4, 10, 28, 28.1, 28.2, 28.3, 30, 30.1, 53, 92, 92.1, 92.2, 92.3, 38.2):
                nonmodal_axes = True
        k = 25.4 if self.units == "G20" else 1.0
        w = {kk: float(v) for kk, v in words if kk in "XYZAFS"}
        if "F" in w and w["F"] < 0:                       # refused by the parser: the axis words move nothing
            self.est += 0.04
            return 0.04
        if "F" in w and w["F"] > 0:
            self.f = w["F"]
        t = 0.0
        if "S" in w and w["S"] != self.s:
            self.s = w["S"]
            t += 0.6
        for m in ms:
            if m in (3, 4):
                self.spindle = "cw" if m == 3 else "ccw"
                t += 0.6
            elif m == 5:
                self.spindle = "off"
                t += 0.6
            elif m in (7, 8):
                self.cool = 1
                t += 0.6
            elif m == 9:
                self.cool = 0
                t += 0.6
            elif m in (2, 30):
                self.spindle, self.cool = "off", 0
                self.dist, self.plane, self.fmode, self.wcs = "G90", "G17", "G94", "G54"
        axes = {a: w[a] for a in "XYZA" if a in w}
        if axes and not nonmodal_axes and self.motion in ("G0", "G1", "G2", "G3"):
            new = dict(self.pos)
            for a, v in axes.items():
                vv = v if a == "A" else v * k
                new[a] = vv if self.dist == "G90" else self.pos[a] + vv
            d = math.dist([self.pos[a] for a in "XYZ"], [new[a] for a in "XYZ"])
            if self.motion == "G0":
                t += d / (caps / 60.0) + 0.05
            elif self.fmode == "G93":
                t += 60.0 / self.f if self.f else 1.0
            else:
                t += (d * (1.4 if self.motion in ("G2", "G3") else 1.0)) / ((self.f or 100) * k / 60.0) + 0.02
            self.pos = new
        t = max(t, 0.04)                                  # the board parses about 25 lines a second
        self.est += t
        return t

    def preamble(self):
        out = ["G21 G90 G17 G94 M9", self.wcs,             # the file's work offset first: the position is in it
               "G0 " + " ".join(f"{a}{self.pos[a]:.4f}" for a in "XYZA"),
               f"{self.units} {self.dist} {self.plane}"]
        if self.f:
            out.append(f"G94 F{self.f:g}")
        if self.fmode == "G93":
            out.append("G93")
        out.append(f"{'M3' if self.spindle == 'cw' else 'M4'} S{self.s:g}" if self.spindle != "off" else f"M5 S{self.s:g}")
        out.append("M8" if self.cool else "M9")
        if self.motion in ("G0", "G1"):
            out.append(self.motion)
        return out


def _duty(pwm, mode, s):
    if mode == "off":
        return pwm["pof"]
    lo, hi, plo, phi = ((pwm["csl"], pwm["csh"], pwm["cpl"], pwm["cph"]) if mode == "cw" else
                        (pwm["wsl"], pwm["wsh"], pwm["wpl"], pwm["wph"]))
    s = min(max(s, lo), hi)
    return phi if hi == lo else (s - lo) / (hi - lo) * (phi - plo) + plo


def _rises(edges):
    return [t for t, v in edges if v == 1]


def _one_startup_pulse(a, b):
    """Opposite initial DIR phases may differ once; every following pulse must pair."""
    if len(a) < len(b):
        a, b = b, a
    return (len(a) == len(b) + 1 and bool(b) and a[0] < b[0] and
            all(abs(x - y) <= 20e-6 for x, y in zip(a[1:], b)))


def _signed(step_edges, dir_edges, dir_initial, plus_level):
    level, di, net = dir_initial, 0, 0
    for t in _rises(step_edges):
        while di < len(dir_edges) and dir_edges[di][0] <= t:
            level = dir_edges[di][1]
            di += 1
        net += 1 if level == plus_level else -1
    return net


def _dead(tg):
    """States that never become idle without a reset."""
    return set(tg.dialect.stats("SHUTDOWN", "PANIC"))


def _settle(tg, timeout):
    end = time.monotonic() + timeout
    alarm, idle = tg.dialect.stat("ALARM"), tg.dialect.stats("READY", "PROGRAM_STOP", "PROGRAM_END")
    while time.monotonic() < end:
        s = tg.stat()
        if s == alarm or s in _dead(tg) or (s in idle and tg.get("qr") == 32):
            return s
        time.sleep(0.1)
    return None


def _windows(lines, caps, rng):
    """[(start, end, modal at start, est)]: windows of ~WINDOW_S estimated seconds, no stopper inside."""
    scan, states, times = _Modal(), [], []
    for ln in lines:
        states.append(json.loads(json.dumps(scan.__dict__)))
        times.append(scan.step(ln, caps))
    states.append(json.loads(json.dumps(scan.__dict__)))    # states[k] is the state before line k, so after line k-1
    starts = sorted(rng.sample(range(len(lines)), min(WINDOWS * 3, len(lines))))
    out, last_end = [], -1
    for i in starts:
        if i <= last_end:
            continue
        g = i                                              # a bare arc continuation needs its arc's own G word:
        while g < len(lines) and g - i < 200 and states[g]["motion"] in ("G2", "G3") and not re.search(
                r"G0*[0-3](?![0-9.])", _code(lines[g])):  # start at the next line that names its motion
            g += 1
        if g >= len(lines) or g - i >= 200:
            continue
        i = g
        t, j = 0.0, i
        while j < len(lines) and t < WINDOW_S and j - i < 600:
            if STOPPERS.search(_code(lines[j])):
                break
            t += times[j]
            j += 1
        if j - i >= 3 and t >= 1.0:
            m = _Modal()
            m.__dict__.update(json.loads(json.dumps(states[i])))
            out.append((i, j, m, t, states[j]))
            last_end = j
        if len(out) >= WINDOWS:
            break
    return out, scan


def test_bench3_file_mso(tg, keep, record, request):
    if not FILE:
        pytest.skip("MSO_FILE not set")
    bench = benches.bench(tg.transport.port) or {}
    cfg, sbcfg = bench.get("mso") or {}, bench.get("switchbox") or {}
    if not cfg.get("digital"):
        pytest.skip("needs the MSO (bench 3)")
    pwm_name = (cfg.get("analog") or {}).get("0")
    box = _WebBox(sbcfg["host"]) if sbcfg.get("host") and sbcfg.get("outputs") else None
    raw = open(FILE, "rb").read().decode("latin-1")
    lines = [ln.rstrip("\r") for ln in raw.split("\n") if ln.strip()]
    for ln in lines:
        tg.guard.check_gcode(ln, allow=WHY)
    keep("ej", "rxm", "ex", "qv", "sv", "si")
    geo = [kv.split("=") for kv in os.environ.get("MSO_SETTINGS", "").split(",") if "=" in kv]
    for token, _ in geo:
        keep(token)
    for token, value in {"ej": 1, "qv": 1, "sv": 1, **sender.MODES[MODE]}.items():
        if tg.get(token) != value:
            tg.set(token, value, allow="stream settings")
    for token, value in geo:
        tg.set(token, float(value) if "." in value else int(value), allow="file geometry")
    if re.search(r"(^|[^A-Za-z])A-?[0-9.]", raw, re.M) and tg.get("aam") == 3 and float(tg.get("ara")) < 1.0:
        pytest.skip("A in radius mode with a tiny ara (N48)")
    s_max = max([float(v) for ln in lines for k, v in WORD.findall(_code(ln)) if k == "S"] or [0.0])
    if FIT_S and s_max > 0:
        keep("p1csl", "p1csh", "p1wsl", "p1wsh")
        for tok, v in (("p1csl", 0), ("p1csh", s_max), ("p1wsl", 0), ("p1wsh", s_max)):
            tg.set(tok, v, allow="fit the spindle range to the file")
    motors = {}
    for m in (1, 2, 3, 4):
        ax = tg.get(f"{m}ma")
        if ax in (0, 1, 2, 3):
            motors[m] = ("xyza"[ax], 360.0 * tg.get(f"{m}mi") / (tg.get(f"{m}sa") * tg.get(f"{m}tr")))
    # dir line level for + travel: low with $Npo=0 (bench 3 scenario runs, every motor), so high with $Npo=1
    plus_level = {m: 1 if tg.get(f"{m}po") else 0 for m in motors}
    pwm = {t: tg.get(f"p1{t}") for t in ("csl", "csh", "cpl", "cph", "wsl", "wsh", "wpl", "wph", "pof")}
    if FIT_S and s_max > 0:                                # speeds read back rounded to whole numbers: use what was set
        pwm.update({"csl": 0.0, "csh": s_max, "wsl": 0.0, "wsh": s_max})
    caps = min(tg.get(f"{a}vm") for a in "xy")
    rng = random.Random(SEED)
    wins, scan = _windows(lines, caps, rng)
    if not wins:
        pytest.skip("no window fits this file")
    tg.send_gcode("G21 G90", check=True)
    start = {a: tg.get(f"mpo{a}") for a in "xyza"}
    digital = [f"M{m} {k}" for m in sorted(motors) for k in ("step", "dir")]
    results, bad = [], []
    print("FILE", FILE, "seed", SEED, "lines", len(lines), "est_total_s", round(scan.est, 1),
          "windows", [(i, j, round(t, 1)) for i, j, _, t, _ in wins])
    try:
        for n, (i, j, modal, est, end_state) in enumerate(wins):
            pre = modal.preamble()
            for ln in pre:
                tg.guard.check_gcode(ln, allow=WHY)
            r_pre = sender.stream(tg, pre, MODE, timeout=600)
            st_pre = _settle(tg, 600)                      # a preamble rapid can be long (a file's far start point)
            if st_pre in _dead(tg) or st_pre == tg.dialect.stat("ALARM"):
                exc = [e.raw for e in tg.transport.history if e.kind == "exception"][-2:]
                bad.append(f"window {n} lines {i}-{j}: stat {st_pre} during the preamble {pre} {exc}")
                break
            p0 = {a: tg.get(f"mpo{a}") for a in "xyza"}
            secs = min(25.0, 1.0 + est * 2.5 + 3.0)
            mark = len(tg.transport.history)
            with mso.Capture(cfg, digital=digital, analog=[pwm_name] if pwm_name else [], seconds=secs,
                             rate=1_000_000, pwm_window=0.1,
                             save_as=request.config._tinyg_recorder.dir / f"win{n}-{i}") as cap:
                time.sleep(0.3)                                # data starts ~50 ms after STARTED (test_mso_start_lag)
                t0 = time.monotonic()
                res = sender.stream(tg, lines[i:j], MODE, timeout=max(30.0, secs * 4))
                stat = _settle(tg, max(1.0, secs - 0.6 - (time.monotonic() - t0)))
                done_s = time.monotonic() - t0
                outs = box.lines(sbcfg["outputs"]) if box and stat is not None else None
            if stat is None:
                stat = _settle(tg, 600)
            p1 = {a: tg.get(f"mpo{a}") for a in "xyza"}
            exc = [e.raw for e in tg.transport.history[mark:] if e.kind == "exception"]
            e, init = cap.edges(), cap.initial()
            r = {"window": n, "lines": [i, j], "est_s": round(est, 2), "done_s": round(done_s, 2), "capture_s": secs,
                 "stat": stat, "ok": bool(getattr(res, "ok", True)), "exceptions": exc[:5], "preamble": pre,
                 "preamble_ok": bool(getattr(r_pre, "ok", True)), "motors": {}}
            settled_in_capture = done_s < secs - 0.4
            for m, (ax, spm) in motors.items():
                st_e, d_e = e.get(f"M{m} step", []), e.get(f"M{m} dir", [])
                rep = (p1[ax] - p0[ax]) * spm
                plus = plus_level[m]
                net = _signed(st_e, d_e, init.get(f"M{m} dir", 0), plus)
                r["motors"][m] = {"axis": ax, "steps": len(_rises(st_e)), "net": net, "reported_steps": round(rep, 2),
                                  "plus_level": plus}
                # a move from rest to rest can end up to a step from its fractional position at each end, and the
                # reported positions have 0.001 mm resolution (a bench 1 ICE check of a whole file: within 1 step)
                if settled_in_capture and abs(net - rep) > 2.0 + 0.0011 * spm:
                    bad.append(f"window {n} lines {i}-{j}: M{m} ({ax}) {net} net steps, reported {rep:.1f}")
            ys = [m for m, (ax, _) in motors.items() if ax == "y"]
            twins = len(ys) == 2 and abs(motors[ys[0]][1] - motors[ys[1]][1]) < 1e-6   # same steps/mm (a geometry can differ)
            if settled_in_capture and twins and r["motors"][ys[0]]["steps"] != r["motors"][ys[1]]["steps"]:
                a, b = (_rises(e[f"M{m} step"]) for m in ys)
                if n == 0 and _one_startup_pulse(a, b):
                    # The loader preserves its original first-segment phase rule: with opposite initial
                    # DIR levels one accumulator starts at 0 and the other at -dda_ticks_X_substeps.
                    # After that first tick their accumulators and all subsequent pulses agree.
                    r["twin_startup_pulse"] = {"motors": ys, "extra": ys[0] if len(a) > len(b) else ys[1],
                                               "paired_pulses": min(len(a), len(b)), "max_pair_skew_s": 20e-6}
                else:
                    bad.append(f"window {n}: Y motors stepped {r['motors'][ys[0]]['steps']} and {r['motors'][ys[1]]['steps']}")
            r["refused"] = [(status, text[:60]) for _, status, text in getattr(res, "errors", [])][:10]
            # refused lines are recorded, not failed: files with codes TinyG does not support are expected to have
            # them (as in test_soak_file); a stall raises, and alarms, panics and mismatches are checked here
            if r["refused"]:                              # a refused line may be the spindle command itself (Estlcam's
                results.append(r)                         # unfilled "M03 S<s>"): the file's state no longer applies
                print("WINDOW", json.dumps(r, default=str)[:1500])
                continue
            want_mode = end_state["spindle"]
            want_duty = round(_duty(pwm, want_mode, end_state["s"]), 4)
            r["spindle_want"] = [want_mode, end_state["s"], want_duty, end_state["cool"]]
            if settled_in_capture and pwm_name:
                tail = [row["duty"] for row in cap.pwm(pwm_name)][-3:-1]      # at rest: the capture's last windows
                r["pwm_tail"] = tail
                if tail and any(abs(d - want_duty) > 0.006 for d in tail) and stat != tg.dialect.stat("ALARM"):
                    bad.append(f"window {n} lines {i}-{j}: PWM duty {tail} at rest, file state {r['spindle_want']}")
            if outs is not None:
                r["outputs"] = outs
                want_en = 0 if want_mode == "off" else 1
                if stat != tg.dialect.stat("ALARM") and (outs["spindle_enable"] != want_en or outs["coolant"] != end_state["cool"]
                                                         or (want_en and outs["spindle_dir"] != (1 if want_mode == "ccw" else 0))):
                    bad.append(f"window {n} lines {i}-{j}: outputs {outs}, file state {r['spindle_want']}")
            if stat in _dead(tg):
                bad.append(f"window {n} lines {i}-{j}: stat {stat} {exc[-1:] if exc else ''}")
            elif stat == tg.dialect.stat("ALARM"):
                bad.append(f"window {n} lines {i}-{j}: ALARM {exc[-1:] if exc else ''}")
                tg.clear_alarm()
            results.append(r)
            print("WINDOW", json.dumps(r, default=str)[:1500])
            if bad:
                break
    finally:
        if tg.stat() in _dead(tg):
            tg.reset()
        if tg.stat() not in tg.dialect.idle_stats:
            tg.send_raw(b"!%")
            time.sleep(0.5)
            _settle(tg, 30)
        tg.send_gcode("M5 M9 G92.1 G90 G21 G94 G64", allow="cleanup")
        tg.send_gcode(f"G53 G0 X{start['x']:.4f} Y{start['y']:.4f} Z{start['z']:.4f} A{start['a']:.4f}",
                      allow="return to the start")
        _settle(tg, 300)
    record("file_mso", {"file": FILE, "seed": SEED, "mode": MODE, "windows": results,
                        "settings": os.environ.get("MSO_SETTINGS", "")})
    assert not bad, f"{os.path.basename(FILE)} seed {SEED}: {bad}"
