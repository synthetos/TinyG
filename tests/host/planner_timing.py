"""Planner timing replay: the real planner, exec and segment prep on the host, under a machine profile.

    python3 -I planner_timing.py replay FILE.nc [FILE.nc ...] --settings PROFILE.json --out RESULT.json [--source TREE]

For each file: the planned duration (segment times plus dwells) and every planned block with its source line (the
N word if the line has one, otherwise the file line), kind (feed, rapid, arc-seg, dwell), length, entry/cruise/exit
speeds (mm/min, as the firmware plans them), time split into head/body/tail (accelerate, cruise, decelerate),
exit_limit (see _exit_limit) and causes (the feed-limit advisor's tags for the block, feedlimit.h). A dwell entry
has only n, kind and time_s. The top-level planner object holds the constants that shape the result and a hash of
the planner sources. Any line the firmware rejects fails the replay. Deterministic: the same firmware, profile
and file give the same numbers. This is the host layer of
the planner timing fixtures (tests/hil/fixtures/planner_timing, schema 1); the bench layer measures the same files
from step pulses. Bench-only directives (;HOST-PAUSE, ;HOLD-RESUME) are G-code comments here and change nothing.
A profile sets per-axis vm, fr, jm, jd, the junction acceleration and chordal tolerance, and the motor map with
steps per unit (see tests/hil/fixtures/planner_timing/profiles).
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import re
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import trajectory_replay as tr  # noqa: E402

AXES = "xyzabc"
ACCEPTED = {0, 3}    # STAT_OK, STAT_NOOP; anything else changed the program and fails the replay


def _profile_patch(profile):
    """C replacing the replay driver's fixed axis and motor setup with the profile's values."""
    def patch(driver):
        axis_line = re.search(r"\n\s*for\(int a=0;a<AXES;a\+\+\)\{cm\.a\[a\]\.axis_mode=[^\n]*\n", driver)
        motor_line = re.search(r"\n\s*for\(int m=0;m<MOTORS;m\+\+\)\{st_cfg\.mot\[m\]\.motor_map=[^\n]*\n", driver)
        assert axis_line and motor_line, "replay driver setup lines not found"
        # unlisted axes are disabled but keep finite rates and jerk: the planner divides every axis length (zero
        # for them) by its rate, and 0/0 would make the move time NaN
        c = ["for(int a=0;a<AXES;a++){cm.a[a].axis_mode=AXIS_DISABLED;cm.a[a].velocity_max=cm.a[a].feedrate_max=a<3?16000:3600;"
             "cm_set_axis_jerk(a,500);cm.a[a].junction_dev=.05f;cm.a[a].radius=1;cm.a[a].travel_min=-1;cm.a[a].travel_max=-1;}"]
        for name, ax in profile["axes"].items():
            i = AXES.index(name)
            c.append(f"cm.a[{i}].axis_mode=AXIS_STANDARD;cm.a[{i}].velocity_max={float(ax['vm'])}f;cm.a[{i}].feedrate_max={float(ax['fr'])}f;"
                     f"cm_set_axis_jerk({i},{float(ax['jm'])}f);cm.a[{i}].junction_dev={float(ax['jd'])}f;")
        m = []
        for k, mot in enumerate(profile["motors"]):
            spu = float(mot["steps_per_unit"])
            m.append(f"st_cfg.mot[{k}].motor_map={AXES.index(mot['axis'])};st_cfg.mot[{k}].steps_per_unit={spu}f;"
                     f"st_cfg.mot[{k}].units_per_step={1.0 / spu}f;")
        driver = driver.replace(axis_line.group(0), "\n " + "".join(c) + "\n", 1)
        driver = driver.replace(motor_line.group(0), "\n " + "".join(m) + "\n", 1)
        driver = re.sub(r"cm\.chordal_tolerance=[^;]*;", f"cm.chordal_tolerance={float(profile['ct'])}f;", driver, count=1)
        driver = re.sub(r"cm\.junction_acceleration=[^;]*;", f"cm.junction_acceleration={float(profile['ja'])}f;", driver, count=1)
        return driver
    return patch


def build(root, out, profile):
    out.mkdir(parents=True, exist_ok=True)
    return tr.build(Path(root).resolve(), out, extra_patch=_profile_patch(profile))


def numbered(gcode):
    """Give every line without an N word its file line number, so blocks map back to file lines."""
    lines = []
    for i, line in enumerate(gcode.splitlines(), start=1):
        s = line.strip()
        if s and not s.startswith("%") and not re.match(r"[Nn]\d", s):
            line = f"N{i} {line}"
        lines.append(line)
    return "\n".join(lines) + "\n"


KIND = {0: "rapid", 1: "feed", 2: "arc-seg", 3: "arc-seg"}   # the block's motion mode (G0, G1, G2, G3)
CAUSES = ((0x01, "axis_cap"), (0x02, "junction"), (0x04, "exact_stop"), (0x08, "short"), (0x10, "min_segment"),
          (0x20, "command"))                                     # the feed-limit advisor's tags (feedlimit.h)
SECTIONS = ("head_s", "body_s", "tail_s")                         # SECTION_HEAD/BODY/TAIL: accel, cruise, decel


def _exit_limit(b, nx):
    """Why the planned exit speed is what it is (speeds compared within 0.1% or 0.5 mm/min). nx is the next block
    as the planner saw it when this block started: (state, type, entry_vmax, braking_velocity), state 0 = none.
    end - the block stops at a program stop or end (M0/M1/M2/M30), or nothing was queued after it (the queue ran dry);
    stop - the next block is not a move (a dwell, or a command that waits for motion to stop);
    junction - the exit is the corner limit with the next move (zero at an exact stop, G61/G61.1);
    cruise - the block cruises and exits at that speed (why the cruise is below F is in causes);
    accel - the block accelerates the whole way and ends below every limit: too short to reach speed;
    lookahead - lower than all of those: braking for later blocks."""
    x, (state, kind, nx_entry, _nx_brake) = b["exit"], nx
    tol = lambda v: max(0.5, abs(v) * 0.001)
    if state == 0:
        return "end" if x < tol(0) else "lookahead"
    if kind != 1:                                      # not BLOCK_TYPE_ALINE
        if x < tol(0):
            return "end" if kind in (6, 7) else "stop"  # BLOCK_TYPE_STOP / _END (M0, M1, M2, M30)
    elif abs(x - nx_entry) < tol(nx_entry):
        return "junction"
    elif x < tol(0):
        return "end"                                   # stopped though the next move allowed more: planned before it came
    if b["body_s"] > 0 and abs(x - b["cruise"]) < tol(b["cruise"]):
        return "cruise"
    if x > b["entry"] + tol(x) and b["body_s"] == 0 and b["tail_s"] == 0:
        return "accel"
    return "lookahead"


def replay(exe, gcode, timeout=900):
    p = subprocess.run([str(exe), "28"], input=numbered(gcode).encode(), capture_output=True, timeout=timeout)
    if p.returncode:
        raise RuntimeError(f"replay exit {p.returncode}: {p.stderr.decode()[-2000:]}")
    blocks, cur, total_min, dwell_s, rejected = [], None, 0.0, 0.0, []
    seg = None
    for line in p.stdout.decode().splitlines():
        f = line.split()
        if not f:
            continue
        if f[0] == "SEG":
            seg = (float(f[1]), int(f[8]), int(f[9]))  # dt (min), line, section
            total_min += seg[0]
        elif f[0] == "BLK" and seg:
            key = (seg[1], f[1], f[2], f[3], f[4])    # line + the planned block: a new block when either changes
            if cur is None or cur.get("key") != key:
                cur = {"key": key, "n": seg[1], "kind": KIND.get(int(f[8]), "other"), "length_mm": float(f[4]),
                       "entry": float(f[1]), "cruise": float(f[2]), "exit": float(f[3]), "time_s": 0.0,
                       "head_s": 0.0, "body_s": 0.0, "tail_s": 0.0,
                       "nx": (int(f[9]), int(f[10]), float(f[11]), float(f[12])),
                       "causes": [name for bit, name in CAUSES if int(f[13]) & bit] if len(f) > 13 else []}
                blocks.append(cur)
            cur["time_s"] += seg[0] * 60
            cur[SECTIONS[seg[2]]] += seg[0] * 60
        elif f[0] == "DWELL":
            t = float(f[1]) / 1e6
            dwell_s += t
            cur = {"n": int(f[2]) if len(f) > 2 else 0, "kind": "dwell", "time_s": round(t, 6)}
            blocks.append(cur)
        elif f[0] == "LINE" and int(f[2]) not in ACCEPTED:
            rejected.append({"n": int(f[1]), "status": int(f[2])})
        elif f[0] in ("ARC_STATUS", "STATE", "EXC", "EXEC", "STUCK", "BAD") or (f[0] in ("JSON", "TEXT") and int(f[2]) not in ACCEPTED):
            rejected.append({"n": int(f[1]) if len(f) > 1 and f[1].isdigit() else 0, "status": line})
    if 'EXCEPTION' in p.stderr.decode() or not math.isfinite(total_min) or total_min < 0:
        rejected.append({'status': p.stderr.decode()[-1000:] or 'invalid duration'})
    if rejected:                                        # a rejected line changes the program: never time it
        raise RuntimeError(f"firmware rejected lines: {rejected[:10]}")
    for b in blocks:
        if b["kind"] == "dwell":
            continue
        del b["key"]
        b["exit_limit"] = _exit_limit(b, b.pop("nx"))
        for k in ("time_s",) + SECTIONS:
            b[k] = round(b[k], 6)
        for k in ("length_mm", "entry", "cruise", "exit"):
            b[k] = round(b[k], 4)
    return {"duration_s": round(total_min * 60 + dwell_s, 6), "motion_s": round(total_min * 60, 6),
            "dwell_s": round(dwell_s, 6), "blocks": blocks}


PLANNER_CONSTANTS = ("PLANNER_BUFFER_POOL_SIZE", "PLANNER_BUFFER_HEADROOM", "NOM_SEGMENT_USEC", "MIN_SEGMENT_USEC",
                     "PLANNER_STARTUP_MS", "JERK_MULTIPLIER")
PLANNER_SOURCES = ("planner.h", "planner.c", "plan_line.c", "plan_zoid.c", "plan_exec.c", "plan_arc.c")


def planner_info(root):
    """The constants that shape the result, and a hash of the planner sources: a fixture set made from another
    planner is detectable even when no constant changed."""
    fw = Path(root) / "firmware" / "tinyg"
    text = "".join((fw / n).read_text(errors="replace") for n in ("planner.h", "planner.c") if (fw / n).exists())
    info = {}
    for name in PLANNER_CONSTANTS:
        m = re.search(rf"#define\s+{name}\s+\(?\s*(?:\(float\))?\s*([0-9.]+)", text)
        if m:
            v = float(m.group(1))
            info[name.lower()] = int(v) if v.is_integer() else v
    h = hashlib.sha256()
    for n in PLANNER_SOURCES:
        if (fw / n).exists():
            h.update(n.encode() + b"\0" + (fw / n).read_bytes())
    info["sources_sha256"] = h.hexdigest()
    return info


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("replay")
    r.add_argument("files", type=Path, nargs="+")
    r.add_argument("--settings", type=Path, required=True)
    r.add_argument("--out", type=Path, required=True)
    r.add_argument("--source", type=Path, default=Path(__file__).resolve().parents[2])
    r.add_argument("--work", type=Path, default=None, help="build directory (default: next to --out)")
    a = ap.parse_args()
    profile = json.loads(a.settings.read_text())
    work = a.work or a.out.with_suffix(".build")
    exe = build(a.source, work, profile)
    commit = subprocess.run(["git", "-C", str(a.source), "rev-parse", "--short=8", "HEAD"], capture_output=True, text=True).stdout.strip()
    result = {"schema": 1, "firmware": {"commit": commit, "source": "host replay"}, "planner": planner_info(a.source),
              "profile": profile, "files": []}
    failed = 0
    result['host_limits'] = ['No UART, parsing throughput or real-time latency measurement',
                             'HOST-PAUSE and HOLD-RESUME comments are not executed',
                             '449 command argument arrays padded to AXES for sanitizer-safe unused slots',
                             'No feed-limit cause reporting on firmware without advisor']
    for f in a.files:
        text = f.read_text(errors="replace")
        row = {"name": f.name, "sha256": hashlib.sha256(text.encode()).hexdigest()}
        try:
            row['host'] = replay(exe, text)
            print(f"{f.name}: {row['host']['duration_s']:.3f} s, {len(row['host']['blocks'])} blocks", flush=True)
        except (RuntimeError, subprocess.TimeoutExpired) as error:
            failed += 1
            row['error'] = str(error)
            print(f"{f.name}: ERROR {str(error)[:200]}", flush=True)
        result['files'].append(row)
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(json.dumps(result, indent=1, allow_nan=False))
    return 1 if failed else 0



if __name__ == "__main__":
    sys.exit(main())
