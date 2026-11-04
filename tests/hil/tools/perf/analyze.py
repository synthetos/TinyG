"""Motion metrics from Saleae step/dir captures (bench 2: M1=X ch6/7, M2=Y ch2/3, M3=Z ch4/5).

usage: analyze.py <steps_per_mm X> <Y> <Z> <feed mm/min> <edges.json>...
Per capture: motion time (first to last step), pauses (no step on any motor for > PAUSE s),
time below half the programmed feed (10 ms bins, 3D path speed), mean path speed, path length.
"""
import json, math, sys

PAUSE = 0.030
BIN = 0.010
MOTORS = {"X": ("6", "7"), "Y": ("2", "3"), "Z": ("4", "5")}


def motor_steps(d, step, dirn):
    tr = d["transitions"]; init = d["initial"]
    level = init[dirn]; dir_tr = tr.get(dirn, []); k = 0; out = []
    prev = init[step]
    for t, v in tr.get(step, []):
        if v == 1 and prev == 0:
            while k < len(dir_tr) and dir_tr[k][0] <= t:
                level = dir_tr[k][1]; k += 1
            out.append((t, 1 if level else -1))
        prev = v
    return out


def cut_window(steps):
    """(t_start, t_end) of the cutting: first X/Y step after the first plunge (Z down) to the last X/Y
    step before the final retract (Z up). Leaves out the entry and exit rapids, whose length depends
    on where the previous run left the machine. 'Up' is the sign of each file's first Z move (a lift)."""
    z, xy = steps.get("Z", []), sorted(t for ax in ("X", "Y") for t, _ in steps.get(ax, []))
    if not z or not xy:
        return None
    up = z[0][1]
    t_plunge = next((t for t, s in z if s != up), None)
    t_retract = next((t for t, s in reversed(z) if s == up), None)
    if t_plunge is None or t_retract is None:
        return None
    t0 = next((t for t in xy if t > t_plunge), None)
    t1 = next((t for t in reversed(xy) if t < t_retract), None)
    return (t0, t1) if t0 is not None and t1 is not None and t1 > t0 else None


def load_steps(path, window=True):
    d = json.load(open(path))
    steps = {ax: motor_steps(d, *MOTORS[ax]) for ax in MOTORS
             if MOTORS[ax][0] in d["initial"] and MOTORS[ax][1] in d["initial"]}
    w = cut_window(steps) if window else None
    if w:
        steps = {ax: [(t, s) for t, s in steps[ax] if w[0] <= t <= w[1]] for ax in steps}
    return steps


def analyze(path, spmm, feed, window=True):
    steps = load_steps(path, window)
    times = sorted(t for ax in steps for t, _ in steps[ax])
    if len(times) < 2:
        return {"file": path, "steps": len(times)}
    t0, t1 = times[0], times[-1]
    gaps = [b - a for a, b in zip(times, times[1:]) if b - a > PAUSE]
    nb = int((t1 - t0) / BIN) + 1
    net = {ax: [0] * nb for ax in steps}
    for ax in steps:
        for t, s in steps[ax]:
            net[ax][min(nb - 1, int((t - t0) / BIN))] += s
    speeds = [math.sqrt(sum((net[ax][i] / spmm[ax]) ** 2 for ax in net)) / BIN for i in range(nb)]
    length = sum(s * BIN for s in speeds)
    slow = sum(1 for s in speeds if s < 0.5 * feed / 60.0) * BIN
    return {"file": path.split("/")[-1], "motion_s": round(t1 - t0, 3), "pauses": len(gaps),
            "pause_s": round(sum(gaps), 3), "longest_pause_s": round(max(gaps), 3) if gaps else 0.0,
            "slow_s": round(slow, 3), "slow_frac": round(slow / (t1 - t0), 4),
            "mean_mm_min": round(length / (t1 - t0) * 60, 1), "path_mm": round(length, 2),
            "steps": {ax: len(steps[ax]) for ax in steps}}


if __name__ == "__main__":
    sx, sy, sz, feed = map(float, sys.argv[1:5])
    for p in sys.argv[5:]:
        print(json.dumps(analyze(p, {"X": sx, "Y": sy, "Z": sz}, feed)))
