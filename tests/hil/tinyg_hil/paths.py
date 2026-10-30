"""Expected vs actual toolpaths from a test record (see record.py).

actual: positions the board reported (status reports carry work coordinates posx/posy/posz;
        GETs of pos*/mpo* add samples too). Status reports only carry changed fields, so the
        state is accumulated. Machine-coordinate GETs are converted with the start offsets.
expected: gsim run over the G-code lines the test sent, from the recorded start position.
"""
import json
import math

from . import gsim
from .protocol import loads

AXES3 = "xyz"


def _is_gcode(line):
    line = line.strip()
    return bool(line) and line[0] not in "{$?!~%" and not line.startswith(("\x18", "\x05"))


def gcode_lines(record):
    lines = []
    for s in record.get("sent", []):
        for line in s["data"].splitlines():
            if _is_gcode(line):
                lines.append(line)
    return lines


def start_offsets(record):
    sp = record.get("start_position") or {}
    return {a: (sp.get(f"mpo{a}") or 0.0) - (sp.get(f"pos{a}") or 0.0) for a in AXES3} if sp else None


def actual(record):
    """Work-coordinate samples [(t, {x,y,z})] reported during the test."""
    sp = record.get("start_position") or {}
    state = {a: sp.get(f"pos{a}") for a in AXES3}
    offs = start_offsets(record) or {a: 0.0 for a in AXES3}
    samples = []
    if all(v is not None for v in state.values()):
        samples.append((0.0, dict(state)))
    for ev in record.get("events", []):
        if not ev["raw"].startswith("{"):
            continue
        try:
            data = loads(ev["raw"])
        except ValueError:
            continue
        body = data.get("sr") if isinstance(data.get("sr"), dict) else data.get("r") if isinstance(data.get("r"), dict) else None
        if not body:
            continue
        changed = False
        for a in AXES3:
            if isinstance(body.get(f"pos{a}"), (int, float)):
                state[a] = body[f"pos{a}"]
                changed = True
            elif isinstance(body.get(f"mpo{a}"), (int, float)):
                state[a] = body[f"mpo{a}"] - offs[a]
                changed = True
        if changed and all(v is not None for v in state.values()):
            samples.append((ev["t"], dict(state)))
    return samples


def expected(record):
    """Expected work-coordinate polyline [{x,y,z}] or None when the test has no motion to model."""
    lines = gcode_lines(record)
    sp = record.get("start_position")
    if not lines or not sp:
        return None
    offs = start_offsets(record)
    machine = gsim.Machine()
    machine.position.update({a.upper(): sp.get(f"mpo{a}") or 0.0 for a in AXES3})
    for a in AXES3:
        machine.coord_offsets[1][a.upper()] = offs[a]
    machine.run(lines)
    pts = [{a: p[a.upper()] for a in AXES3} for p in gsim.path(machine)]
    return pts or None


def _seg_dist(p, a, b):
    ab = [b[k] - a[k] for k in AXES3]
    ap = [p[k] - a[k] for k in AXES3]
    denom = sum(c * c for c in ab)
    t = 0.0 if denom == 0 else max(0.0, min(1.0, sum(x * y for x, y in zip(ap, ab)) / denom))
    return math.sqrt(sum((ap[i] - t * ab[i]) ** 2 for i in range(3)))


def compare(exp, act):
    """Largest distance of any reported sample from the expected path, and end-point error."""
    if not exp or not act:
        return None
    pts = [s for _, s in act]
    if len(exp) == 1:
        exp = exp * 2
    worst = max(min(_seg_dist(p, exp[i], exp[i + 1]) for i in range(len(exp) - 1)) for p in pts)
    end = math.sqrt(sum((pts[-1][k] - exp[-1][k]) ** 2 for k in AXES3))
    return {"max_deviation": round(worst, 4), "end_error": round(end, 4), "samples": len(pts)}


def analyze(record):
    exp, act = expected(record), actual(record)
    return {"expected": exp, "actual": act, "metrics": compare(exp, act)}


def load(path):
    return json.loads(open(path).read())
