"""Demo G-code programs for streaming tests, and the path they command.

Every program is written in absolute work coordinates inside [0, SIZE] x [0, SIZE] (Z 0..1); the test
sets the work origin at the start position with G92 before streaming and removes it afterwards.
path() interprets the same subset the generators emit (G0/G1/G2/G3 in XY with I/J, G90/G91, G17/G21,
F, G4, N words, comments) into a dense polyline in work coordinates.
"""
import math
import re

SIZE = 20.0                       # mm: every demo program stays in [0, SIZE] in X and Y, Z in [0, 2]


def squares(rings=25, feed=1500):
    """Concentric squares shrinking to the centre: straight G1 moves, many collinear junctions."""
    out = ["(squares)", "G21 G90 G17", f"G1 F{feed}"]
    c = SIZE / 2
    for k in range(rings):
        h = c * (1 - k / rings)
        for x, y in ((c - h, c - h), (c + h, c - h), (c + h, c + h), (c - h, c + h), (c - h, c - h)):
            out.append(f"G1 X{x:.3f} Y{y:.3f}")
    out.append("G0 X0 Y0")
    return out


def circles(count=12, feed=1200):
    """Full circles and half arcs with G2/G3 I/J, alternating direction, plus line numbers."""
    out = ["(circles)", "G21 G90 G17", f"G0 X{SIZE / 2:.3f} Y1"]
    n = 10
    for k in range(count):
        r = 1 + (SIZE / 2 - 2) * k / count
        out.append(f"N{n} G1 X{SIZE / 2:.3f} Y{SIZE / 2 - r:.3f} F{feed}"); n += 10
        g = "G2" if k % 2 else "G3"
        out.append(f"N{n} {g} X{SIZE / 2:.3f} Y{SIZE / 2 - r:.3f} I0 J{r:.3f}"); n += 10
    out.append(f"N{n} G0 X0 Y0")
    return out


def goose(spacing=0.08, feed=1800, passes=6):
    """A dense CAM-style outline (a goose silhouette from a few Fourier terms) resampled at an even
    `spacing` along its length, as chordal-tolerance CAM output looks; `passes` nested outlines scaled
    toward the centre give ~2500 G1 segments of 0.08 mm.
    (Uniform spacing matters: the raw parameterisation crawls through tight cusps in micro-steps far
    below the planner's 2.5 ms minimum move time, which the firmware merges into chords by design.)"""
    def curve(t, scale):
        x = 10 + scale * sum(a * math.cos(k * t) + b * math.sin(k * t) for a, b, k in ((6.0, 0.0, 1), (1.4, 0.6, 2), (0.0, -0.8, 3)))
        y = 10 + scale * sum(b * math.sin(k * t) + a * math.cos(k * t) for a, b, k in ((0.0, 5.5, 1), (0.9, 0.0, 2), (-0.6, 0.4, 3)))
        return min(max(x, 0.0), SIZE), min(max(y, 0.0), SIZE)
    out = ["(goose: dense short segments)", "G21 G90 G17"]
    for k in range(passes):
        scale = 1.0 - 0.5 * k / max(1, passes - 1)
        fine = [curve(2 * math.pi * i / 40000, scale) for i in range(40001)]
        pts, run = [fine[0]], 0.0
        for a, b in zip(fine, fine[1:]):
            run += math.hypot(b[0] - a[0], b[1] - a[1])
            if run >= spacing:
                pts.append(b)
                run = 0.0
        pts.append(fine[-1])
        out += ["G0 Z1", f"G0 X{pts[0][0]:.4f} Y{pts[0][1]:.4f}", "G1 Z0 F300", f"G1 F{feed}"]
        out += [f"G1 X{x:.4f} Y{y:.4f}" for x, y in pts[1:]]
    out += ["G0 Z1", "G0 X0 Y0", "G0 Z0"]
    return out


def mixed(feed=1000):
    """A job-shaped file: header comments, units/plane/distance modes, F changes, a dwell, relative
    moves, spindle on/off words (no spindle fitted on the bench) and blank lines."""
    out = ["%", "(mixed job)", "(T1 D=3 flat)", "G21 G17 G90", "", "M3 S1000", "G4 P0.2",
           f"G0 X2 Y2", f"G1 Z0 F{feed // 2}", f"G1 X18 F{feed}", "G1 Y18", "G91", "G1 X-4 Y-4", "G1 X-4",
           "G90", "G2 X6 Y10 I-4 J0", "G3 X2 Y2 I-2 J-4", "G1 X2 Y2", "M5", "G0 X0 Y0", "%"]
    return out


def braid():
    """TinyG's own braid program (firmware/tinyg/tests/test_051_braid.h) as a file of lines: a real
    CAM toolpath of ~2500 short G1 moves spanning about -31..52 mm in X and -51..53 mm in Y (needs that
    much free travel)."""
    from pathlib import Path
    src = (Path(__file__).resolve().parents[3] / "firmware/tinyg/tests/test_051_braid.h").read_text(errors="replace")
    # The compiled string stops at N383; N384-N2517 sit in a comment (they did not fit in flash).
    # Streaming has no such limit, so the whole original toolpath is sent.
    lines = []
    for m in re.findall(r"^(N\d+[^\r\n]*)", src, re.M):
        lines.append(re.sub(r"\\n\\.*$", "", m).strip())
    return lines


PROGRAMS = {"squares": squares, "circles": circles, "goose": goose, "mixed": mixed}
LARGE_PROGRAMS = {"braid": braid}     # need more travel than the 20 mm box: opt-in


def path(lines, arc_step=0.05):
    """Commanded XY polyline [(x, y), ...] for the subset above, starting at (0, 0) in work coordinates.
    A G92 redefines the position without motion: the polyline breaks there (a None entry)."""
    x = y = 0.0
    absolute = True
    motion = None
    pts = [(x, y)]
    for raw in lines:
        line = re.sub(r"\(.*?\)|;.*", "", raw).upper().replace(" ", "")
        if not line or line == "%":
            continue
        words = dict()
        gs = []
        for letter, value in re.findall(r"([A-Z])([-+]?\d*\.?\d+)", line):
            if letter == "G":
                gs.append(float(value))
            else:
                words[letter] = float(value)
        for g in gs:
            if g == 90: absolute = True
            elif g == 91: absolute = False
            elif g in (0, 1, 2, 3): motion = int(g)
        if 92 in gs:                                    # G92: redefine the current position, no motion
            x, y = words.get("X", x), words.get("Y", y)
            pts = [(x, y)] if len(pts) == 1 else pts + [None, (x, y)]
            continue
        if 4 in gs or not ({"X", "Y"} & words.keys()):
            continue
        nx = (words.get("X", x) if absolute else x + words.get("X", 0.0))
        ny = (words.get("Y", y) if absolute else y + words.get("Y", 0.0))
        if motion in (2, 3):
            cx, cy = x + words.get("I", 0.0), y + words.get("J", 0.0)
            r = math.hypot(x - cx, y - cy)
            a0, a1 = math.atan2(y - cy, x - cx), math.atan2(ny - cy, nx - cx)
            sweep = a1 - a0
            if motion == 2:
                if sweep >= 0: sweep -= 2 * math.pi
            else:
                if sweep <= 0: sweep += 2 * math.pi
            steps = max(2, int(abs(sweep) * r / arc_step))
            pts += [(cx + r * math.cos(a0 + sweep * i / steps), cy + r * math.sin(a0 + sweep * i / steps))
                    for i in range(1, steps + 1)]
        else:
            pts.append((nx, ny))
        x, y = nx, ny
    return pts


def distance_to_path(p, pts):
    """Smallest distance from point p to the polyline pts."""
    best = float("inf")
    px, py = p
    for a, b in zip(pts, pts[1:]):
        if a is None or b is None:
            continue
        (ax, ay), (bx, by) = a, b
        dx, dy = bx - ax, by - ay
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best


def coverage(samples, pts):
    """How completely status-report samples cover the commanded path.

    Samples must be in time order. Each is projected onto the polyline by arc length (pieces split by
    G92 are concatenated), never behind the furthest point already reached, so a path that crosses or
    retraces itself is not mistaken for a gap.
    Returns the path length, the arc-length positions reached at the start and end, and the longest
    stretch of path with no sample on it - a skipped or unexecuted part shows up as a large gap."""
    segs, total = [], 0.0
    for a, b in zip(pts, pts[1:]):
        if a is None or b is None:
            continue
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        segs.append((a, b, total, L))
        total += L
    if not segs or not samples:
        return {"path_mm": round(total, 2), "samples": len(samples)}
    proj, last = [], 0.0
    for px, py in samples:                       # time order: the machine only moves forward along the path
        best = None
        for (ax, ay), (bx, by), s0, L in segs:
            if s0 + L < last - 0.5:
                continue
            dx, dy = bx - ax, by - ay
            t = 0.0 if L == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (L * L)))
            d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
            if best is None or d < best[0]:
                best = (d, s0 + t * L)
        if best is None:
            continue
        proj.append(best[1])
        last = max(last, best[1])
    proj.sort()
    gaps = [b - a for a, b in zip(proj, proj[1:])]
    return {"path_mm": round(total, 2), "samples": len(samples), "first_mm": round(proj[0], 2),
            "last_mm": round(proj[-1], 2), "max_gap_mm": round(max(gaps + [proj[0], total - proj[-1]]), 2)}
