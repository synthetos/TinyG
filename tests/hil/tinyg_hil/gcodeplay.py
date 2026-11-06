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
    # The compiled string stops at N370 and ends with N2519 M30"; N371-N2517 sit in a comment after it (they
    # did not fit in flash). Streaming has no such limit, so the whole original toolpath is sent, without the C
    # string terminator and without the program end: M30 cancels the program's G92 offset (NIST 3.6.1), which would
    # move the reported end away from the toolpath's last point.
    lines = []
    for m in re.findall(r"^(N\d+[^\r\n]*)", src, re.M):
        lines.append(re.sub(r'(\\n)?\\?";.*$|\\n\\.*$', "", m).strip())
    return [ln for ln in lines if not re.search(r"\bM(0?2|30)\b", ln)]


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
        cands = []
        for (ax, ay), (bx, by), s0, L in segs:
            if s0 + L < last - 0.5:
                continue
            dx, dy = bx - ax, by - ay
            t = 0.0 if L == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (L * L)))
            cands.append((math.hypot(px - (ax + t * dx), py - (ay + t * dy)), s0 + t * L))
        if not cands:
            continue
        near = min(d for d, _ in cands)
        # where the path comes back over itself, the earliest equally close place is the one reached
        best = min(s for d, s in cands if d <= near + 0.05)
        proj.append(best)
        last = max(last, best)
    proj.sort()
    gaps = [b - a for a, b in zip(proj, proj[1:])]
    return {"path_mm": round(total, 2), "samples": len(samples), "first_mm": round(proj[0], 2),
            "last_mm": round(proj[-1], 2), "max_gap_mm": round(max(gaps + [proj[0], total - proj[-1]]), 2)}


def path_check(status, pts, sr_interval, tol, min_samples):
    """Judge status reports [(t, sr dict)] against the commanded path pts.

    Returns the samples, the worst-first distances [(distance, t, sr)], the coverage, the gap limit
    (three report intervals at the peak reported speed, at least 2 mm) and a list of failures. Too few
    reports, a report off the path, or a stretch of path with no report all fail; no reports at all is
    a failure, never a pass by default."""
    with_pos = [(t, st) for t, st in status if "posx" in st and "posy" in st]
    samples = [(st["posx"], st["posy"]) for _, st in with_pos]
    scored = sorted(((distance_to_path((st["posx"], st["posy"]), pts), t, st) for t, st in with_pos),
                    key=lambda r: -r[0])
    cover = coverage(samples, pts)
    top_mm_s = max((st.get("vel", 0) for _, st in status), default=0) / 60.0
    gap_limit = max(2.0, 3 * sr_interval * top_mm_s)
    failures = []
    if len(samples) < min_samples:
        failures.append(f"only {len(samples)} status reports with a position (need {min_samples}): "
                        "no evidence the path ran")
    if scored and scored[0][0] > tol:
        failures.append(f"a status report was {scored[0][0]:.3f} mm off the commanded path")
    if "max_gap_mm" not in cover or cover["max_gap_mm"] > gap_limit:
        failures.append(f"part of the path has no status report on it ({cover}, limit {gap_limit:.1f} mm): "
                        "it did not run")
    return {"samples": samples, "scored": scored, "worst": scored[0][0] if scored else None,
            "coverage": cover, "gap_limit": gap_limit, "failures": failures}


# Throughput benchmark programs (from the perf branch): fixed content, so builds compare on identical lines.

def _goose_outline(spacing, scale):
    """One goose silhouette (a few Fourier terms), scaled about the centre and resampled at an even
    `spacing` along its length."""
    def curve(t):
        x = 10 + scale * sum(a * math.cos(k * t) + b * math.sin(k * t) for a, b, k in ((6.0, 0.0, 1), (1.4, 0.6, 2), (0.0, -0.8, 3)))
        y = 10 + scale * sum(b * math.sin(k * t) + a * math.cos(k * t) for a, b, k in ((0.0, 5.5, 1), (0.9, 0.0, 2), (-0.6, 0.4, 3)))
        return min(max(x, 0.0), SIZE), min(max(y, 0.0), SIZE)
    fine = [curve(2 * math.pi * i / 40000) for i in range(40001)]
    pts, run = [fine[0]], 0.0
    for a, b in zip(fine, fine[1:]):
        run += math.hypot(b[0] - a[0], b[1] - a[1])
        if run >= spacing:
            pts.append(b)
            run = 0.0
    pts.append(fine[-1])
    return pts


def _outline_lines(pts, feed):
    return (["G0 Z1", f"G0 X{pts[0][0]:.4f} Y{pts[0][1]:.4f}", "G1 Z0 F300", f"G1 F{feed}"]
            + [f"G1 X{x:.4f} Y{y:.4f}" for x, y in pts[1:]])


def perfbench(feed=1800):
    """The throughput benchmark, about 20 s on edge-next at jerk 20: three goose outlines (each with its
    cusps, where the planner queue fills) in sections that load the firmware differently:
      A  0.08 mm segments, full size: steady cruise limited by lines per second
      B  0.04 mm segments, 85%: the dense case (speed saws at jerk 200 with a full queue)
      C  0.08 mm segments, 70%: tighter curves
    Ends where it started. Fixed content: comparisons between builds use exactly the same lines."""
    out = ["(perfbench: goose sections A 0.08 mm, B 0.04 mm, C 0.08 mm)", "G21 G90 G17"]
    for name, spacing, scale in (("A", 0.08, 1.0), ("B", 0.04, 0.85), ("C", 0.08, 0.7)):
        out += [f"(section {name})"] + _outline_lines(_goose_outline(spacing, scale), feed)
    out += ["G0 Z1", "G0 X0 Y0", "G0 Z0"]
    return out


def perfbench_ext():
    """Behaviour coverage beside perfbench, for checking each throughput step on what perfbench does not touch.
    It is not a speed yardstick. It returns to X0 Y0 Z0 A0, so net steps must be zero. Sections:
      E1  arcs G2/G3, half and full circles: segments the firmware generates itself
      E2  helix: X, Y and Z together (three motors per tick)
      E3  short X/Y/Z segments (3D finishing): three motors with dense lines
      E4  A axis with X/Y/Z: all four motors per tick (motor 4 is A on the bench)
      E5  spindle on with speed changes mid-job, coolant (PWM and outputs; nothing fitted on the bench)
      E6  dwell, very slow feed, long rapids"""
    out = ["(perfbench-ext: behaviour coverage)", "G21 G90 G17", "M3 S1000", "M8",
           "G0 Z1", "G0 X5 Y10", "G1 Z0 F300",
           "(section E1)", "G2 X15 Y10 I5 J0 F1200", "G3 X5 Y10 I-5 J0", "G2 X5 Y10 I3 J0", "S8000",
           "G3 X5 Y10 I3 J0",
           "(section E2)", "G2 X5 Y10 Z1.5 I2 J0 F900", "G2 X5 Y10 Z0 I2 J0",
           "(section E3)"]
    for k in range(1, 121):
        out.append(f"G1 X{5 + 0.08 * k:.4f} Y{10 + 0.6 * math.sin(k * 0.21):.4f} Z{0.5 + 0.4 * math.sin(k * 0.3):.4f} F800")
    out += ["G1 Z0 F300",
            "(section E4)", "S3000", "G1 X10 Y12 Z0.5 A90 F600", "G1 X12 Y8 Z0 A-45", "G1 X14.6 Y10 Z0.3 A30",
            "G1 X14.6 Y10 Z0 A0",
            "(section E5)", "S12000", "G1 X16 Y11 F1200", "M9", "S500", "G1 X14.6 Y10",
            "(section E6)", "G4 P0.3", "G1 X15.1 Y10 F30", "G0 Z1", "G0 X18 Y18", "G0 X2 Y2", "M5",
            "G0 X0 Y0", "G0 Z0"]
    return out
