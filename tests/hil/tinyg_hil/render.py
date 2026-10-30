"""SVG snapshot of expected vs actual toolpaths: XY (top) and XZ (front) views side by side."""
from html import escape

VIEWS = (("XY", "x", "y"), ("XZ", "x", "z"))
W, H, PAD = 340, 300, 34


def _bounds(points, u, v):
    us = [p[u] for p in points]
    vs = [p[v] for p in points]
    lo_u, hi_u, lo_v, hi_v = min(us), max(us), min(vs), max(vs)
    span = max(hi_u - lo_u, hi_v - lo_v, 1.0) * 1.15        # equal scale on both axes, >= 1 mm
    cu, cv = (lo_u + hi_u) / 2, (lo_v + hi_v) / 2
    return cu - span / 2, cv - span / 2, span


def _panel(x0, title, u, v, exp, act):
    pts = (exp or []) + [s for _, s in act]
    out = [f'<g transform="translate({x0},0)">',
           f'<rect x="0" y="0" width="{W}" height="{H}" class="frame"/>',
           f'<text x="10" y="18" class="title">{title}</text>']
    if not pts:
        out.append(f'<text x="{W / 2}" y="{H / 2}" class="empty" text-anchor="middle">no positions</text></g>')
        return "".join(out)
    bu, bv, span = _bounds(pts, u, v)
    scale = (W - 2 * PAD) / span

    def xy(p):
        return f"{PAD + (p[u] - bu) * scale:.1f},{H - PAD - (p[v] - bv) * scale:.1f}"

    # grid line through work zero when visible, and a 1 mm scale bar
    if bu <= 0 <= bu + span:
        gx = PAD + (0 - bu) * scale
        out.append(f'<line x1="{gx:.1f}" y1="{PAD}" x2="{gx:.1f}" y2="{H - PAD}" class="axis"/>')
    if bv <= 0 <= bv + span:
        gy = H - PAD - (0 - bv) * scale
        out.append(f'<line x1="{PAD}" y1="{gy:.1f}" x2="{W - PAD}" y2="{gy:.1f}" class="axis"/>')
    out.append(f'<line x1="{PAD}" y1="{H - 12}" x2="{PAD + scale:.1f}" y2="{H - 12}" class="bar"/>'
               f'<text x="{PAD + scale + 4:.1f}" y="{H - 8}" class="label">1 mm</text>')
    out.append(f'<text x="{W - 10}" y="{H - 8}" class="label" text-anchor="end">{u.upper()}→ {v.upper()}↑</text>')
    if exp:
        out.append(f'<polyline points="{" ".join(xy(p) for p in exp)}" class="expected"/>')
        out.append(f'<circle cx="{xy(exp[-1]).split(",")[0]}" cy="{xy(exp[-1]).split(",")[1]}" r="4" class="exp-end"/>')
    if act:
        out.append(f'<polyline points="{" ".join(xy(s) for _, s in act)}" class="actual"/>')
        for _, s in act:
            cx, cy = xy(s).split(",")
            out.append(f'<circle cx="{cx}" cy="{cy}" r="1.8" class="sample"/>')
    out.append("</g>")
    return "".join(out)


def svg(analysis, caption=""):
    exp, act, m = analysis["expected"], analysis["actual"], analysis["metrics"]
    width = 2 * W + 30
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {H + 46}" width="{width}" height="{H + 46}">',
             "<style>",
             ".frame{fill:var(--panel,#fafafa);stroke:var(--line,#ccc)}.title{font:600 13px sans-serif;fill:var(--ink,#222)}"
             ".axis{stroke:var(--line,#ccc);stroke-dasharray:2 3}.bar{stroke:var(--ink,#222);stroke-width:2}"
             ".label,.empty{font:11px sans-serif;fill:var(--muted,#666)}"
             ".expected{fill:none;stroke:#7a8699;stroke-width:2;stroke-dasharray:6 4}"
             ".exp-end{fill:none;stroke:#7a8699;stroke-width:2}"
             ".actual{fill:none;stroke:#d9480f;stroke-width:1.6}.sample{fill:#d9480f}"
             ".legend{font:12px sans-serif;fill:var(--ink,#222)}",
             "</style>"]
    for i, (title, u, v) in enumerate(VIEWS):
        parts.append(_panel(i * (W + 30), title, u, v, exp, act))
    legend = "expected (dashed)  ·  board-reported (solid)"
    if m:
        legend += f"  ·  max deviation {m['max_deviation']:.3f} mm  ·  end error {m['end_error']:.3f} mm  ·  {m['samples']} samples"
    if caption:
        legend = f"{caption}  —  {legend}"
    parts.append(f'<text x="4" y="{H + 30}" class="legend">{escape(legend)}</text></svg>')
    return "".join(parts)
