"""Wiring guide for the switchbox: board profiles -> terminal map, SVG drawings, PNG, Markdown.

    python3 tools/render_pinout.py              # docs/wiring.svg, docs/wiring-dark.svg, docs/wiring.md (+ PNGs)
    python3 tools/render_pinout.py --check      # exit 1 if the committed outputs are out of date
    python3 tools/render_pinout.py --carrier terminal-block-featherwing --feather huzzah-esp8266

This is the only drawing code. The firmware build (firmware/scripts/embed_assets.py) imports it to
embed the same board drawing in the web UI's Wiring view, where JavaScript only fills in the live
channel names and contact state; this script renders it with the firmware's default channel map
for the docs.

Profiles (boards/, see boards/README.md):
  carrier/<id>.json  the board the wires go to: screw terminals, silkscreen, which Feather header
                     position each terminal is wired to, geometry (generated from Eagle files)
  feather/<id>.json  per Feather: header position -> MCU pin and capability, with sources
  target/<id>.json   the controller the switches go to (TinyG v8 J7/J8)
  style.json         capability colours (light and dark) and legend text
The live channel map (channel name -> GPIO) is never in a profile: the UI reads it from the board's
config at runtime; this script uses the firmware's default map (lib/sbcore/sbcore.cpp).
"""
import argparse
import json
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent.parent           # tests/hil/switchbox
BOARDS = ROOT / "boards"
DOCS = ROOT / "docs"
CORE = ROOT / "firmware" / "lib" / "sbcore" / "sbcore.cpp"

USABLE = ("switch", "bootstrap")         # capabilities a channel may use
STRIP_LEN = 39.5                         # mm: label strip outside each board edge
STRIP_GAP = 0.8                          # mm between the board edge and the strip
BAR = 3.0                                # mm: coloured bar with the position number
FONT = 2.15                              # mm: label font (monospace, advance 0.6 em)
ADV = 0.6 * FONT
MARK = 2.8                               # mm kept free at the far end for the live closed/warning mark
STRIP_CHARS = int((STRIP_LEN - BAR - .6 - MARK) / ADV)   # characters that fit in a strip
SEP = "  "
MONO = "ui-monospace,SFMono-Regular,Menlo,Consolas,monospace"
SANS = "system-ui,-apple-system,'Segoe UI',Roboto,sans-serif"


def fmt(v):
    s = f"{v:.2f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def esc(s):
    return (str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace('"', "&quot;"))


# ---- profiles and the terminal map ------------------------------------------------------------

def load(carrier="terminal-block-featherwing", feather="huzzah-esp8266", target="tinyg-v8"):
    read = lambda p: json.loads(p.read_text(encoding="utf-8"))    # noqa: E731
    return {"carrier": read(BOARDS / "carrier" / f"{carrier}.json"),
            "feather": read(BOARDS / "feather" / f"{feather}.json"),
            "target": read(BOARDS / "target" / f"{target}.json") if target else None,
            "style": read(BOARDS / "style.json")}


def default_channels(core=CORE):
    """The firmware's default channel map, [(name, gpio)], read from default_config() in sbcore.cpp."""
    text = pathlib.Path(core).read_text()
    body = text[text.index("void default_config("):]
    body = body[:body.index("};")]
    return [(n, int(g)) for n, g in re.findall(r'\{"([^"]+)",\s*(\d+)\}', body)]


def terminal_map(b):
    """One row per screw terminal: carrier wiring joined with the Feather's pin, plus which terminal
    is the one to use when a GPIO reaches several (the labelled one, else the lowest id)."""
    positions = b["feather"]["positions"]
    rows = []
    for t in b["carrier"]["terminals"]:
        p = positions[t["feather"]]
        rows.append({"id": t["id"], "edge": t["edge"], "position": t["position"], "x": t["x"], "y": t["y"],
                     "silk": t["silk"], "feather": t["feather"], "feather_name": t["feather_name"],
                     "mcu": p["mcu"], "gpio": p["gpio"], "capability": p["capability"], "note": p["note"],
                     "sources": [f"carrier: {t['source']}", f"feather: {p['source']}"], "same_as": None})
    by_gpio = {}
    for r in rows:
        if r["gpio"] is not None:
            by_gpio.setdefault(r["gpio"], []).append(r)
    for group in by_gpio.values():
        primary = sorted(group, key=lambda r: (r["silk"] is None, r["id"][0] != "T", r["position"]))[0]
        for r in group:
            if r is not primary:
                r["same_as"] = primary["id"]
    return rows


def target_pins(b):
    """TinyG input name (e.g. xmin) -> 'J7-3' and the connector rows, from the target profile."""
    pins = {}
    for c in (b["target"] or {}).get("connectors", []):
        for p in c["pins"]:
            if p.get("input"):
                pins[p["input"]] = f"{c['id']}-{p['pin']}"
    return pins


def assign(rows, channels, b):
    """terminal id -> (channel, target pin) for the primary terminal of each channel's GPIO."""
    tp = target_pins(b)
    out = {}
    for name, gpio in channels:
        for r in rows:
            if r["gpio"] == gpio and r["same_as"] is None:
                out[r["id"]] = (name, tp.get(name.lower()))
    return out


def validate(b, rows, channels):
    """Problems with the profiles or a channel map (an empty list means OK)."""
    problems = []
    positions = b["feather"]["positions"]
    styles = b["style"]["capabilities"]
    for t in b["carrier"]["terminals"]:
        if t["feather"] not in positions:
            problems.append(f"terminal {t['id']}: Feather position {t['feather']} missing from {b['feather']['id']}")
    for pos, p in positions.items():
        if p["capability"] not in styles:
            problems.append(f"{b['feather']['id']} {pos}: unknown capability {p['capability']}")
        if not p.get("source"):
            problems.append(f"{b['feather']['id']} {pos}: no source")
    for name, gpio in channels:
        reach = [r for r in rows if r["gpio"] == gpio]
        primary = [r for r in reach if r["same_as"] is None]
        if not reach:
            problems.append(f"channel {name}: GPIO{gpio} reaches no terminal on {b['carrier']['id']}")
        elif len(primary) != 1:
            problems.append(f"channel {name}: GPIO{gpio} has {len(primary)} primary terminals")
        elif primary[0]["capability"] not in USABLE:
            problems.append(f"channel {name}: GPIO{gpio} is {primary[0]['capability']} (unsafe)")
    return problems


def board_json(b):
    """What GET /api/board serves: the profiles, the derived terminal map and render limits."""
    rows = terminal_map(b)
    return {"carrier": b["carrier"], "feather": b["feather"], "target": b["target"], "style": b["style"],
            "terminals": [{k: v for k, v in r.items() if k not in ("x", "y")} for r in rows],
            "render": {"strip_chars": STRIP_CHARS, "sep": SEP}}


# ---- label text ---------------------------------------------------------------------------------

def strip_parts(r, assignment, b):
    """[(text, css class)] from the board edge outwards for one terminal's label strip."""
    silk = (r["silk"], "silk") if r["silk"] else (f"[{r['feather_name']}]", "silk unl")
    cap = r["capability"]
    parts = [silk]
    if r["gpio"] is not None:
        parts.append((f"GPIO{r['gpio']}", "gpio"))
    elif cap == "power":
        parts.append(({"3V3": "3.3V out", "VBUS": "USB 5V", "VBAT": "battery"}.get(r["mcu"], r["mcu"] or ""), "role"))
    elif cap in ("control", "analog"):
        parts.append((r["mcu"].split(" ")[0], "gpio"))
    if r["same_as"]:
        primary = next(x for x in terminal_map(b) if x["id"] == r["same_as"])
        parts.append((f"= {primary['silk'] or primary['feather_name']} {primary['id']}", "alt"))
    elif r["id"] in assignment:
        ch, tg = assignment[r["id"]]
        parts.append((ch, "ch"))
        if tg:
            parts.append((tg, "tg"))
    else:
        parts.append(({"switch": "free", "bootstrap": "free", "pulldown": "not usable", "reserved": "serial",
                       "control": "do not wire", "analog": "analog only", "gnd": "common ground",
                       "nc": "not connected", "power": ""}[cap], "role"))
    return [(t, c) for t, c in parts if t]


def fit(parts, limit=STRIP_CHARS):
    """Truncate the channel name if the strip would overflow."""
    total = sum(len(t) for t, _ in parts) + len(SEP) * (len(parts) - 1)
    if total <= limit:
        return parts
    out = []
    for t, c in parts:
        if c == "ch":
            keep = max(1, len(t) - (total - limit) - 1)
            t = t[:keep] + "…"
        out.append((t, c))
    return out


# ---- SVG ----------------------------------------------------------------------------------------

def style_block(b, dark=None):
    """CSS for the drawings: capability colours as variables, light and dark."""
    caps = b["style"]["capabilities"]
    light = "".join(f"--c-{k}:{v['light']};" for k, v in caps.items())
    darkv = "".join(f"--c-{k}:{v['dark']};" for k, v in caps.items())
    base_l = ("--ink:#1f2328;--mute:#59636e;--bg:#ffffff;--strip:#f6f8fa;--line:#d1d9e0;--pcb:#1c2128;"
              "--pcb-edge:#0d1117;--silk:#f0f3f6;--block:#2da44e;--block-edge:#1a7f37;--screw:#d0d7de;"
              "--feather:#24292f;--feather-edge:#57606a;--closed:#cf222e;--warn:#bf8700;--hl:#0969da;--on-gnd:#ffffff;")
    base_d = ("--ink:#e6edf3;--mute:#9198a1;--bg:#0d1117;--strip:#161b22;--line:#30363d;--pcb:#1c2128;"
              "--pcb-edge:#3d444d;--silk:#f0f3f6;--block:#238636;--block-edge:#2ea043;--screw:#b7bdc8;"
              "--feather:#2d333b;--feather-edge:#768390;--closed:#ff7b72;--warn:#d29922;--hl:#58a6ff;--on-gnd:#0d1117;")
    if dark is True:
        root = f".sbw{{{base_d}{darkv}}}"
    elif dark is False:
        root = f".sbw{{{base_l}{light}}}"
    else:
        root = (f".sbw{{{base_l}{light}}}"
                f"@media (prefers-color-scheme:dark){{.sbw{{{base_d}{darkv}}}}}")
    rules = (
        ".sbw text{font-family:" + MONO + ";fill:var(--ink);white-space:pre}"
        ".sbw .t-sans{font-family:" + SANS + "}"
        ".sbw .pcb{fill:var(--pcb);stroke:var(--pcb-edge);stroke-width:.3}"
        ".sbw .hole{fill:var(--bg);stroke:#c9a227;stroke-width:.5}"
        ".sbw .block{fill:var(--block);stroke:var(--block-edge);stroke-width:.25}"
        ".sbw .screw{fill:var(--screw);stroke:#57606a;stroke-width:.2}"
        ".sbw .slot{stroke:#57606a;stroke-width:.25}"
        ".sbw .feather{fill:var(--feather);stroke:var(--feather-edge);stroke-width:.3;fill-opacity:.92}"
        ".sbw .silkt{fill:var(--silk);font-family:" + SANS + "}"
        ".sbw .strip{fill:var(--strip);stroke:var(--line);stroke-width:.15}"
        ".sbw .unlabeled .strip{stroke-dasharray:.6 .4}"
        ".sbw .alt .strip{fill-opacity:.5}"
        ".sbw .bar{stroke:none}"
        ".sbw .pos{fill:#fff;font-size:1.5px;font-weight:700;font-family:" + SANS + "}"
        ".sbw .cap-gnd .pos{fill:var(--on-gnd)}"
        ".sbw .lbl{font-size:" + fmt(FONT) + "px;white-space:pre}"
        ".sbw .silk{font-weight:700}"
        ".sbw .unl{font-weight:400;font-style:italic;fill:var(--mute)}"
        ".sbw .ch{font-weight:700;fill:var(--hl)}"
        ".sbw .tg{fill:var(--mute)}"
        ".sbw .role,.sbw .alt{fill:var(--mute)}"
        ".sbw .hdr{stroke:#000;stroke-width:.1}"
        ".sbw .hdrt{font-size:1.15px;fill:var(--silk);font-family:" + SANS + "}"
        ".sbw .trace{stroke:var(--hl);stroke-width:.35;fill:none;opacity:0}"
        ".sbw .term.hl .trace{opacity:.9}"
        ".sbw .term.hl .strip{stroke:var(--hl);stroke-width:.45}"
        ".sbw .term .state{display:none}"
        ".sbw .term.closed .screw{fill:var(--closed)}"
        ".sbw .term.closed .state.closed{display:inline}"
        ".sbw .term.warn .strip{stroke:var(--warn);stroke-width:.5}"
        ".sbw .term.warn .state.warn{display:inline}"
        ".sbw .lvl{display:none}"
        ".sbw .term.lv0 .lvl,.sbw .term.lv1 .lvl{display:inline}"
        ".sbw .term.lv0 .lvl{fill:#0969da}.sbw .term.lv1 .lvl{fill:#2da44e}"
        ".sbw .head{font-family:" + SANS + ";font-weight:700}"
        ".sbw .note{font-family:" + SANS + ";fill:var(--mute)}"
        ".sbw .gndnote{font-family:" + SANS + ";font-weight:700;fill:var(--c-gnd)}"
        ".sbw .pin{fill:var(--strip);stroke:var(--line);stroke-width:.2}"
    )
    caps_rules = "".join(f".sbw .cap-{k} .bar,.sbw .fill-{k}{{fill:var(--c-{k})}}"
                         f".sbw .cap-{k} .gpio{{fill:var(--c-{k})}}" for k in caps)
    return root + rules + caps_rules


class Svg:
    """Collects elements and the boxes of every text, for the overlap check."""

    def __init__(self):
        self.out = []
        self.boxes = []          # (x0, y0, x1, y1, what)

    def add(self, s):
        self.out.append(s)

    def text_box(self, x0, y0, x1, y1, what):
        self.boxes.append((min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1), what))


def flip(y):
    return -y                     # Eagle y up -> SVG y down


def board_svg(b, rows, assignment, ids=True):
    """The breakout with the Feather on it and one label strip per terminal. Returns (Svg, viewBox)."""
    c = b["carrier"]
    o = c["outline"]
    s = Svg()
    pitch = c["terminal_pitch"]
    caps = {r["feather"]: r["capability"] for r in rows}
    # board, holes, switch
    s.add(f'<rect class="pcb" x="{fmt(o["x0"])}" y="{fmt(flip(o["y1"]))}" width="{fmt(o["x1"] - o["x0"])}" '
          f'height="{fmt(o["y1"] - o["y0"])}" rx="{fmt(o["corner"])}"/>')
    for h in c["holes"]:
        s.add(f'<circle class="hole" cx="{fmt(h["x"])}" cy="{fmt(flip(h["y"]))}" r="{fmt(h["d"] / 2)}"/>')
    sw = c["switch"]
    sx, sy = sw["x"], flip(sw["y"])
    s.add(f'<rect x="{fmt(sx - 1.6)}" y="{fmt(sy - 4.4)}" width="3.2" height="8.8" rx=".5" fill="#d0d7de" stroke="#57606a" stroke-width=".2"/>'
          f'<rect x="{fmt(sx + 1.6)}" y="{fmt(sy - 3.4)}" width="2.4" height="1.6" fill="#24292f"/>')
    s.add(f'<text class="silkt" font-size="1.3" x="{fmt(sx - 2.6)}" y="{fmt(sy)}" text-anchor="middle" '
          f'transform="rotate(-90 {fmt(sx - 2.6)} {fmt(sy)})">3v Enable: ON</text>')
    s.text_box(sx - 3.3, sy - 5.2, sx - 1.9, sy + 5.2, "switch label")
    # terminal blocks (one rect per Eagle block)
    blocks = {}
    for t in c["terminals"]:
        blocks.setdefault(t["block"], []).append(t)
    for name in sorted(blocks, key=lambda n: int(n[1:])):
        ts = blocks[name]
        x0 = min(t["x"] for t in ts) - pitch / 2
        x1 = max(t["x"] for t in ts) + pitch / 2
        yc = flip(ts[0]["y"])
        out = -1 if yc < 0 else 1             # towards the nearer board edge
        y0, y1 = sorted((yc - 2.6 * out, yc + 4.0 * out))
        s.add(f'<rect class="block" x="{fmt(x0 + .15)}" y="{fmt(y0)}" width="{fmt(x1 - x0 - .3)}" height="{fmt(y1 - y0)}" rx=".4"/>')
    # Feather, its headers and their pin labels
    f = c["feather"]
    fx, fy = f["x0"], flip(f["y0"] + f["height"])
    s.add(f'<rect class="feather" x="{fmt(fx)}" y="{fmt(fy)}" width="{fmt(f["width"])}" height="{fmt(f["height"])}" rx="2"/>')
    s.add(f'<rect x="{fmt(fx - 1.2)}" y="{fmt(fy + f["height"] / 2 - 2.2)}" width="5" height="4.4" rx=".5" fill="#8c959f"/>')
    s.add(f'<text class="silkt" font-size="1.25" x="{fmt(fx + 5)}" y="{fmt(fy + f["height"] / 2 + .45)}">USB</text>')
    s.text_box(fx + 5, fy + f["height"] / 2 - .6, fx + 8.4, fy + f["height"] / 2 + .6, "usb")
    fname = b["feather"]["name"].replace("Adafruit ", "")
    s.add(f'<text class="silkt" font-size="1.7" font-weight="700" x="0" y="{fmt(fy + f["height"] / 2 - .2)}" text-anchor="middle">{esc(fname)}</text>')
    s.add(f'<text class="silkt" font-size="1.2" x="0" y="{fmt(fy + f["height"] / 2 + 2.1)}" text-anchor="middle">plugs in here, USB end towards the left</text>')
    s.text_box(-len(fname) * .55, fy + f["height"] / 2 - 1.6, len(fname) * .55, fy + f["height"] / 2 + 2.6, "feather name")
    positions = b["feather"]["positions"]
    for h in f["headers"]:
        p = positions[h["pos"]]
        hx, hy = h["x"], flip(h["y"])
        cap = p["capability"]
        s.add(f'<rect class="hdr fill-{cap}" x="{fmt(hx - .75)}" y="{fmt(hy - .75)}" width="1.5" height="1.5" rx=".2"/>')
        short = (str(p["gpio"]) if p["gpio"] is not None else
                 {"3V3": "3V", "GND": "G", "RST": "RST", "CH_PD": "CH", "EN": "EN", "VBAT": "BAT", "VBUS": "USB",
                  "ADC (TOUT)": "A"}.get(p["mcu"] or "", "nc"))
        ty = hy + 2.0 if hy < 0 else hy - 1.3       # labels inside the Feather outline
        s.add(f'<text class="hdrt" x="{fmt(hx)}" y="{fmt(ty)}" text-anchor="middle">{esc(short)}</text>')
        s.text_box(hx - len(short) * .33, ty - .95, hx + len(short) * .33, ty + .2, f"hdr {h['pos']}")
    # terminals: block screw, label strip, live-state marks, trace to its header pin
    hdr = {h["pos"]: h for h in f["headers"]}
    for r in rows:
        x, yc = r["x"], flip(r["y"])
        top = yc < 0
        edge = flip(o["y1"]) if top else flip(o["y0"])
        cls = f'term cap-{r["capability"]}' + (" unlabeled" if not r["silk"] else "") + (" alt" if r["same_as"] else "")
        attrs = f' id="t-{r["id"]}" data-term="{r["id"]}"' if ids else ""
        if r["gpio"] is not None:
            attrs += f' data-gpio="{r["gpio"]}"'
        s.add(f'<g class="{cls}"{attrs}>')
        h = hdr[r["feather"]]
        s.add(f'<path class="trace" d="M{fmt(x)} {fmt(yc)}L{fmt(h["x"])} {fmt(flip(h["y"]))}"/>')
        s.add(f'<circle class="screw" cx="{fmt(x)}" cy="{fmt(yc)}" r="1.25"/>'
              f'<path class="slot" d="M{fmt(x - .8)} {fmt(yc + .8)}L{fmt(x + .8)} {fmt(yc - .8)}"/>')
        # strip
        sgn = -1 if top else 1
        near = edge + sgn * STRIP_GAP
        far = near + sgn * STRIP_LEN
        y0, y1 = sorted((near, far))
        w = pitch - .5
        s.add(f'<rect class="strip" x="{fmt(x - w / 2)}" y="{fmt(y0)}" width="{fmt(w)}" height="{fmt(y1 - y0)}" rx=".5"/>')
        b0, b1 = sorted((near, near + sgn * BAR))
        s.add(f'<rect class="bar" x="{fmt(x - w / 2)}" y="{fmt(b0)}" width="{fmt(w)}" height="{fmt(b1 - b0)}" rx=".5"/>')
        s.add(f'<text class="pos" x="{fmt(x)}" y="{fmt((b0 + b1) / 2 + .55)}" text-anchor="middle">{r["position"]}</text>')
        parts = fit(strip_parts(r, assignment, b))
        live = r["capability"] in USABLE and not r["same_as"]
        if live and not any(k == "tg" for _, k in parts):
            parts = parts + [("", "tg")]              # filled in by the UI when a channel is assigned
        start = near + sgn * (BAR + .6)
        tx = x + FONT * .35
        anchor = "start" if top else "end"
        seq = parts if top else list(reversed(parts))
        spans = []
        for i, (t, k) in enumerate(seq):
            pre = SEP if (i and top and t) else ""
            post = SEP if (i < len(seq) - 1 and not top and t) else ""
            role = ""
            if k == "ch":
                role = ' data-role="ch"'
            elif k == "tg":
                role = ' data-role="tg"'
            elif k == "role" and r["capability"] in USABLE and not r["same_as"]:
                role = ' data-role="ch"'
            spans.append(f'<tspan class="{k}"{role}>{esc(pre + t + post)}</tspan>')
        shown = [t for t, _ in parts if t]
        text_len = sum(len(t) for t in shown) + len(SEP) * (len(shown) - 1)
        s.add(f'<text class="lbl" x="{fmt(tx)}" y="{fmt(start)}" text-anchor="{anchor}" '
              f'transform="rotate(-90 {fmt(tx)} {fmt(start)})" data-sep="{"before" if top else "after"}">{"".join(spans)}</text>')
        span = text_len * ADV
        s.text_box(x - FONT / 2, start, x + FONT / 2, start + (sgn * span), f"strip {r['id']}")
        assert text_len <= STRIP_CHARS, (r["id"], parts)
        # live marks (hidden until the UI sets classes)
        mark_y = far - sgn * 1.8
        s.text_box(x - 1, mark_y - 1.3, x + 1, mark_y + 1.3, f"mark {r['id']}")
        s.add(f'<text class="state closed t-sans" x="{fmt(x)}" y="{fmt(mark_y + .6)}" font-size="1.8" text-anchor="middle" fill="var(--closed)">●</text>'
              f'<text class="state warn t-sans" x="{fmt(x)}" y="{fmt(mark_y + .6)}" font-size="2" text-anchor="middle" fill="var(--warn)">!</text>'
              f'<circle class="lvl" cx="{fmt(x + 1.15)}" cy="{fmt((b0 + b1) / 2 - 1.0)}" r=".35"/>')
        s.add("</g>")
    pad = 1.6
    vx0, vx1 = o["x0"] - pad, o["x1"] + 2.4
    vy0 = flip(o["y1"]) - STRIP_GAP - STRIP_LEN - pad
    vy1 = flip(o["y0"]) + STRIP_GAP + STRIP_LEN + pad
    return s, (vx0, vy0, vx1 - vx0, vy1 - vy0)


def tinyg_svg(b, rows, assignment, width):
    """Where each wire lands on the TinyG (target profile), with the common-ground reminder."""
    s = Svg()
    tgt = b["target"]
    if not tgt:
        return s, 0
    by_input = {}
    for tid, (ch, _) in assignment.items():
        r = next(x for x in rows if x["id"] == tid)
        by_input[ch.lower()] = (ch, r)
    gnd_terms = [r for r in rows if r["capability"] == "gnd"]
    fs = 1.75
    y = 2.4
    s.add(f'<text class="head" x="0" y="{fmt(y)}" font-size="2.4">{esc(tgt["name"])} switch terminals</text>')
    y += 3.0
    gtxt = "Common ground: wire " + "/".join(r["id"] for r in gnd_terms) + " (GND) to J7-1 or J8-1"
    s.add(f'<text class="gndnote" x="0" y="{fmt(y)}" font-size="{fmt(fs)}">{esc(gtxt)}</text>')
    y += 1.6
    colw = width / 2
    top = y
    for ci, conn in enumerate(tgt["connectors"]):
        x0 = ci * colw
        yy = top + 2.6
        s.add(f'<text class="head" x="{fmt(x0)}" y="{fmt(yy)}" font-size="1.9">{esc(conn["id"])}  {esc(conn["label"])}</text>')
        for p in conn["pins"]:
            yy += 2.7
            s.add(f'<rect class="pin" x="{fmt(x0)}" y="{fmt(yy - 1.9)}" width="2.6" height="2.4" rx=".3"/>'
                  f'<text x="{fmt(x0 + 1.3)}" y="{fmt(yy - .1)}" font-size="1.5" text-anchor="middle">{p["pin"]}</text>')
            label = p["label"]
            if p.get("kind") == "gnd":
                right = "← GND " + "/".join(r["id"] for r in gnd_terms)
                k = "gndnote"
            elif p.get("kind") == "power":
                right = "leave unconnected"
                k = "role"
            elif p["input"] in by_input:
                ch, r = by_input[p["input"]]
                right = f"← {ch}  {r['silk'] or '[' + r['feather_name'] + ']'} {r['id']}"
                k = "ch"
            else:
                right = "unused"
                k = "role"
            s.add(f'<text x="{fmt(x0 + 3.4)}" y="{fmt(yy)}" font-size="{fmt(fs)}">'
                  f'<tspan class="silk">{esc(label):6}</tspan><tspan class="{k}" xml:space="preserve"> {esc(right)}</tspan></text>')
            s.text_box(x0 + 3.4, yy - 1.4, x0 + 3.4 + (len(label) + 1 + len(right)) * fs * .6, yy + .3, f"tinyg {conn['id']}-{p['pin']}")
    notes = ["Each TinyG input: 2.7 kΩ pull-up to 3.3 V and 0.22 µF to GND",
             "(v8h schematic), so edges under about 1 ms are smoothed."]
    yy = top + 2.6 + 2.7 * 6 + 1.6
    for n in notes:
        yy += 2.1
        s.add(f'<text class="note" x="0" y="{fmt(yy)}" font-size="1.45">{esc(n)}</text>')
        s.text_box(0, yy - 1.1, len(n) * 1.45 * .6, yy + .3, "tinyg note")
    return s, yy + 1.2


def legend_svg(b, width):
    s = Svg()
    caps = b["style"]["capabilities"]
    order = b["style"]["order"]
    fs = 1.75
    for i, k in enumerate(order):
        y = 1.6 + i * 2.8
        s.add(f'<rect class="fill-{k}" x="0" y="{fmt(y - 1.5)}" width="2.2" height="2.2" rx=".4"/>'
              f'<text class="t-sans" x="3" y="{fmt(y + .3)}" font-size="{fmt(fs)}">{esc(caps[k]["label"])}</text>')
        s.text_box(3.0, y - 1.2, 3.0 + len(caps[k]["label"]) * fs * .6, y + .5, f"legend {k}")
        assert 3.0 + len(caps[k]["label"]) * fs * .6 <= width + 2, k
    y = 1.6 + len(order) * 2.8 + .4
    tips = ["[13] = position with no silkscreen label (count from the 3v3 end)",
            "= SCK B11: same GPIO as that terminal, leave it free"]
    for t in tips:
        s.add(f'<text class="note" x="0" y="{fmt(y)}" font-size="1.5">{esc(t)}</text>')
        y += 2.2
    return s, y


def wrap_svg(inner, viewbox, b, cls="sbw", dark=None, px_width=None, background=False):
    vx, vy, vw, vh = viewbox
    size = f' width="{fmt(px_width)}" height="{fmt(px_width * vh / vw)}"' if px_width else ""
    bg = f'<rect x="{fmt(vx)}" y="{fmt(vy)}" width="{fmt(vw)}" height="{fmt(vh)}" fill="var(--bg)"/>' if background else ""
    return (f'<svg xmlns="http://www.w3.org/2000/svg" class="{cls}" viewBox="{fmt(vx)} {fmt(vy)} {fmt(vw)} {fmt(vh)}"{size} '
            f'role="img"><style>{style_block(b, dark)}</style>{bg}{inner}</svg>')


def ui_fragments(b, channels=None):
    """The three SVGs the web UI's Wiring view embeds (live state is added in the browser)."""
    rows = terminal_map(b)
    asg = assign(rows, channels if channels is not None else default_channels(), b)
    board, vb = board_svg(b, rows, asg)
    width = vb[2]
    tg, th = tinyg_svg(b, rows, asg, width - 4)
    lg, lh = legend_svg(b, width - 4)
    return {"board": wrap_svg("".join(board.out), vb, b),
            "tinyg": wrap_svg("".join(tg.out), (-1, 0, width - 2, th), b),
            "legend": wrap_svg("".join(lg.out), (-1, 0, width - 2, lh), b)}


def docs_svg(b, channels, dark=None):
    """One image: title, board drawing, then legend and TinyG panel side by side."""
    rows = terminal_map(b)
    asg = assign(rows, channels, b)
    board, vb = board_svg(b, rows, asg, ids=False)
    vx, vy, vw, vh = vb
    side_w = 66
    tg, th = tinyg_svg(b, rows, asg, side_w)
    lg, lh = legend_svg(b, side_w)
    gap = 6
    title_h = 12
    total_w = vw + gap + side_w + 4
    total_h = max(vh, th + lh + 10) + title_h
    title = (f'<text class="head" x="{fmt(vx + 1)}" y="{fmt(vy + 4.6)}" font-size="3.6">Switchbox wiring</text>'
             f'<text class="note" x="{fmt(vx + 1)}" y="{fmt(vy + 8)}" font-size="1.8">'
             f'{esc(b["carrier"]["name"])} with {esc(b["feather"]["name"])}</text>'
             f'<text class="note" x="{fmt(vx + 1)}" y="{fmt(vy + 10.6)}" font-size="1.8">Default channel map. '
             f'Terminals are numbered from the 3v3 end of each edge. Generated by tools/render_pinout.py.</text>')
    body = f'<g transform="translate(0 {fmt(title_h)})">{"".join(board.out)}</g>'
    sx = vx + vw + gap
    side = (f'<g transform="translate({fmt(sx)} {fmt(vy + title_h + 4)})">{"".join(lg.out)}</g>'
            f'<g transform="translate({fmt(sx)} {fmt(vy + title_h + lh + 10)})">{"".join(tg.out)}</g>')
    viewbox = (vx, vy, total_w, total_h)
    return wrap_svg(title + body + side, viewbox, b, dark=dark, px_width=1400, background=True)


def overlaps(boxes):
    """Pairs of text boxes that intersect (touching edges allowed)."""
    bad = []
    for i, a in enumerate(boxes):
        for c in boxes[i + 1:]:
            if a[0] < c[2] - 1e-6 and c[0] < a[2] - 1e-6 and a[1] < c[3] - 1e-6 and c[1] < a[3] - 1e-6:
                bad.append((a[4], c[4]))
    return bad


# ---- Markdown -----------------------------------------------------------------------------------

def markdown(b, channels):
    rows = terminal_map(b)
    asg = assign(rows, channels, b)
    caps = b["style"]["capabilities"]
    edges = b["carrier"]["edges"]
    out = [f"# Wiring: {b['carrier']['name']} + {b['feather']['name']}", "",
           "Generated by `tools/render_pinout.py` from `boards/` (do not edit). Terminals are numbered from the "
           "3v3 end of each edge, with the Micro-USB end of the Feather on the left:",
           f"- **T1-T16**: {edges['scl_sda']};",
           f"- **B1-B16**: {edges['rx_tx']}.",
           "",
           "A label in [brackets] is not printed on the board; the bracket shows the Feather pin name. The channel column is the firmware's default map; the live map is in the board's config.", "",
           "| terminal | silkscreen | Feather pin | ESP pin | capability | default channel | TinyG | sources |",
           "|---|---|---|---|---|---|---|---|"]
    for r in rows:
        ch, tg = asg.get(r["id"], ("", ""))
        if r["same_as"]:
            ch = f"same signal as {r['same_as']}: leave free"
        out.append(f"| {r['id']} | {r['silk'] or '[' + r['feather_name'] + ']'} | {r['feather']} ({r['feather_name']}) | "
                   f"{r['mcu'] or '-'} | {caps[r['capability']]['label']} | {ch or ''} | {tg or ''} | "
                   + "; ".join(r["sources"]).replace("|", "/") + " |")
    out += ["", f"Profiles: `boards/carrier/{b['carrier']['id']}.json`, `boards/feather/{b['feather']['id']}.json`, "
            f"`boards/target/{b['target']['id']}.json`.", ""]
    return "\n".join(out)


# ---- PNG ----------------------------------------------------------------------------------------

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"


def to_png(svg_path, png_path, width_px=1400):
    """Render an SVG to PNG with a local renderer: rsvg-convert, else headless Chrome. Returns the tool used."""
    svg_path, png_path = pathlib.Path(svg_path), pathlib.Path(png_path)
    if shutil.which("rsvg-convert"):
        subprocess.run(["rsvg-convert", "-w", str(width_px), "-o", str(png_path), str(svg_path)], check=True)
        return "rsvg-convert"
    chrome = shutil.which("google-chrome") or shutil.which("chromium") or (CHROME if pathlib.Path(CHROME).exists() else None)
    if not chrome:
        return None
    m = re.search(r'width="([\d.]+)" height="([\d.]+)"', svg_path.read_text())
    w, h = (int(float(m.group(1))), int(float(m.group(2)) + .999)) if m else (width_px, width_px)
    with tempfile.TemporaryDirectory() as prof:
        subprocess.run([chrome, "--headless=new", "--disable-gpu", "--hide-scrollbars", f"--user-data-dir={prof}",
                        "--force-device-scale-factor=1", f"--window-size={w},{h}", f"--screenshot={png_path}",
                        "--default-background-color=00000000", svg_path.resolve().as_uri()],
                       check=True, capture_output=True, timeout=120)
    return "headless Chrome"


def outputs(b, channels):
    return {DOCS / "wiring.svg": docs_svg(b, channels, dark=False),
            DOCS / "wiring-dark.svg": docs_svg(b, channels, dark=True),
            DOCS / "wiring.md": markdown(b, channels)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--carrier", default="terminal-block-featherwing")
    ap.add_argument("--feather", default="huzzah-esp8266")
    ap.add_argument("--target", default="tinyg-v8")
    ap.add_argument("--check", action="store_true", help="exit 1 if the committed outputs differ")
    ap.add_argument("--no-png", action="store_true")
    args = ap.parse_args(argv)
    b = load(args.carrier, args.feather, args.target)
    channels = default_channels()
    problems = validate(b, terminal_map(b), channels)
    if problems:
        print("\n".join(problems), file=sys.stderr)
        return 1
    outs = outputs(b, channels)
    if args.check:
        stale = [str(p.relative_to(ROOT)) for p, text in outs.items() if not p.exists() or p.read_text() != text]
        if stale:
            print("out of date: " + ", ".join(stale), file=sys.stderr)
            return 1
        return 0
    for p, text in outs.items():
        p.write_text(text)
        print(f"wrote {p.relative_to(ROOT)}")
    if not args.no_png:
        for name in ("wiring", "wiring-dark"):
            tool = to_png(DOCS / f"{name}.svg", DOCS / f"{name}.png")
            print(f"{name}.png: " + (f"rendered with {tool}" if tool else "no SVG renderer found, skipped"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
