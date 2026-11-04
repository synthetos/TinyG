#!/usr/bin/env python3
"""Overnight G-code: generated files (good, ugly-valid, mixed with refused lines) and a filtered corpus copy.

usage: gen_gcode.py <outdir> <corpus_dir> <corpus_out>
Generated motion stays within +/-60 mm of the start in X/Y and +/-10 in Z (unloaded bench motors).
Refused-line kinds avoid alarms (no bad arc radii), so a stream keeps going after them.
"""
import math
import re, os, random, re, sys

OUT, CORPUS, CORPUS_OUT = sys.argv[1], sys.argv[2], sys.argv[3]
BOX = 60.0


def num(r, v, ugly):
    s = f"{v:.4f}".rstrip("0").rstrip(".") if not ugly else r.choice([f"{v:.3f}", f"{v:+.4f}", f"{v:.1f}", f"{v:.5f}", f"{v:08.3f}"])
    if ugly and s.startswith("0.") and r.random() < 0.3:
        s = s[1:]                                        # .5
    return s or "0"


MAX_MINUTES = 8.0


def _fits(lines, minutes):
    """How many leading lines run within `minutes` at their feeds (no acceleration): G0 at 16 m/min, G93 by F."""
    x = y = z = a = 0.0; f = 0.0; mode = "G0"; t = 0.0; g93 = False
    for n, raw in enumerate(lines):
        s = raw.split("(")[0].split(";")[0].strip().upper()
        if not s or s[0] in "{$":
            continue
        for g in re.findall(r"G0*([0-3])\b", s):
            mode = "G" + g
        g93 = "G93" in s or (g93 and "G94" not in s)
        w = {k: float(v) for k, v in re.findall(r"([XYZAF])(-?[0-9.]+)", s)}
        f = w.get("F", f)
        if not any(k in w for k in "XYZA"):
            continue
        nx, ny, nz, na = w.get("X", x), w.get("Y", y), w.get("Z", z), w.get("A", a)
        d, da = math.dist((x, y, z), (nx, ny, nz)), abs(na - a)
        if mode == "G0":
            t += max(d / (16000 / 60), da / (3600 / 60))
        elif g93:
            t += 60 / f if f else 1
        else:
            t += max(d, da) / (f / 60 if f else 0.001)
        x, y, z, a = nx, ny, nz, na
        if t > minutes * 60:
            return n
    return len(lines)


def ugly_line(r, words):
    sep = r.choice([" ", "", "  ", "\t"])
    line = sep.join(words)
    if r.random() < 0.3:
        line = line.lower()
    if r.random() < 0.15:
        line = f"N{r.randint(1, 99999)} " + line
    if r.random() < 0.15:
        line += r.choice([" (comment)", " ;comment", "(c)", " ( spaced  comment ) "])
    return line


def gen(seed, kind):
    r = random.Random(seed)
    ugly = kind != "good"
    x = y = z = a = 0.0
    feed = r.choice([300, 600, 1200, 2400])
    out = ["G21 G90 G17 G94", "G92 X0 Y0 Z0 A0", f"F{feed}"]     # a feed before the first G1: no F is refused (142)
    n = r.randint(300, 3000)
    for _ in range(n):
        op = r.random()
        if op < 0.45:                                    # G1 in a random-length step
            step = r.choice([0.005, 0.02, 0.1, 0.5, 2.0, 10.0])
            nx = max(-BOX, min(BOX, x + r.uniform(-step, step)))
            ny = max(-BOX, min(BOX, y + r.uniform(-step, step)))
            xs, ys = num(r, nx, ugly), num(r, ny, ugly)
            words = ["G1", "X" + xs, "Y" + ys]
            if r.random() < 0.2:
                feed = r.choice([100, 300, 600, 1200, 2400, 5000]); words.append(f"F{feed}")
            out.append(ugly_line(r, words) if ugly else " ".join(words))
            x, y = float(xs), float(ys)                  # track what the board saw: ugly rounding must not break the next arc
        elif op < 0.55:                                  # arc with consistent I/J
            rad = r.choice([0.5, 2.0, 5.0, 15.0])
            ang = r.uniform(0, 2 * math.pi)
            cx, cy = x - rad * math.cos(ang), y - rad * math.sin(ang)
            a2 = ang + r.choice([-1, 1]) * r.uniform(0.2, 3.0)
            nx, ny = cx + rad * math.cos(a2), cy + rad * math.sin(a2)
            if abs(nx) > BOX or abs(ny) > BOX:
                continue
            g = "G3" if a2 > ang else "G2"
            xs, ys = num(r, nx, False), num(r, ny, False)
            words = [g, "X" + xs, "Y" + ys, "I" + num(r, cx - x, False), "J" + num(r, cy - y, False)]
            out.append(ugly_line(r, words) if ugly else " ".join(words)); x, y = float(xs), float(ys)
        elif op < 0.62:                                  # rapid
            nx, ny = r.uniform(-BOX, BOX), r.uniform(-BOX, BOX)
            out.append(f"G0 X{nx:.3f} Y{ny:.3f}"); x, y = round(nx, 3), round(ny, 3)
        elif op < 0.67:                                  # Z
            nz = max(-10, min(10, z + r.uniform(-2, 2)))
            out.append(f"G1 Z{nz:.3f} F{r.choice([100, 300])}"); z = nz
        elif op < 0.72:                                  # relative block, then back to absolute
            dx, dy = r.uniform(-3, 3), r.uniform(-3, 3)
            if abs(x + dx) < BOX and abs(y + dy) < BOX:
                out += ["G91", f"G1 X{dx:.3f} Y{dy:.3f}", "G90"]; x += dx; y += dy
        elif op < 0.76:                                  # spindle / coolant between moves
            out.append(r.choice(["M3 S1000", "M3 S12000", "M4 S500", "M5", "M7", "M8", "M9", f"S{r.randint(0, 20000)}"]))
        elif op < 0.78:
            out.append(f"G4 P{r.choice([0.01, 0.05, 0.2])}")
        elif op < 0.79:                                  # rotary: A alone, XY+A, or a G93 inverse-time block
            na = max(-720, min(720, a + r.uniform(-90, 90)))
            pick = r.random()
            if pick < 0.4:
                out.append(f"G1 A{na:.3f} F{r.choice([360, 1800, 7200])}")
            elif pick < 0.75:
                nx = max(-BOX, min(BOX, x + r.uniform(-5, 5)))
                out.append(f"G1 X{nx:.3f} A{na:.3f} F{r.choice([300, 1200])}"); x = nx
            else:
                nx = max(-BOX, min(BOX, x + r.uniform(-5, 5)))
                out += ["G93", f"G1 X{nx:.3f} A{na:.3f} F{r.choice([30, 120, 600])}", "G94", f"F{feed}"]; x = nx
            a = na
        elif op < 0.80 and ugly:                         # inch block
            out += ["G20", f"G1 X{x / 25.4:.4f} Y{y / 25.4:.4f}", "G21"]
        elif op < 0.83:                                  # exact stop / continuous
            out.append(r.choice(["G61", "G61.1", "G64"]))
        elif op < 0.86 and ugly:                         # blank, whitespace, comment-only, long comment
            out.append(r.choice(["", "   ", "(only a comment)", "; semicolon", "(" + "x" * r.randint(50, 150) + ")"]))
        elif op < 0.90 and kind == "chain":              # new features chained into the live job
            out.append(r.choice([
                '{"rt":{"flk":null,"flm":null,"flp":null,"flf":null}}', '{"rt":{"fs":null}}', '{"rt":{"up":null}}',
                '{"rt":{"fi":2}}', '{"rt":{"fi":1}}', '{"rt":{"hb":1}}', '{"rt":{"hb":0}}', '{"rt":null}',
                '{"sr":null}', '{"spe":null}', '{"sps":null}', '{"spc":null}', '{"cof":null}', '{"com":null}',
                '{"qr":null}', '{"stat":null}', '{"hold":null}', 'M3 S8000', 'M5', 'M8', 'M9', 'M7',
                '{"xjm":20}', '{"si":200}', '{"ja":100000}']))     # settings written while moving
        elif op < 0.90 and kind == "mixed":              # refused lines (no alarms)
            out.append(r.choice(["G1 X1X2", "G99", "M99", "G1 Xnan", "G1 X1e40", "Q5", "G1 X", "N-1 G1 X0",
                                 "{\"xvm\":1}x", "G1 F-5 X1", "G28.9", "G38.9", "G41 D1", "G43 H1",
                                 "G81 R1 Z-1", "M98 P100", "#1=5", "G1 X[1+2]", "G1 X1 G0 Y1", "G90 G91 X1", "O100 sub"]))
    out = out[:_fits(out, MAX_MINUTES)]                 # a hunt wants many short jobs, not one slow hour
    out += ["G94 G90 G0 X0 Y0 Z0 A0", "G92.1"]
    if ugly and random.Random(seed + 1).random() < 0.5:
        return "\r\n".join(out) + "\r\n"
    return "\n".join(out) + "\n"


os.makedirs(OUT, exist_ok=True)
for i in range(80):
    kind = ["good", "ugly", "mixed", "chain"][i % 4]
    with open(os.path.join(OUT, f"gen-{kind}-{i:03d}.nc"), "w", newline="") as f:
        f.write(gen(1000 + i, kind))

DROP = re.compile(r"\b(G10|G28|G30|G38|M0?0|M0?1|M60)(?![0-9.])", re.I)
os.makedirs(CORPUS_OUT, exist_ok=True)
kept = 0
for root, _, files in os.walk(CORPUS):
    for name in sorted(files):
        if not re.search(r"\.(nc|ngc|gcode|tap|cnc|gc|txt)$", name, re.I):
            continue
        src = os.path.join(root, name)
        text = open(src, "rb").read().decode("latin-1")
        lines = [l for l in text.splitlines() if not DROP.search(l.split(";")[0].split("(")[0])]
        if len([l for l in lines if l.strip()]) < 20:
            continue
        rel = os.path.relpath(src, CORPUS).replace(os.sep, "__")
        with open(os.path.join(CORPUS_OUT, rel + ".nc"), "w") as f:
            f.write("\n".join(lines) + "\n")
        kept += 1
print("generated 80, corpus kept", kept)
