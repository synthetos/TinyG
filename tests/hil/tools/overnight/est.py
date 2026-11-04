#!/usr/bin/env python3
"""Estimated run time of a G-code file in minutes (no acceleration; G0 at 16 m/min; G93 by F). usage: est.py FILE"""
import math, re, sys
x = y = z = a = 0.0; f = 0.0; mode = "G0"; t = 0.0; g93 = False
for raw in open(sys.argv[1], errors="replace"):
    s = raw.split("(")[0].split(";")[0].strip().upper()
    if not s or s[0] in "{$%!~": continue
    for g in re.findall(r"G0*([0-3])\b", s): mode = "G" + g
    g93 = "G93" in s or (g93 and "G94" not in s)
    w = {k: float(v) for k, v in re.findall(r"([XYZAF])(-?[0-9.]+)", s)}
    f = w.get("F", f)
    if not any(k in w for k in "XYZA"): continue
    nx, ny, nz, na = w.get("X", x), w.get("Y", y), w.get("Z", z), w.get("A", a)
    d, da = math.dist((x, y, z), (nx, ny, nz)), abs(na - a)
    if mode == "G0": t += max(d / (16000 / 60), da / (3600 / 60))
    elif g93: t += 60 / f if f else 1
    else: t += max(d, da) / (f / 60 if f else 0.001)
    x, y, z, a = nx, ny, nz, na
print(f"{t / 60:.1f}")
