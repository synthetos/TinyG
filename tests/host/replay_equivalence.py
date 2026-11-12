"""Behaviour-neutral refactor gate: the segment stream of two firmware trees must be bit-identical.

    python3 -I replay_equivalence.py --base <tree> --cand <tree> --work <dir> [--corpus extra.nc ...]

Builds the trajectory replay (real planner, exec and segment prep) for both trees and replays a fixed corpus
through each: every prepared segment's time, per-motor steps, exec segment velocity, block jerk, line and section,
plus the final step counters. Any difference is printed with its first diverging line and fails (exit 1).
Used for the RAM reclamation steps ahead of the live feed override (feedoverride brief, 2026-11-09).
"""
from pathlib import Path
import argparse
import hashlib
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import trajectory_replay as tr  # noqa: E402

HERE = Path(__file__).resolve().parent

PROGRAMS = {
    "long_straight": "G21 G90 G94\nN10 G1 X100 F1200\n",
    "square": "G21 G90 G94\nN10 G1 X20 F1000\nN20 G1 Y20\nN30 G1 X0\nN40 G1 Y0\n",
    "zigzag": "G21 G90 G94\n" + "".join(f"N{i} G1 X{(i % 2) * 5} Y{i * 0.5} F1200\n" for i in range(1, 40)),
    "dense": "G21 G90 G94\n" + "".join(f"N{i} G1 X{i * 0.08:.3f} Y{(i % 7) * 0.01:.3f} F1200\n" for i in range(1, 400)),
    "arc": "G21 G90 G94 G17\nN10 G1 X10 F800\nN20 G2 X10 Y0 I-5 J0\n",
    "helix": "G21 G90 G94 G17\nN10 G1 X5 F600\nN20 G3 X5 Y0 Z-2 I-5 J0\n",
    "rapids_feeds": "G21 G90 G94\nN10 G0 X30\nN20 G1 X35 F300\nN30 G0 X0\nN40 G1 X5 Z-1 F200\nN50 G0 Z5\n",
    "outputs_dwell": "G21 G90 G94\nN10 M3 S1000\nN20 G1 X10 F900\nN30 M8\nN40 G4 P0.2\nN50 G1 X20\nN60 M9\nN70 M5\nN80 G1 X0\n",
    "inches": "G20 G90 G94\nN10 G1 X1 F40\nN20 G1 Y1\nN30 G21\nN40 G1 X0 Y0 F1000\n",
}
CORPUS = [HERE.parents[1] / "tests/hil/corpus/n39/dense-isolation-excerpt.nc"]


def stream(exe, gcode, batch):
    segs, steps = tr.run(exe, gcode, batch=batch)
    lines = [" ".join(repr(x) for x in (s[0], *s[1], s[2], s[3], s[4], s[5])) for s in segs]
    lines += [" ".join(r) for r in steps]
    return lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", type=Path, required=True)
    ap.add_argument("--cand", type=Path, required=True)
    ap.add_argument("--work", type=Path, required=True)
    ap.add_argument("--corpus", type=Path, nargs="*", default=[])
    a = ap.parse_args()
    exes = {}
    for tag, tree in (("base", a.base), ("cand", a.cand)):
        out = a.work / tag
        out.mkdir(parents=True, exist_ok=True)
        exes[tag] = tr.build(tree.resolve(), out)
    cases = dict(PROGRAMS)
    for f in CORPUS + a.corpus:
        if f.exists():
            cases[f.name] = f.read_text(errors="replace")
    failed = 0
    for name, gcode in cases.items():
        for batch in (4, 28):
            try:
                b, c = stream(exes["base"], gcode, batch), stream(exes["cand"], gcode, batch)
            except RuntimeError as error:
                failed += 1
                print(f"ERROR {name} batch={batch}: {error}")
                continue
            hb = hashlib.sha256("\n".join(b).encode()).hexdigest()[:16]
            if b == c:
                print(f"SAME {name} batch={batch} segs={len(b)} sha={hb}")
                continue
            failed += 1
            first = next((i for i, (x, y) in enumerate(zip(b, c)) if x != y), min(len(b), len(c)))
            print(f"DIFF {name} batch={batch} base={len(b)} cand={len(c)} first={first}")
            print(f"   base: {b[first] if first < len(b) else '<end>'}")
            print(f"   cand: {c[first] if first < len(c) else '<end>'}")
    print("EQUIVALENT" if not failed else f"NOT EQUIVALENT ({failed} case(s))")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
