"""Firmware line and function coverage of the host replay (real parser, canonical machine, planner, exec, prep).

    python3 -I replay_coverage.py --out DIR [--source TREE] FILE_OR_DIR [...]

Builds the trajectory replay, recompiles it with clang source-based coverage, replays every G-code file given (dirs:
*.nc *.ngc *.gcode *.tap, recursively), merges the profiles and writes DIR/coverage.json and DIR/coverage.md:
per firmware module the line and function coverage, every function never executed, and the largest blocks of
never-executed lines in executed functions. Host replay only reaches what the replay compiles (gcode_parser,
canonical_machine, plan_*, planner, util, spindle, runtime, feedlimit, plan_exec, kinematics, stepper prep);
xio, switches, hardware and the ISRs are bench territory. The point: aim new tests at code nothing reaches.
"""
from pathlib import Path
import argparse
import json
import os
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import trajectory_replay as tr  # noqa: E402

EXTS = {".nc", ".ngc", ".gcode", ".tap"}
FIRMWARE = {m + ".c" for m in ("gcode_parser", "canonical_machine", "plan_arc", "planner", "plan_line", "plan_zoid",
                               "util", "spindle", "runtime", "feedlimit", "plan_exec", "kinematics")}


def xcrun(tool):
    return subprocess.run(["xcrun", "--find", tool], capture_output=True, text=True, check=True).stdout.strip()


def build_cov(root, out):
    out.mkdir(parents=True, exist_ok=True)
    exe = tr.build(Path(root).resolve(), out)
    args = json.loads((out / "command.json").read_text())
    args[args.index("-lm"):args.index("-lm")] = [str(out / (n + ".c")) for n in ["plan_exec", "kinematics", "step_engine"]]
    args.insert(1, "-DF_CPU=32000000UL")
    o = args.index("-o")
    args[o + 1] = str(out / "replay_cov")
    args[1:1] = ["-fprofile-instr-generate", "-fcoverage-mapping"]
    p = subprocess.run(args, capture_output=True, text=True)
    (out / "coverage_build.log").write_text(p.stdout + p.stderr)
    if p.returncode:
        raise RuntimeError(p.stderr[-3000:])
    return out / "replay_cov", exe


def inputs(paths):
    for p in paths:
        p = Path(p)
        if p.is_dir():
            yield from sorted(q for q in p.rglob("*") if q.suffix.lower() in EXTS and q.is_file())
        elif p.is_file():
            yield p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--source", type=Path, default=Path(__file__).resolve().parents[2])
    ap.add_argument("--timeout", type=float, default=300)
    a = ap.parse_args()
    out = a.out.resolve()
    exe, _ = build_cov(a.source, out / "build")
    prof = out / "profraw"
    prof.mkdir(exist_ok=True)
    for old in prof.glob("*.profraw"):
        old.unlink()
    files = list(inputs(a.paths))
    ran = failed = 0
    for k, f in enumerate(files):
        env = dict(os.environ, LLVM_PROFILE_FILE=str(prof / f"{k:05d}.profraw"))
        try:
            subprocess.run([str(exe), "28"], input=f.read_bytes(), capture_output=True, timeout=a.timeout, env=env)
            ran += 1
        except subprocess.TimeoutExpired:
            failed += 1
            print(f"timeout: {f}", file=sys.stderr)
    data = out / "merged.profdata"
    subprocess.run([xcrun("llvm-profdata"), "merge", "-sparse", "-o", str(data)] + [str(p) for p in prof.glob("*.profraw")],
                   check=True)
    exp = json.loads(subprocess.run([xcrun("llvm-cov"), "export", str(exe), f"-instr-profile={data}"],
                                    capture_output=True, text=True, check=True).stdout)
    report(exp, out, ran, failed, len(files))


def report(exp, out, ran, failed, total):
    build = str(out / "build")
    d = exp["data"][0]
    mods, never, gaps = {}, [], []
    for f in d["files"]:
        name = Path(f["filename"]).name
        if not f["filename"].startswith(build) or name not in FIRMWARE:
            continue
        s = f["summary"]
        mods[name] = {"lines": s["lines"]["count"], "lines_hit": s["lines"]["covered"],
                      "functions": s["functions"]["count"], "functions_hit": s["functions"]["covered"]}
    for fn in d["functions"]:
        fname = Path(fn["filenames"][0]).name
        if fname not in mods:
            continue
        line0 = fn["regions"][0][0] if fn["regions"] else 0
        if fn["count"] == 0:
            never.append({"file": fname, "function": fn["name"], "line": line0,
                          "lines": (fn["regions"][0][2] - line0 + 1) if fn["regions"] else 0})
            continue
        # the largest run of never-executed code regions inside an executed function
        zero = [r for r in fn["regions"] if r[4] == 0 and r[7] == 0]          # count 0, code region
        for r in zero:
            if r[2] - r[0] >= 3:
                gaps.append({"file": fname, "function": fn["name"], "from": r[0], "to": r[2], "lines": r[2] - r[0] + 1})
    never.sort(key=lambda x: (-x["lines"], x["file"]))
    gaps.sort(key=lambda x: -x["lines"])
    res = {"files_replayed": ran, "timeouts": failed, "files_total": total, "modules": mods, "never_executed": never,
           "largest_unexecuted_blocks": gaps[:60]}
    (out / "coverage.json").write_text(json.dumps(res, indent=1))
    md = [f"# Host replay coverage\n\n{ran} of {total} files replayed ({failed} timeouts).\n",
          "| module | lines | functions |", "|---|---|---|"]
    for m, v in sorted(mods.items(), key=lambda kv: kv[1]["lines_hit"] / max(1, kv[1]["lines"])):
        md.append(f"| {m} | {v['lines_hit']}/{v['lines']} ({100 * v['lines_hit'] / max(1, v['lines']):.0f}%) | "
                  f"{v['functions_hit']}/{v['functions']} |")
    md.append("\n## Functions never executed\n")
    md += [f"- {x['file']}:{x['line']} `{x['function']}` ({x['lines']} lines)" for x in never]
    md.append("\n## Largest never-executed blocks inside executed functions\n")
    md += [f"- {x['file']}:{x['from']}-{x['to']} in `{x['function']}` ({x['lines']} lines)" for x in gaps[:60]]
    (out / "coverage.md").write_text("\n".join(md) + "\n")
    print("\n".join(md[:20]))


if __name__ == "__main__":
    main()
