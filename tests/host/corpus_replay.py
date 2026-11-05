#!/usr/bin/env python3
"""Replay files through the real parser, canonical machine, arc and planner code.

Physical execution is a logical buffer drain. See corpus_replay_support/README.md
for the configuration, numeric checks and limits of this host experiment.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import time

from source_extract import function, region

HERE = Path(__file__).resolve().parent
SUPPORT = HERE / "corpus_replay_support"
SOURCES = ("gcode_parser", "canonical_machine", "plan_arc", "planner",
           "plan_line", "plan_zoid", "util", "spindle", "runtime", "feedlimit")
SUFFIXES = {".nc", ".ngc", ".gcode", ".tap"}
MATH_NAMES = ("sqrt", "cbrt", "pow", "sin", "cos", "atan2", "atan", "asin",
              "acos", "floor", "ceil", "round", "fabs", "fmax", "fmin", "hypot")
FLOAT_TOKENS = re.compile(
    r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|'
    r'(?<![\w.])(?:(?:[0-9]+\.[0-9]*|\.[0-9]+)(?:[eE][+-]?[0-9]+)?|'
    r'[0-9]+[eE][+-]?[0-9]+)[fFlL]?|\b(?:' +
    "|".join(MATH_NAMES) + r')\s*\(', re.S)


def binary32(source):
    """AVR doubles are floats; retain comments/strings and use float math calls.

    This does not emulate AVR libm or the AVR integer-promotion rules.
    """
    def literal(match):
        token = match.group()
        if token.startswith(('//', '/*', '"', "'")) or token[-1] in "fFlL":
            return token
        if token.endswith("("):
            return token[:-1].rstrip() + "f("
        return token + "f"
    return FLOAT_TOKENS.sub(literal, source)


def extract(fw, filename, name):
    source = (fw / filename).read_text(encoding="latin1")
    signature = (r'(?:static )?(?:void|bool|char(?:\s*\*)?|nvObj_t\s*\*|'
                 r'stat_t|uint8_t)\s*' + re.escape(name) +
                 r'\([^;{}]*\)(?=\s*(?://[^\n]*\n\s*)?\{)')
    definition = function(source, signature)
    line = source[:source.index(definition)].count("\n") + 1
    return f'\n#line {line} "{fw / filename}"\n{definition}\n'


def json_chain(fw):
    """Use actual tokenization and setters with an explicitly limited table."""
    app = (fw / "config_app.c").read_text(encoding="latin1")
    rows = re.findall(r'\{\s*"[^"\n]*"\s*,\s*"[^"\n]*"[^\n{}]*\}', app)
    fields = {row.split(",")[1].strip().strip('"'): row[1:-1].split(",")
              for row in rows}
    pre = (SUPPORT / "json_prelude.h").read_text()
    for marker, token in (("@JA_SET@", "ja"), ("@VM_SET@", "xvm")):
        pre = pre.replace(marker, fields[token][6].strip())
    tokens = "static const char *firmware_tokens[] = {" + ",".join(
        json.dumps(token) for token in sorted(fields)) + "};"
    code = pre.replace("@FIRMWARE_TOKENS@", tokens)
    # N60's in-cycle refusal (config.c _refused_in_cycle): replays apply settings before streaming, so never refuse
    code += "\nstatic bool _refused_in_cycle(nvObj_t *nv) { (void)nv; return false; }\n"
    code += extract(fw, "config.c", "nv_reset_nv_list").replace(
        "nv_reset_nv_list(", "real_reset_nv_list(", 1)
    for name in ("nv_reset_nv", "nv_next", "nv_prev", "nv_next_empty", "set_nul",
                 "nv_set", "nv_get", "nv_copy_string", "get_flt", "set_flt"):
        code += extract(fw, "config.c", name)
    code += extract(fw, "config_app.c", "set_flu")
    code += extract(fw, "report.c", "sr_set_si")
    for name in ("_is_key_char", "_scan_tid", "_scan_rt", "_skip_whitespace",
                 "_skip_quote", "_value_type", "_tokenize_json_string", "_next_token",
                 "_marshall_nv_pair", "_motion_sync_envelope", "_json_parser_kernal",
                 "json_parser"):
        code += extract(fw, "json_parser.c", name)
    for name in ("_text_parser_kernal", "text_parser"):
        code += extract(fw, "text_parser.c", name)
    return code + """
void nv_print_list(stat_t s,uint8_t t,uint8_t j){}
stat_t help_general(nvObj_t *n){return STAT_OK;}
stat_t sr_run_text_status_report(void){return STAT_OK;}
"""


def build(source, output, cc="cc"):
    fw = source / "firmware/tinyg"
    output.mkdir(parents=True, exist_ok=True)
    exe = output / "replay"
    exe.unlink(missing_ok=True)  # a failed build must never reuse an old executable
    shutil.copytree(HERE / "planner_startup/avr", output / "avr", dirs_exist_ok=True)
    pgm = output / "avr/pgmspace.h"
    pgm.write_text(pgm.read_text().replace("(*(const uint16_t *)(a))", "(*(a))"))
    (output / "host_types.h").write_text(
        "typedef struct {unsigned char OUTSET,OUTCLR,OUT,IN;} PORT_t;\n"
        "typedef struct {int unused;} TC1_t;\n")
    for name in ("host_xio.h", "host_gpio.h", "driver.c"):
        shutil.copyfile(SUPPORT / name, output / name)
    for name in SOURCES:
        code = (fw / (name + ".c")).read_text(encoding="latin1")
        code = code.replace('#include "xio.h"', '#include "host_xio.h"')
        code = code.replace('#include "gpio.h"', '#include "host_gpio.h"')
        (output / (name + ".c")).write_text(binary32(code))
    stepper = (fw / "stepper.c").read_text(encoding="latin1")
    if "float st_get_move_time(" in stepper:
        # Support both the cached and original motor-rate guard implementations.
        if "static float motor_time_per_unit" in stepper:
            motor = region(stepper, r"static float motor_time_per_unit",
                           r"float st_get_move_time\(")
        else:
            motor = function(stepper, r"float st_get_move_time\(")
            motor += "\nvoid st_refresh_move_time(void){}\n"
    else:
        # This source predates the pulse cap; its planner never calls this helper.
        motor = "void st_refresh_move_time(void){}\n"
    motor = ('#include "tinyg.h"\n#include "config.h"\n'
             '#include "canonical_machine.h"\n#include "stepper.h"\n') + motor
    (output / "motor_time.c").write_text(binary32(motor))
    (output / "json_chain.c").write_text(binary32(json_chain(fw)))
    for path in fw.rglob("*.h"):
        dest = output / "headers" / path.relative_to(fw)
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(binary32(path.read_text(encoding="latin1")))
    args = [cc, "-std=gnu99", "-D__AVR=1", "-DFREQUENCY_DDA=50000",
            "-Dsnprintf_P=snprintf", "-O1", "-g", "-funsigned-char",
            "-fsanitize=address,undefined,float-cast-overflow",
            "-fno-sanitize-recover=all", "-Wno-incompatible-pointer-types",
            "-D_POSIX_C_SOURCE=200809L", "-include", "tgmath.h", "-include",
            str(output / "host_types.h"), "-I" + str(output),
            "-I" + str(output / "headers"), "-I" + str(fw)]
    if platform.system() == "Darwin":
        args += ["-Wl,-dead_strip"]
    else:
        args += ["-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections"]
    args += [str(output / (name + ".c")) for name in
             (*SOURCES, "motor_time", "json_chain", "driver")]
    args += ["-lm", "-o", str(exe)]
    result = subprocess.run(args, capture_output=True, text=True)
    (output / "build.log").write_text(result.stdout + result.stderr)
    (output / "command.json").write_text(json.dumps(args, indent=2) + "\n")
    if result.returncode:
        raise RuntimeError(f"Host build failed; see {output / 'build.log'}\n{result.stderr[-4000:]}")
    return exe


def self_test(exe):
    cases = [
        ("G21 G90 G17 G94 G92.1 G54\nF600\nG1 X10\n"
         "G2 X10 Y0 I1 J0 F100\nM3 S1000\nG1 X20\nM30\n", [0] * 7),
        ("G1 X1X2\nG1 Xnan\nG1 F-5 X1\nG99\n", [130, 102, 108, 131]),
        ('{"xjm":20}\n{"ja":100000}\n{"xvm":1}x\n$HZ\n', [0, 0, 111, 100]),
    ]
    for data, expected in cases:
        result = subprocess.run([str(exe)], input=data, text=True,
                                capture_output=True, timeout=15, check=True)
        actual = [int(line.split()[2]) for line in result.stdout.splitlines()
                  if line.startswith(("LINE ", "JSON ", "TEXT "))]
        if actual != expected or result.stderr:
            raise AssertionError((actual, expected, result.stderr))
        if any(line.startswith(("BAD ", "SLOW ", "BASE_SLOW ", "STATE "))
               for line in result.stdout.splitlines()):
            raise AssertionError(result.stdout)
    result = subprocess.run([str(exe), "--detector-selftest"], text=True,
                            capture_output=True, timeout=15, check=True)
    if "BAD " not in result.stdout or "SLOW " not in result.stdout:
        raise AssertionError("Numeric detector controls did not fire")


def replay(exe, path, output, ordinal, timeout, diagnostics):
    data = path.read_bytes()
    source_lines = data.decode("latin1").split("\n")
    if source_lines[-1] == "":
        source_lines.pop()
    source_lines = [line.rstrip("\r") for line in source_lines]
    stem = f"{ordinal:04d}-{path.name}"
    started = time.perf_counter()
    try:
        result = subprocess.run([str(exe)], input=data, capture_output=True, timeout=timeout,
                                env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0"})
    except subprocess.TimeoutExpired as error:
        result = subprocess.CompletedProcess([], 124, error.stdout or b"",
                                             (error.stderr or b"") + b"FILE_TIMEOUT\n")
    elapsed = time.perf_counter() - started
    (output / "logs" / (stem + ".out")).write_bytes(result.stdout)
    (output / "logs" / (stem + ".err")).write_bytes(result.stderr)
    row = dict(file=str(path), sha256=hashlib.sha256(data).hexdigest(),
               lines=len(source_lines), exit=result.returncode, wall_seconds=elapsed,
               statuses={}, alarms=[], findings=[], diagnostic_infinity_lines=[],
               report_stubs=[], cpu_seconds=0, blocks=0, complete=False)
    infinity = set()
    accounted = set()
    for line in result.stdout.decode(errors="replace").splitlines():
        words = line.split()
        if not words:
            continue
        tag = words[0]
        if tag == "DONE":
            row.update(complete=int(words[1]) == row["lines"], blocks=int(words[2]),
                       cpu_seconds=float(words[5]), max_profile_time_ratio=float(words[6]))
            continue
        if tag not in {"LINE", "JSON", "TEXT", "STATE", "BAD", "SLOW", "BASE_SLOW",
                       "ARC_STATUS", "NAIVE_INF", "REPORT_STUB", "UNMODELED_CONFIG"}:
            continue
        number = int(words[1])
        item = dict(file=str(path), line=number, type=tag, detail=line,
                    text=source_lines[number - 1] if 0 < number <= len(source_lines) else "<preamble>")
        if tag in {"LINE", "JSON", "TEXT"}:
            accounted.add(number)
            status = int(words[2])
            if not status:
                continue
            row["statuses"][str(number)] = status
            item["status"] = status
        elif tag == "STATE":
            row["alarms"].append(number)
        elif tag == "NAIVE_INF":
            if words[2:] == ["inf", "0", "0"]:
                if number in infinity:
                    continue
                infinity.add(number)
            else:
                row["findings"].append(item)
        elif tag == "REPORT_STUB":
            row["report_stubs"].append(number)
        else:
            row["findings"].append(item)
        diagnostics.write(json.dumps(item) + "\n")
    row["complete"] = row["complete"] and accounted == set(range(1, row["lines"] + 1))
    row["diagnostic_infinity_lines"] = sorted(infinity)
    if result.stderr:
        row["stderr"] = result.stderr.decode(errors="replace")
    if b"UNMODELED" in result.stderr or b"DRAIN_STALLED" in result.stderr:
        row["findings"].append({"type": "FIXTURE_GAP", "detail": row["stderr"]})
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", type=Path, help="files or recursively searched corpus directories")
    parser.add_argument("--source", type=Path, default=HERE.parents[1], help="firmware repository/worktree")
    parser.add_argument("--output", type=Path, required=True, help="build, logs and machine-readable reports")
    parser.add_argument("--cc", default=os.environ.get("CC", "cc"))
    parser.add_argument("--timeout", type=float, default=45, help="seconds per file; individual lines also have a 5 s watchdog")
    parser.add_argument("--self-test", action="store_true", help="run positive/negative harness controls before replay")
    parser.add_argument("--baseline", type=Path, help="previous per-file.json; report status changes on identical inputs")
    args = parser.parse_args()
    if not args.paths and not args.self_test:
        parser.error("supply files/directories or --self-test")
    output, source = args.output.resolve(), args.source.resolve()
    output.mkdir(parents=True, exist_ok=True)
    baseline = json.loads(args.baseline.read_text()) if args.baseline else None
    files = set()
    for path in args.paths:
        if path.is_dir():
            files.update(p.resolve() for p in path.rglob("*") if p.is_file() and p.suffix.lower() in SUFFIXES)
        elif path.is_file():
            files.add(path.resolve())
        else:
            parser.error(f"missing input: {path}")
    if args.paths and not files:
        parser.error("no G-code files found")
    exe = build(source, output / "build", args.cc)
    if args.self_test:
        self_test(exe)
        print("Harness controls passed", flush=True)
    (output / "logs").mkdir(exist_ok=True)
    started = time.perf_counter()
    with (output / "diagnostics.jsonl").open("w") as diagnostics:
        rows = []
        for i, path in enumerate(sorted(files)):
            row = replay(exe, path, output, i, args.timeout, diagnostics)
            rows.append(row)
            print(f"{i+1}/{len(files)} {path.name}: exit={row['exit']} "
                  f"refusals={len(row['statuses'])} findings={len(row['findings'])}", flush=True)
    elapsed = time.perf_counter() - started
    status_changes = []
    if baseline is not None:
        old = {(Path(r["file"]).name, r["sha256"]): r for r in baseline}
        for row in rows:
            before = old.get((Path(row["file"]).name, row["sha256"]))
            if before is None:
                status_changes.append({"file": row["file"], "change": "no identical baseline input"})
            elif before["statuses"] != row["statuses"]:
                status_changes.append({"file": row["file"], "before": before["statuses"], "after": row["statuses"]})
    head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=source, capture_output=True, text=True)
    dirty = subprocess.run(["git", "status", "--porcelain"], cwd=source, capture_output=True, text=True)
    total = sum(row["lines"] for row in rows)
    summary = dict(source=str(source), commit=head.stdout.strip(), source_dirty=bool(dirty.stdout),
                   files=len(rows), lines=total, blocks=sum(row["blocks"] for row in rows),
                   wall_seconds=elapsed, lines_per_second=total / elapsed,
                   cpu_seconds=sum(row["cpu_seconds"] for row in rows),
                   failed_files=sum(bool(row["exit"] or not row["complete"]) for row in rows),
                   finding_count=sum(len(row["findings"]) for row in rows),
                   alarm_files=sum(bool(row["alarms"]) for row in rows),
                   refusal_count=sum(len(row["statuses"]) for row in rows),
                   diagnostic_infinity_lines=sum(len(row["diagnostic_infinity_lines"]) for row in rows),
                   status_changes=status_changes, host=platform.platform())
    (output / "per-file.json").write_text(json.dumps(rows, indent=2) + "\n")
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    # Foreign dialect refusals require interpretation; they are not automatically bugs.
    return int(bool(summary["failed_files"] or summary["finding_count"] or
                    summary["alarm_files"] or status_changes))


if __name__ == "__main__":
    sys.exit(main())
