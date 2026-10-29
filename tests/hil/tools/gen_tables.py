"""Generate the token manifest and status-code table for one firmware source tree.

usage: python tools/gen_tables.py REF NAME
  REF   a git ref (e.g. origin/edge-0.97, HEAD) whose firmware/tinyg/ is read
  NAME  output name; writes tinyg_hil/data/NAME.json

The cfgArray is read from the avr-gcc preprocessed config_app.c so that #if'd
rows are handled exactly as the firmware build does. Requires git and avr-gcc.
"""
import json
import pathlib
import re
import subprocess
import sys
import tarfile
import tempfile
import io

HERE = pathlib.Path(__file__).resolve().parent
CFLAGS = ["-mmcu=atxmega192a3", "-DF_CPU=32000000UL", "-std=gnu99"]

ROW = re.compile(r'\{\s*"([^"]*)"\s*,\s*"([^"]+)"\s*,\s*([^,]+?)\s*,\s*(\d+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,')
INT_GETTERS = {"get_ui8", "get_int", "get_int8", "get_u32", "get_data"}


def git_show(ref, path):
    return subprocess.run(["git", "show", f"{ref}:{path}"], check=True, capture_output=True,
                          text=True).stdout.replace("\r", "")


def flag_value(expr, persist_bit):
    expr = re.sub(r"\bTYPE_\w+\b", "0", expr)
    return int(eval(expr, {"__builtins__": {}}))  # expression of hex literals and '|'


def row_type(expr, getter):
    m = re.search(r"\bTYPE_(\w+)\b", expr)
    if m:
        return m.group(1).lower()
    if getter == "get_flt":
        return "float"
    if getter in INT_GETTERS:
        return "integer"
    return "other"


def main(ref, name):
    root = subprocess.run(["git", "rev-parse", "--show-toplevel"], check=True, capture_output=True,
                          text=True).stdout.strip()
    sha = subprocess.run(["git", "rev-parse", "--short", ref], check=True, capture_output=True,
                         text=True, cwd=root).stdout.strip()
    config_h = git_show(ref, "firmware/tinyg/config.h")
    persist_bit = int(re.search(r"#define\s+F_PERSIST\s+(0x[0-9a-fA-F]+)", config_h).group(1), 16)

    with tempfile.TemporaryDirectory() as tmp:
        archive = subprocess.run(["git", "archive", ref, "firmware/tinyg"], check=True,
                                 capture_output=True, cwd=root).stdout
        tarfile.open(fileobj=io.BytesIO(archive)).extractall(tmp, filter="data")
        src = pathlib.Path(tmp, "firmware/tinyg")
        pre = subprocess.run(["avr-gcc", "-E", "-P", *CFLAGS, "config_app.c"], check=True,
                             capture_output=True, text=True, cwd=src).stdout
        status_src = (src / "error.h") if (src / "error.h").exists() else (src / "tinyg.h")
        status_text = status_src.read_text(errors="replace")
        tinyg_h = (src / "tinyg.h").read_text(errors="replace")

    tokens = {}
    for grp, tok, flags, prec, _printer, getter, setter in ROW.findall(pre):
        value = flag_value(flags, persist_bit)
        tokens[tok] = {"group": grp, "type": row_type(flags, getter), "precision": int(prec),
                       "persist": bool(value & persist_bit), "get": getter, "set": setter}

    status = {}
    for sym, code in re.findall(r"^#define\s+(STAT_\w+)\s+(\d+)", status_text, re.M):
        status.setdefault(sym[len("STAT_"):], int(code))

    fb = float(re.search(r"#define\s+TINYG_FIRMWARE_BUILD\s+([\d.]+)", tinyg_h).group(1))
    out = {"name": name, "ref": ref, "commit": sha, "fb": fb, "status": status, "tokens": tokens}
    path = HERE.parent / "tinyg_hil" / "data" / f"{name}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=1, sort_keys=True) + "\n")
    persisted = sum(t["persist"] for t in tokens.values())
    print(f"{path.name}: {sha} fb {fb}, {len(tokens)} tokens ({persisted} persisted), {len(status)} status codes")


if __name__ == "__main__":
    main(*sys.argv[1:3])
