"""Settings backup and restore through the token interface (works across builds)."""
import datetime
import json
import pathlib
import re

from .driver import StatusError, TinyG

IDENTITY = ("fb", "fv", "hp", "hv", "id")
NEVER_WRITE = set(IDENTITY) | {"baud"}
# se00-se29 hold the status-report list as raw cfgArray row numbers, which differ between builds,
# and writing them stores the low 16 bits of the float (E140). The list is saved by name via "sr".
INTERNAL = re.compile(r"^se\d\d$")
# Communication settings change how the board answers, so they are written last, jv very last.
COMM_ORDER = ["js", "ee", "ec", "ej", "ex", "rxm", "jv"]


def _units_mm(tg: TinyG):
    # F_CONVERT settings read and write in the current units; snapshots are always in mm.
    tg.send_gcode("G21", check=True)


def take(tg: TinyG) -> dict:
    _units_mm(tg)
    tg.wait_idle()
    values, missing = {}, []
    unrecognized = tg.dialect.status("UNRECOGNIZED_NAME")
    for token in tg.dialect.persisted_tokens():
        if token in IDENTITY or INTERNAL.match(token):
            continue
        try:
            values[token] = tg.get(token)
        except StatusError as e:
            if e.response.status != unrecognized:
                raise
            missing.append(token)
    sr = tg.get("sr")
    return {"identity": dict(tg.identity), "dialect": tg.dialect.name,
            "taken": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
            "values": values, "sr": list(sr) if isinstance(sr, dict) else [], "missing": missing}


def save(snap: dict, directory) -> pathlib.Path:
    directory = pathlib.Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    ident = snap["identity"]
    stamp = snap["taken"].replace(":", "").replace("+0000", "Z")
    path = directory / f"{ident['id']}-{ident['fb']}-{stamp}.json"
    path.write_text(json.dumps(snap, indent=1) + "\n")
    return path


def load(path) -> dict:
    return json.loads(pathlib.Path(path).read_text())


def _typed(tg: TinyG, token: str, value):
    kind = tg.dialect.tokens[token]["type"]
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if kind in ("integer", "boolean", "data"):
            return int(round(value))
        if kind == "float":
            return float(value)
    return value


def restore(tg: TinyG, snap: dict) -> dict:
    """Write back every snapshot value that differs on the board. Returns what changed/dropped/failed."""
    _units_mm(tg)
    tg.wait_idle()
    result = {"changed": [], "dropped": [], "failed": []}
    tokens = tg.dialect.tokens
    values = snap["values"]
    for token in [t for t in values if t not in COMM_ORDER]:
        _restore_one(tg, token, values[token], result)
    if snap.get("sr"):
        current = tg.get("sr")
        if not isinstance(current, dict) or list(current) != snap["sr"]:
            set_status_report(tg, snap["sr"])
            result["changed"].append("sr")
    # jv is last in COMM_ORDER: it can make later responses terse, so nothing is read after it
    for token in [t for t in COMM_ORDER if t in values]:
        _restore_one(tg, token, values[token], result)
    return result


def _restore_one(tg: TinyG, token: str, value, result: dict):
    tokens = tg.dialect.tokens
    if token in NEVER_WRITE or INTERNAL.match(token):
        return
    if token not in tokens or not tokens[token]["persist"]:
        result["dropped"].append(token)
        return
    want = _typed(tg, token, value)
    try:
        if tg.values_equal(token, tg.get(token), want):
            return
    except StatusError:
        pass
    for attempt in range(3):            # an EEPROM write can drop serial input: verify, retry
        try:
            tg.set(token, want, allow="settings restore")
        except StatusError as e:
            result["failed"].append((token, want, e.response.status))
            return
        if token == "jv" or tg.values_equal(token, tg.get(token), want):
            result["changed"].append(token)
            return
    result["failed"].append((token, want, "readback mismatch"))


def set_status_report(tg: TinyG, keys: list):
    """Set the status-report list by name. 449 rejects JSON input over 128 chars, so it gets relaxed syntax."""
    if tg.dialect.name == "d449":
        line = "{sr:{" + ",".join(f"{k}:t" for k in keys) + "}}"
    else:
        line = json.dumps({"sr": {k: True for k in keys}}, separators=(",", ":"))
    tg.send_json(line, check=True)
