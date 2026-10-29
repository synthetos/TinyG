"""Motion guard: refuse to send anything that could move a machine or switch outputs unless allowed.

The harness also runs on real machines, so motion is opt-in (--allow-motion) and individual
calls that rely on the firmware rejecting a command pass allow="<reason>".
"""
import re

AXES = set("XYZABC")
# G-codes whose axis words set values instead of moving
NON_MOTION_AXIS_G = {"10", "92", "28.1", "30.1", "28.3", "43.1"}
MOTION_G = {"28", "30", "28.2", "28.4", "38.2", "38.3", "38.4", "38.5", "53"}
OUTPUT_M = {"3", "4", "6", "7", "8"}
GUARDED_KEYS = {"test", "home", "defa", "baud", "net", "ex"}

_COMMENT = re.compile(r"\([^)]*\)|;.*$")
_WORD = re.compile(r"([A-Z])\s*([-+]?\d*\.?\d+)")


class MotionBlocked(Exception):
    pass


def _num(value: str) -> str:
    """Normalize a G/M number: '01' -> '1', '28.10' -> '28.1'."""
    f = float(value)
    return str(int(f)) if f == int(f) else f"{f:g}"


def gcode_risks(line: str) -> list[str]:
    words = _WORD.findall(_COMMENT.sub("", line.upper()))
    gs = {_num(v) for k, v in words if k == "G"}
    ms = {_num(v) for k, v in words if k == "M"}
    risks = []
    if any(k in AXES for k, _ in words) and not (gs & NON_MOTION_AXIS_G):
        risks.append("axis words")
    risks += [f"G{g}" for g in sorted(gs & MOTION_G)]
    risks += [f"M{m}" for m in sorted(ms & OUTPUT_M)]
    return risks


def json_risks(obj) -> list[str]:
    risks = []
    if isinstance(obj, dict):
        for key, value in obj.items():
            if key in GUARDED_KEYS and value is not None:
                risks.append(f"{key} set")
            if key in ("gc", "txt") and isinstance(value, str):
                risks += gcode_risks(value)
    return risks


class Guard:
    def __init__(self, allow_motion: bool = False):
        self.allow_motion = allow_motion

    def check_gcode(self, line: str, allow: str | None = None):
        risks = gcode_risks(line)
        if risks and not (self.allow_motion or allow):
            raise MotionBlocked(f"{line!r}: {', '.join(risks)} (use --allow-motion or allow=...)")

    def check_json(self, obj, allow: str | None = None):
        risks = json_risks(obj)
        if risks and not (self.allow_motion or allow):
            raise MotionBlocked(f"{obj!r}: {', '.join(risks)} (use --allow-motion or allow=...)")
