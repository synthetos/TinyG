"""Reference G-code path simulator: where a block sequence should move, per RS274/NGC (NIST).

This is the "expected" side of path comparisons. It models the geometry only: G0/G1/G2/G3
(IJK and R arcs, full circles), G17/G18/G19, G20/G21, G90/G91, G90.1/G91.1, G92/G92.1,
G10 L2/L20, G54-G59, G28.3 and G43.1/G49. Feed rate, acceleration and timing are ignored.
Positions are work coordinates in mm unless the caller asks for machine coordinates.
"""
import math
import re
from dataclasses import dataclass, field

AXES = "XYZABC"
_COMMENT = re.compile(r"\([^)]*\)|;.*$")
_WORD = re.compile(r"([A-Z])\s*([-+]?\d*\.?\d+)")
# arc plane: (first axis, second axis, normal axis); G2 is clockwise viewed from +normal
PLANES = {17: ("X", "Y", "Z"), 18: ("Z", "X", "Y"), 19: ("Y", "Z", "X")}
OFFSET_INDEX = {"I": "X", "J": "Y", "K": "Z"}


@dataclass
class Segment:
    kind: str                   # "rapid", "feed", "arc"
    points: list                # machine coordinates, dicts axis -> mm
    line: str
    offset: dict = field(default_factory=dict)   # work offset in force for this segment


@dataclass
class Machine:
    position: dict = field(default_factory=lambda: {a: 0.0 for a in AXES})    # machine coords, mm
    coord_offsets: dict = field(default_factory=lambda: {n: {a: 0.0 for a in AXES} for n in range(1, 7)})
    coord: int = 1              # 1 = G54
    g92: dict = field(default_factory=lambda: {a: 0.0 for a in AXES})
    tool_offset: dict = field(default_factory=lambda: {a: 0.0 for a in AXES})
    inches: bool = False
    absolute: bool = True
    arc_absolute: bool = False  # G90.1
    plane: int = 17
    motion: int | None = None
    segments: list = field(default_factory=list)
    errors: list = field(default_factory=list)

    def offset(self, axis):
        return self.coord_offsets[self.coord][axis] + self.g92[axis] + self.tool_offset[axis]

    def work(self, axis=None):
        if axis:
            return self.position[axis] - self.offset(axis)
        return {a: self.position[a] - self.offset(a) for a in AXES}

    def _mm(self, value, axis="X"):
        return value * 25.4 if self.inches and axis in "XYZIJKR" else value

    def run(self, lines):
        for line in lines:
            self.block(line)
        return self

    def block(self, line):
        words = [(k, float(v)) for k, v in _WORD.findall(_COMMENT.sub("", line.upper()))]
        gs = [v for k, v in words if k == "G"]
        vals = {k: v for k, v in words if k not in "GMN"}
        axis_words = {a: vals[a] for a in AXES if a in vals}

        for g in gs:                                    # modal state first, as the standard orders it
            if g in (20, 21):
                self.inches = g == 20
            elif g in (17, 18, 19):
                self.plane = int(g)
            elif g in (90, 91):
                self.absolute = g == 90
            elif g == 90.1 or g == 91.1:
                self.arc_absolute = g == 90.1
            elif 54 <= g <= 59:
                self.coord = int(g) - 53
            elif g == 49:
                self.tool_offset = {a: 0.0 for a in AXES}
        non_modal = next((g for g in gs if g in (10, 28.3, 92, 92.1, 43.1)), None)
        if non_modal == 10:
            self._g10(vals, axis_words)
            return
        if non_modal == 28.3:
            for a, v in axis_words.items():
                self.position[a] = self._mm(v, a)
            return
        if non_modal == 92:
            for a, v in axis_words.items():
                self.g92[a] = self.position[a] - self.coord_offsets[self.coord][a] - self.tool_offset[a] - self._mm(v, a)
            return
        if non_modal == 92.1:
            self.g92 = {a: 0.0 for a in AXES}
            return
        if non_modal == 43.1:
            for a, v in axis_words.items():
                self.tool_offset[a] = self._mm(v, a)
            return
        for g in gs:
            if g in (0, 1, 2, 3, 80):
                self.motion = None if g == 80 else int(g)
        if not axis_words and self.motion in (0, 1):
            return
        if self.motion is None or (not axis_words and not any(k in vals for k in "IJKR")):
            return
        target = dict(self.position)
        for a, v in axis_words.items():
            v = self._mm(v, a)
            target[a] = (self.position[a] + v) if not self.absolute else (v + self.offset(a))
        if self.motion in (0, 1):
            self.segments.append(Segment("rapid" if self.motion == 0 else "feed", [dict(self.position), target], line,
                                         {a: self.offset(a) for a in AXES}))
        else:
            pts = self._arc(target, vals, line)
            if pts is None:
                return
            self.segments.append(Segment("arc", pts, line, {a: self.offset(a) for a in AXES}))
        self.position = target

    def _g10(self, vals, axis_words):
        p = int(vals.get("P", 0)) or self.coord
        l = int(vals.get("L", 2))
        for a, v in axis_words.items():
            v = self._mm(v, a)
            if l == 2:
                self.coord_offsets[p][a] = v
            elif l == 20:
                self.coord_offsets[p][a] = self.position[a] - self.g92[a] - self.tool_offset[a] - v

    def _arc(self, target, vals, line):
        a1, a2, an = PLANES[self.plane]
        s1, s2 = self.position[a1], self.position[a2]
        e1, e2 = target[a1], target[a2]
        cw = self.motion == 2
        if "R" in vals:
            r = self._mm(vals["R"], "R")
            d = math.hypot(e1 - s1, e2 - s2)
            if d == 0 or abs(r) < d / 2 - 1e-9:
                self.errors.append(f"{line}: bad R arc")
                return None
            h = math.sqrt(max(r * r - d * d / 4, 0.0))
            m1, m2 = (s1 + e1) / 2, (s2 + e2) / 2
            u1, u2 = -(e2 - s2) / d, (e1 - s1) / d          # unit normal, left of travel
            # R>0 takes the short arc; the centre is right of travel for G2, left for G3
            side = (-1 if cw else 1) * (1 if r > 0 else -1)
            c1, c2 = m1 + side * h * u1, m2 + side * h * u2
        else:
            offs = {}
            for letter, axis in OFFSET_INDEX.items():
                if letter in vals:
                    offs[axis] = self._mm(vals[letter], axis)
            o1, o2 = offs.get(a1, 0.0), offs.get(a2, 0.0)
            if self.arc_absolute:
                c1, c2 = o1 + self.offset(a1), o2 + self.offset(a2)
            else:
                c1, c2 = s1 + o1, s2 + o2
        radius = math.hypot(s1 - c1, s2 - c2)
        t0 = math.atan2(s2 - c2, s1 - c1)
        t1 = math.atan2(e2 - c2, e1 - c1)
        sweep = t1 - t0
        if cw:
            while sweep >= 0:
                sweep -= 2 * math.pi
        else:
            while sweep <= 0:
                sweep += 2 * math.pi
        if abs(abs(sweep) - 2 * math.pi) < 1e-9 and (abs(e1 - s1) > 1e-9 or abs(e2 - s2) > 1e-9):
            sweep = math.copysign(sweep % (2 * math.pi), sweep)
        n = max(8, int(abs(sweep) * radius / 0.1))      # ~0.1 mm chords
        pts = []
        for i in range(n + 1):
            f = i / n
            t = t0 + sweep * f
            p = dict(self.position)
            for a in AXES:
                if a not in (a1, a2):
                    p[a] = self.position[a] + (target[a] - self.position[a]) * f   # helix / linear axes
            p[a1], p[a2] = c1 + radius * math.cos(t), c2 + radius * math.sin(t)
            pts.append(p)
        return pts


def simulate(lines, start=None, **state) -> Machine:
    """Run blocks from an optional starting machine position; returns the Machine (segments, errors)."""
    m = Machine(**state)
    if start:
        m.position.update(start)
    return m.run(lines)


def path(machine: Machine, work=True) -> list:
    """Flattened list of points; work coordinates by default."""
    out = []
    for seg in machine.segments:
        for p in seg.points:
            out.append({a: p[a] - seg.offset[a] for a in AXES} if work else dict(p))
    return out
