"""Line classification and footer parsing for TinyG serial output (440 and 449 dialects)."""
import json
import re
from dataclasses import dataclass, field
from typing import Any

XON, XOFF = 0x11, 0x13
PROMPT = re.compile(r"^tinyg \[(mm|in)\] ")
_BARE_KEY = re.compile(r'([{,])\s*([A-Za-z_][A-Za-z0-9_]*)\s*:')


@dataclass
class Event:
    kind: str           # response, banner, status, queue, rx, exception, ack, prompt, text, malformed
    t: float            # host monotonic time the line completed
    raw: str
    data: Any = None    # parsed JSON object (None for text)
    damaged: str | None = None  # the line as received, when `raw` is a JSON object salvaged from its tail

    @property
    def body(self):
        if self.kind in ("response", "banner") and "r" in self.data:
            return self.data["r"]
        if self.kind in ("status", "banner") and "sr" in self.data:
            return self.data["sr"]
        return self.data


def checksum_440(line: str) -> int:
    """Footer checksum used by footer revision 1: h = 31*h + c (uint32) over the line up to the last comma."""
    h = 0
    for c in line[:line.rindex(",")].encode():
        h = (31 * h + c) & 0xFFFFFFFF
    return h % 9999


def loads(raw: str):
    """Parse strict JSON, falling back to TinyG relaxed syntax (bare keys)."""
    try:
        return json.loads(raw)
    except ValueError:
        return json.loads(_BARE_KEY.sub(r'\1"\2":', raw))


def _salvage(raw: str):
    """A line whose start was mangled (an Atmel-ICE halt can freeze the USART mid-byte while the board
    transmits) may still end with a whole JSON object; return the last one that parses, or None."""
    for i in range(len(raw) - 1, 0, -1):
        if raw[i] == "{" and raw[i:i + 2] == '{"':
            try:
                data = loads(raw[i:])
            except ValueError:
                continue
            if isinstance(data, dict) and data:
                return raw[i:], data
    return None


def parse_line(raw: str, t: float) -> Event:
    damaged = None
    if raw.startswith("\0") and raw.lstrip("\0").startswith("{"):
        # Leading NUL bytes only (seen once in regression 95f797f0, cause unconfirmed; an Atmel-ICE halt
        # mid-byte is the suspect): keep the JSON after them, marked damaged. Embedded NULs stay malformed.
        damaged, raw = raw, raw.lstrip("\0")
    if raw.startswith("{"):
        try:
            data = loads(raw)
        except ValueError:
            found = _salvage(raw)
            if found is None:
                return Event("malformed", t, damaged or raw)
            damaged, (raw, data) = damaged or raw, found
        if not isinstance(data, dict):
            return Event("malformed", t, raw, damaged=damaged)
        if "r" in data or "f" in data:
            r = data.get("r")
            if isinstance(r, dict) and r.get("msg") == "SYSTEM READY":
                return Event("banner", t, raw, data, damaged)
            return Event("response", t, raw, data, damaged)
        if "sr" in data:
            sr = data["sr"]
            return Event("banner" if isinstance(sr, dict) and "msg" in sr else "status", t, raw, data, damaged)
        for key, kind in (("qr", "queue"), ("rx", "rx"), ("er", "exception"), ("ack", "ack")):
            if key in data:
                return Event(kind, t, raw, data, damaged)
        return Event("malformed", t, raw, data, damaged)
    if PROMPT.match(raw):
        return Event("prompt", t, raw)
    return Event("text", t, raw)


@dataclass
class Footer:
    rev: int
    status: int
    count: int                  # linelen (rev 1/2) or line buffers free (rev 3)
    checksum_ok: bool | None    # rev 1 only

    @classmethod
    def from_event(cls, ev: Event):
        f = ev.data.get("f")
        if not isinstance(f, list) or len(f) < 3:
            return None
        ok = (checksum_440(ev.raw) == f[3]) if f[0] == 1 and len(f) > 3 else None
        return cls(f[0], f[1], f[2], ok)


@dataclass
class Response:
    sent: str
    event: Event
    footer: Footer | None
    events: list = field(default_factory=list)   # asynchronous lines seen while waiting
    elapsed: float = 0.0

    @property
    def body(self) -> dict:
        r = self.event.data.get("r")
        return r if isinstance(r, dict) else {}

    @property
    def status(self) -> int | None:
        return self.footer.status if self.footer else None

    @property
    def raw(self) -> str:
        return self.event.raw

    def text(self) -> list[str]:
        return [e.raw for e in self.events if e.kind == "text"]
