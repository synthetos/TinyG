"""TinyG driver: one request at a time over the serial transport (ping-pong)."""
import json
import re
import math
import time

from . import dialect as dialects
from .protocol import Event, Footer, Response
from .safety import Guard
from .transport import Transport

ETX_RESET = b"\x18"     # ^X: watchdog reset
ENQ = b"\x05"


class StatusError(Exception):
    def __init__(self, response: Response, name: str):
        self.response = response
        super().__init__(f"{response.sent!r} -> status {response.status} ({name}): {response.raw}")


class NoResponse(TimeoutError):
    pass


def encode(obj) -> str:
    return json.dumps(obj, separators=(",", ":"))


class TinyG:
    def __init__(self, port: str, baud: int = 115200, *, transcript=None, guard: Guard | None = None):
        self.transport = Transport(port, baud, transcript)
        self.guard = guard or Guard()
        self.identity: dict = {}
        self.dialect: dialects.Dialect | None = None

    # ---- connection -------------------------------------------------------------------

    def connect(self, timeout: float = 5.0) -> dict:
        if not self.transport.is_open:
            self.transport.open()
        self.transport.drain(quiet=0.3, limit=timeout)
        # One GET per line: 440 executes only the first pair of a multi-pair object.
        # The first JSON line also takes the board out of text mode if a $ command left it there.
        self.identity = {k: self.get(k) for k in ("fb", "fv", "hp", "hv", "id")}
        if self.identity["fb"] is None:     # jv too low for JSON GETs to echo values
            self.identity.update({k: self.get_text(k) for k in ("fb", "fv", "hp", "hv")})
        self.dialect = dialects.for_build(self.identity["fb"])
        return self.identity

    def close(self):
        self.transport.close()

    # ---- requests ---------------------------------------------------------------------

    def _wait_response(self, sent: str, timeout: float) -> Response:
        start = time.monotonic()
        seen = []
        while True:
            ev = self.transport.next_event(start + timeout - time.monotonic())
            if ev is None:
                tail = "\n  ".join(e.raw for e in seen[-10:])
                raise NoResponse(f"no response to {sent!r} within {timeout}s; saw:\n  {tail}")
            if ev.kind == "response":
                return Response(sent, ev, Footer.from_event(ev), seen, time.monotonic() - start)
            seen.append(ev)

    def send_line(self, line: str, *, timeout: float = 2.0, check: bool = False) -> Response:
        """Send one line (no guard) and wait for its JSON response."""
        self.transport.write(line.encode() + b"\n")
        resp = self._wait_response(line, timeout)
        if check and resp.status != 0:
            raise StatusError(resp, self.status_name(resp.status))
        return resp

    # Persisting a setting writes EEPROM with every interrupt disabled, and serial input that arrives
    # meanwhile is lost; the wiki asks hosts to wait for each config response and pause >= 30 ms
    # (TinyG-Command-Line.md, TinyG-Configuration-for-Firmware-Version-0.94-and-Earlier.md).
    persist_pause = 0.05

    @staticmethod
    def _is_set(obj) -> bool:
        if isinstance(obj, dict):
            return any(v is not None and (not isinstance(v, dict) or TinyG._is_set(v)) for v in obj.values())
        return isinstance(obj, str) and re.search(r':\s*(?!null\b)[^\s{]', obj) is not None

    def send_json(self, obj, *, timeout: float = 2.0, check: bool = False, allow: str | None = None) -> Response:
        setting = self._is_set(obj)
        if not isinstance(obj, str):
            self.guard.check_json(obj, allow)
            obj = encode(obj)
        resp = self.send_line(obj, timeout=timeout, check=check)
        if setting:
            time.sleep(self.persist_pause)          # let the EEPROM write finish before the next line
        return resp

    def send_gcode(self, line: str, *, timeout: float = 2.0, check: bool = False,
                   allow: str | None = None) -> Response:
        self.guard.check_gcode(line, allow)
        return self.send_line(line, timeout=timeout, check=check)

    def get(self, token: str, *, timeout: float = 2.0):
        resp = self.send_json({token: None}, timeout=timeout, check=True)
        return resp.body.get(token)

    def set(self, token: str, value, *, timeout: float = 2.0, allow: str | None = None):
        resp = self.send_json({token: value}, timeout=timeout, check=True, allow=allow)
        return resp.body.get(token)

    def send_text(self, line: str, *, timeout: float = 2.0) -> list[str]:
        """Send a text-mode command ($..., ?) and return its lines up to the prompt.

        Text commands leave the board in text mode, so a JSON GET follows to switch back.
        """
        self.transport.write(line.encode() + b"\n")
        lines, end = [], time.monotonic() + timeout
        while True:
            ev = self.transport.next_event(end - time.monotonic())
            if ev is None:
                raise NoResponse(f"no prompt after {line!r}; got {lines}")
            lines.append(ev.raw)
            if ev.kind == "prompt":
                break
        if "=" in line:                             # a text-mode SET ($xvm=...) persists too
            time.sleep(self.persist_pause)
        self.get("fb")
        return lines

    def get_text(self, token: str) -> float | None:
        """Read a numeric setting through text mode ($token), which works at any jv."""
        for line in self.send_text(f"${token}"):
            m = re.match(rf"^\[{re.escape(token)}\]\s+(.*)$", line)
            if m:
                value = re.sub(r"\s\[[^\]]*\]\s*$", "", m.group(1))     # drop "[0=...]" legend
                numbers = re.findall(r"(?:^|\s)(-?\d+(?:\.\d+)?)(?=\s|$)", value)
                if numbers:
                    n = float(numbers[-1])
                    return int(n) if n == int(n) else n
        return None

    def send_raw(self, data: bytes):
        """Single-character controls (! ~ % ENQ ^X); no response is awaited."""
        self.transport.write(data)

    def drain(self, quiet: float = 0.15, limit: float = 5.0) -> list[Event]:
        return self.transport.drain(quiet, limit)

    def wait_for(self, predicate, timeout: float) -> Event:
        end = time.monotonic() + timeout
        while True:
            ev = self.transport.next_event(end - time.monotonic())
            if ev is None:
                raise NoResponse(f"no matching event within {timeout}s")
            if predicate(ev):
                return ev

    # ---- machine state ----------------------------------------------------------------

    def status_name(self, code) -> str:
        return self.dialect.status_name(code) if self.dialect else str(code)

    def stat(self) -> int:
        return self.get("stat")

    def wait_idle(self, timeout: float = 30.0, poll: float = 0.1) -> int:
        """Poll until the machine is idle and the planner queue is empty."""
        end = time.monotonic() + timeout
        stuck_since, last = None, None
        while True:
            stat = self.stat()
            qr = self.get("qr")
            if stat in self.dialect.idle_stats:
                if qr == self.planner_buffers:
                    return stat
                # "Idle" with commands left in the queue (E069/E033): either nothing will start them,
                # or stat is wrong while a move runs (E033 reports program-end during motion, and qr
                # does not change until a move finishes). Accept once queue and position are still for 0.5 s.
                now = (qr, self.get("pos"))
                if now != last or stuck_since is None:
                    stuck_since = time.monotonic()
                elif time.monotonic() - stuck_since >= 0.5:
                    return stat
                last = now
            else:
                stuck_since, last = None, None
            if time.monotonic() > end:
                raise NoResponse(f"not idle after {timeout}s (stat {stat}, qr {qr})")
            time.sleep(poll)

    planner_buffers = 32    # PLANNER_BUFFER_POOL_SIZE on both builds; qr reads this when idle

    def clear_alarm(self):
        self.send_json({"clear": None})

    def reset(self, timeout: float = 12.0) -> Event:
        """^X reset; returns the startup banner."""
        self.drain(quiet=0.05)
        self.send_raw(ETX_RESET)
        banner = self.wait_for(lambda e: e.kind == "banner", timeout)
        self.drain(quiet=0.3)
        return banner

    # ---- helpers ----------------------------------------------------------------------

    def values_equal(self, token: str, a, b) -> bool:
        if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
            prec = self.dialect.tokens.get(token, {}).get("precision", 3)
            return math.isclose(a, b, rel_tol=1e-6, abs_tol=0.5 * 10 ** -max(prec, 3))
        return a == b
