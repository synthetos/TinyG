"""TinyG driver: one request at a time over the serial transport (ping-pong)."""
import json
import re
import math
import time

from . import dialect as dialects
from .protocol import Event, Footer, Response, loads
from .safety import Guard
from .transport import Transport

ETX_RESET = b"\x18"     # ^X: watchdog reset
ENQ = b"\x05"


class StatusError(Exception):
    def __init__(self, response: Response, name: str):
        self.response = response
        super().__init__(f"{response.sent!r} -> status {response.status} ({name}): {response.raw}")


class IdentityError(ValueError):
    """An application answered, but its firmware identity could not be established."""


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
        self._next_tid = 1
        self.reset_answered = False
        self._json_verbosity = 3

    # ---- connection -------------------------------------------------------------------

    def connect(self, timeout: float = 5.0) -> dict:
        # A flash can replace the dialect and reset verbosity to master defaults.
        self.dialect = None
        self.identity = {}
        self._json_verbosity = 3
        if not self.transport.is_open:
            self.transport.open()
        self.transport.drain(quiet=0.3, limit=timeout)
        if not self._responds():
            self._wake()
        # One GET per line: 440 executes only the first pair of a multi-pair object.
        # The first JSON line also takes the board out of text mode if a $ command left it there.
        self.identity = {k: self._bootstrap_query(k, timeout).body.get(k)
                         for k in ("fb", "fv", "hp", "hv", "id")}
        if self.identity["fb"] is None:     # jv too low for JSON GETs to echo values
            self._json_verbosity = 1
            self.identity.update({k: self.get_text(k) for k in ("fb", "fv", "hp", "hv")})
        try:
            fb = float(self.identity["fb"])
            if not math.isfinite(fb) or fb <= 0:
                raise ValueError("invalid build number")
            self.identity["fb"] = fb
            self.dialect = dialects.for_build(fb)
        except (TypeError, ValueError) as exc:
            raise IdentityError(f"invalid firmware identity after JSON/text bootstrap: {self.identity}") from exc
        return self.identity

    def _untagged_query(self, line: str, keys: tuple[str, ...], timeout: float,
                        *, allow_empty: bool = False) -> Response:
        seen = []
        while (ev := self.transport.next_event(0)) is not None:
            seen.append(ev)
        self.transport.write(line.encode() + b"\n")
        return self._wait_response(line, timeout, keys=keys, allow_empty=allow_empty, seen=seen)

    def _bootstrap_query(self, token: str, timeout: float) -> Response:
        # Startup/ALARM and older firmware must be reachable without a metadata
        # SET. Prefer a matching key; a footer-only OK supports low verbosity.
        return self._untagged_query(encode({token: None}), (token,), timeout, allow_empty=True)

    def _responds(self, timeout: float = 1.5) -> bool:
        try:
            self._bootstrap_query("fb", timeout)
            return True
        except NoResponse:
            return False

    def _wake(self):
        """A silent board: first release a transmitter paused by an XOFF from the host (a stuck
        XON/XOFF stream leaves it waiting for XON forever), then reset it if it is still silent."""
        self.transport.write(b"\x11\n")        # XON; harmless junk line when XON/XOFF is off
        self.transport.drain(quiet=0.3, limit=2.0)
        if self._responds():
            return
        self.reset()                            # ends with the bootstrap query that switches the board to JSON
        if not self.reset_answered:
            raise NoResponse("board silent after XON and a reset")

    def close(self):
        self.transport.close()

    # ---- requests ---------------------------------------------------------------------

    @staticmethod
    def _request_object(line: str, *, strict: bool = False) -> dict | None:
        try:
            obj = json.loads(line) if strict else loads(line)
        except ValueError:
            return None
        return obj if isinstance(obj, dict) else None

    @classmethod
    def _untagged_keys(cls, line: str) -> tuple[str, ...] | None:
        pairs = cls._request_pairs(line)
        keys = tuple(key for key, _ in pairs if key != "tid")
        try:
            obj = json.loads(line)
        except ValueError:
            return keys or None  # relaxed/malformed requests must retain their wire bytes
        if not isinstance(obj, dict):
            return None
        # Metadata is a numeric SET, and group setters consume later pairs as
        # group children. Neither reads nor groups may gain a metadata SET.
        if any(k.lower() == "rt" for k in obj):
            return ("rt",)
        if any(k.lower() == "gc" for k in obj):
            return ("gc",)
        if any(isinstance(v, (dict, list)) for v in obj.values()) or all(v is None for v in obj.values()):
            return keys
        return None

    @staticmethod
    def _request_pairs(line: str) -> list[tuple[str, str]]:
        """Read top-level keys/metadata without rewriting TinyG's relaxed syntax."""
        if not line.lstrip().startswith("{"):
            return []
        start = line.index("{") + 1
        depth, quoted, escaped, parts = 1, False, False, []
        for i in range(start, len(line)):
            c = line[i]
            if quoted:
                if escaped:
                    escaped = False
                elif c == "\\":
                    escaped = True
                elif c == '"':
                    quoted = False
            elif c == '"':
                quoted = True
            elif c in "{[":
                depth += 1
            elif c in "}]":
                depth -= 1
                if depth == 0:
                    parts.append(line[start:i])
                    break
            elif c == "," and depth == 1:
                parts.append(line[start:i])
                start = i + 1
        else:
            parts.append(line[start:])
        pairs = []
        for part in parts:
            match = re.fullmatch(r'\s*(?:"([^"\\]+)"|([\w]+))\s*:\s*(.*?)\s*', part, re.DOTALL)
            if match:
                pairs.append(((match[1] or match[2]).lower(), match[3]))
        return pairs

    def _wait_response(self, sent: str, timeout: float, *, tid: int | None = None,
                       correlate: bool = True, seen: list | None = None,
                       keys: tuple[str, ...] | None = None, allow_empty: bool = False) -> Response:
        start = time.monotonic()
        seen = [] if seen is None else seen
        obj = self._request_object(sent, strict=True) or {}
        gc_message = keys == ("gc",) and any(k.lower() == "gc" and isinstance(v, str)
                                             for k, v in obj.items())
        while True:
            remaining = start + timeout - time.monotonic()
            ev = self.transport.next_event(remaining) if remaining > 0 else None
            if ev is None:
                tail = "\n  ".join(e.raw for e in seen[-10:])
                raise NoResponse(f"no response to {sent!r} within {timeout}s; saw:\n  {tail}")
            if ev.kind == "response":
                tagged = ev.data.get("tid")
                # 449.01 echoes transaction metadata inside r. Keep the 450+
                # top-level contract strict, and never override a top-level tid.
                if ("tid" not in ev.data and self.dialect is not None and
                        self.dialect.name == "d449" and
                        449 <= float(self.identity.get("fb", 0)) < 450 and
                        isinstance(ev.body, dict)):
                    tagged = ev.body.get("tid")
                keyed = keys is not None and isinstance(ev.body, dict) and all(k in ev.body for k in keys)
                footer = Footer.from_event(ev)
                # At ordinary verbosity gc itself is hidden, but (MSG ...) output is returned.
                if gc_message and isinstance(ev.body, dict) and "msg" in ev.body and footer is not None:
                    keyed = True
                empty_ok = allow_empty and not ev.body and footer is not None and footer.status == 0
                # Parse/transport rejection may precede both command echo and tid scanning.
                error_ok = tagged is None and not ev.body and footer is not None and footer.status != 0
                # Syntax errors echo the original malformed bytes under err, not the request key.
                echoed_error = (tagged is None and footer is not None and footer.status != 0 and
                                isinstance(ev.body, dict) and ev.body.get("err") == sent)
                matched_tid = tid is not None and tagged == tid
                matched_plain = tid is None and tagged is None and (keys is None or keyed or empty_ok)
                if not correlate or error_ok or echoed_error or matched_tid or matched_plain:
                    return Response(sent, ev, footer, seen, time.monotonic() - start)
            seen.append(ev)

    def send_line(self, line: str, *, timeout: float = 2.0, check: bool = False,
                  correlate: bool = True) -> Response:
        """Tag flat strict JSON writes; preserve groups, relaxed syntax and caller tids.

        correlate=False preserves exact wire bytes for protocol tests (including
        explicit/invalid tids). It retains first-response matching.
        """
        wire, tid, seen = line, None, []
        # json.loads must consume the entire object before any metadata is considered.
        strict_obj = self._request_object(line, strict=True) if correlate else None
        keys = self._untagged_keys(line) if correlate else None
        if self.dialect is not None and self.dialect.name == "d440" and strict_obj:
            # Master does not implement tid: metadata is an unrecognized setting.
            # It executes one top-level command; correlate its plain response.
            keys = tuple(strict_obj)[:1]
        while (ev := self.transport.next_event(0)) is not None:
            seen.append(ev)
        pairs = self._request_pairs(line) if correlate else []
        caller_tid = next((value for key, value in pairs if key == "tid"), None)
        if caller_tid is not None:
            if re.fullmatch(r"[0-9]+", caller_tid) and int(caller_tid) <= 0xFFFFFFFF:
                tid = int(caller_tid)
        elif correlate and keys is None and strict_obj is not None:
            tid = self._next_tid
            self._next_tid = tid % 0xFFFFFFFF + 1
            _, tail = line.split("{", 1)
            # The firmware alarm-gates numeric pairs as SETs. Put metadata
            # after the command so clear executes before that guard. _scan_tid
            # finds it anywhere, including after a command that returns an error.
            if line.rstrip().endswith("}"):
                end = line.rfind("}")
                comma = "," if tail[:tail.rfind("}")].strip() else ""
                wire = line[:end] + comma + '"tid":' + str(tid) + line[end:]
        self.transport.write(wire.encode() + b"\n")
        resp = self._wait_response(line, timeout, tid=tid, correlate=correlate, seen=seen, keys=keys,
                                   allow_empty=keys is not None and (self._json_verbosity < 3 or
                                               (keys == ("gc",) and self._json_verbosity < 5)))
        if resp.status == 0:
            obj = self._request_object(line) or {}
            for group in (obj, obj.get("sys", {})):
                value = group.get("jv") if isinstance(group, dict) else None
                if isinstance(value, int) and 0 <= value <= 5:
                    self._json_verbosity = value
        if check and resp.status != 0:
            raise StatusError(resp, self.status_name(resp.status))
        return resp

    # Persisting a setting masks serial interrupts, and serial input that arrives
    # meanwhile is lost; the wiki asks hosts to wait for each config response and pause >= 30 ms
    # (TinyG-Command-Line.md, TinyG-Configuration-for-Firmware-Version-0.94-and-Earlier.md).
    persist_pause = 0.05

    @staticmethod
    def _is_set(obj) -> bool:
        if isinstance(obj, dict):
            return any(v is not None and (not isinstance(v, dict) or TinyG._is_set(v)) for v in obj.values())
        return isinstance(obj, str) and re.search(r':\s*(?!null\b)[^\s{]', obj) is not None

    def send_json(self, obj, *, timeout: float = 2.0, check: bool = False, allow: str | None = None,
                  correlate: bool = True) -> Response:
        setting = self._is_set(obj)
        if not isinstance(obj, str):
            self.guard.check_json(obj, allow)
            obj = encode(obj)
        resp = self.send_line(obj, timeout=timeout, check=check, correlate=correlate)
        if setting:
            time.sleep(self.persist_pause)          # let the EEPROM write finish before the next line
        return resp

    def send_gcode(self, line: str, *, timeout: float = 2.0, check: bool = False,
                   allow: str | None = None) -> Response:
        self.guard.check_gcode(line, allow)
        resp = self.send_line(line, timeout=timeout, check=check)
        if re.search(r"\bG10\b", line, re.IGNORECASE):
            time.sleep(self.persist_pause)          # G10 offsets are written to EEPROM too
        return resp

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
        verbosity = re.fullmatch(r"\s*\$jv\s*=\s*([0-5])\s*", line, re.IGNORECASE)
        if verbosity:
            self._json_verbosity = int(verbosity[1])
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
                # does not change until a move finishes). Accept once queue and position are still for
                # stuck_window s; a dwell keeps both still too, so the window outlasts the dwells tests use.
                now = (qr, self.get("pos"))
                if now != last or stuck_since is None:
                    stuck_since = time.monotonic()
                elif time.monotonic() - stuck_since >= self.stuck_window:
                    return stat
                last = now
            else:
                stuck_since, last = None, None
            if time.monotonic() > end:
                raise NoResponse(f"not idle after {timeout}s (stat {stat}, qr {qr})")
            time.sleep(poll)

    stuck_window = 2.5      # s of unchanged queue and position that count as a stuck (never started) queue
    planner_buffers = 32    # PLANNER_BUFFER_POOL_SIZE on both builds; qr reads this when idle

    def clear_alarm(self):
        self.send_json({"clear": None})

    def reset(self, timeout: float = 12.0) -> Event:
        """^X reset; wait through initialization/loading for SYSTEM READY."""
        self.drain(quiet=0.05)
        self.send_raw(ETX_RESET)
        banner = self.wait_for(lambda e: e.kind == "banner" and e.body.get("msg") == "SYSTEM READY", timeout)
        self.drain(quiet=0.3)
        # The board boots in its persisted comm mode; a leading '{' switches it to JSON. One untagged bootstrap
        # query does that and proves it answers, so a G-code line right after a reset is matched as JSON, not
        # met by a text prompt (N12c's cleanup in the chain 15 gate, 2026-11-04).
        self.reset_answered = self._responds(timeout=3.0)   # _wake raises on silence; other callers carry on
        return banner

    # ---- helpers ----------------------------------------------------------------------

    def values_equal(self, token: str, a, b) -> bool:
        if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
            prec = self.dialect.tokens.get(token, {}).get("precision", 3)
            return math.isclose(a, b, rel_tol=1e-6, abs_tol=0.5 * 10 ** -max(prec, 3))
        return a == b
