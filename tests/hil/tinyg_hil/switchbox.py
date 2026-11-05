"""Switchbox: drive the TinyG's limit/homing inputs electrically from a Feather HUZZAH ESP8266.

Each switchbox channel is an ESP8266 GPIO wired to one TinyG switch input (plus a common GND). It
emulates a switch contact: "closed" sinks the line to GND, "open" leaves it high-Z for TinyG's
pull-up. The box (firmware, wiring, API) lives in its own repository, synthetos/tinyg-switchbox.
This client uses a pinned copy of its API, tinyg_hil/data/switchbox-openapi.json; the key
"x-vendored-from" names the switchbox commit it came from, and spec_mismatch() tells a board that
serves a different API version apart from one that matches.

    with SwitchBox("/dev/cu.usbserial-XXXX") as sb:
        sb.close("x", "min")              # trip X min (SoftSwitches semantics, see below)
        sb.open("x", "min")
        sb.pulse("x", "min", ms=20)       # timed on the board, not by two host commands
        sb.close("z", "max", ttl_ms=2000) # opens by itself after 2 s even if this process dies
        sb.bounce("y", "min", edges=6, interval_us=300, final_action="trip")
        sb.state(); sb.events(since=0)
        sb.release_all()

close/open/pulse/release_all mirror tinyg_hil.switches.SoftSwitches so a test can swap backends.
As there, close() trips the switch and open() releases it *for the channel's type*: on an NC channel
close() opens the contact. contact(name, "closed"|"open") sets the literal contact level.
release_all() opens every contact (nothing driven: NC channels then read tripped); idle() un-trips
every channel. Channels are named "<axis><min|max>"; methods take (axis, end) or one channel name.

Serial: newline-delimited JSON at 115200. The CP2104's DTR/RTS lines drive the ESP8266's
auto-reset transistors. connect() sets both asserted before opening (macOS asserts them on open, so
open() changes nothing), then drops RTS before DTR, so RTS is never asserted alone; it checks with
getInfo (uptime, boot line) that opening did not reset the board (see LINE_STRATEGIES). It discards whatever the port
already held when it opened: a reply the board queued for the previous connection would otherwise be read as the
answer to this one. Request ids start at a random number on every connection for the same reason. Lines that are not JSON objects
(the ROM prints a banner at 74880 baud after a reset) are skipped and kept in .garbage.

The port is never guessed. connect() refuses TinyG bench ports and the ports listed under
"never_open" in the bench file (tinyg_hil.benches: $TINYG_BENCHES or ~/.config/tinyg-hil/benches.json),
and takes the harness board lock for the switchbox's port.
"""
import argparse
import collections
import contextlib
import getpass
import hashlib
import json
import os
import pathlib
import random
import re
import sys
import time

import serial

from . import benches as bench_file
from .lock import BoardLock

SPEC_PATH = pathlib.Path(__file__).resolve().parent / "data" / "switchbox-openapi.json"
VENDOR_KEY = "x-vendored-from"
LINE_MAX = 1024                 # the firmware's request line limit
# How connect() handles the CP2104's DTR/RTS, which drive the ESP8266 auto-reset transistors
# (RTS asserted alone pulls EN low = reset; DTR asserted alone pulls GPIO0 low). macOS asserts both
# when the port opens, and pyserial then applies DTR before RTS.
#   "release-after-open" (default): set both True before open, so open() changes nothing, then
#                         drop RTS before DTR. No resets on open or close (diagnostic
#                         test_switchbox_reset_on_open, 2026-10-31).
#   "deasserted":         set both False before open. pyserial drops DTR first, leaving RTS asserted
#                         alone: the board reset on every open.
#   "asserted":           set both True before open and keep them (the OS drops them on close).
LINE_STRATEGIES = ("release-after-open", "deasserted", "asserted")
SPEC_TIMEOUT = 15.0             # getSpec sends ~26 KB at 115200 (~2.3 s)


class SwitchBoxError(RuntimeError):
    """The board answered with an error (code from the spec's ErrorCode enum)."""

    def __init__(self, code, msg, reply=None):
        super().__init__(f"{code}: {msg}")
        self.code, self.msg, self.reply = code, msg, reply


class SwitchBoxTimeout(SwitchBoxError):
    pass


class SwitchBoxRebooted(SwitchBoxError):
    pass


class SwitchBoxSpecMismatch(SwitchBoxError):
    """The box serves a different API than the vendored spec; no contact operation was sent."""


def op(name):
    """Marks a method as the client for the spec operation `name` (checked by the offline tests)."""
    def mark(fn):
        fn.sb_op = name
        return fn
    return mark


def load_spec(path=SPEC_PATH):
    """The vendored switchbox spec, exactly as the box serves it (the provenance key removed)."""
    spec = json.loads(pathlib.Path(path).read_text())
    spec.pop(VENDOR_KEY, None)
    return spec


def vendored_from(path=SPEC_PATH):
    """{"repository", "commit", "path", "note"}: where the vendored spec was copied from."""
    return json.loads(pathlib.Path(path).read_text())[VENDOR_KEY]


def spec_mismatch(info, path=SPEC_PATH):
    """None if a board's getInfo matches the vendored spec, else a message saying which side to update."""
    spec = load_spec(path)
    want_v, want_d = spec["info"]["version"], spec_digest(spec)
    got_v, got_d = info.get("api_version"), info.get("spec_sha256")
    if got_d == want_d:
        return None
    src = vendored_from(path)
    kind = "a different API version" if got_v != want_v else "a different spec with the same API version"
    return (f"switchbox serves {kind}: API {got_v} (spec {str(got_d)[:12]}), but this client vendors API {want_v} "
            f"(spec {want_d[:12]}) from {src['repository']}@{src['commit'][:12]}. Update "
            f"tests/hil/tinyg_hil/data/switchbox-openapi.json from the switchbox repository, or flash the box "
            f"from that commit.")


def spec_digest(spec):
    """SHA-256 of the spec minified as the firmware build embeds it (getInfo reports spec_sha256)."""
    return hashlib.sha256(json.dumps(spec, separators=(",", ":"), ensure_ascii=False).encode()).hexdigest()


def operation_ids(spec):
    """Every operationId: the HTTP paths plus the serial-only operations."""
    ids = {o["operationId"] for p in spec["paths"].values() for o in p.values()}
    return ids | set(spec.get("x-serial-protocol", {}).get("operations", {}))


def channel_name(axis, end=None):
    return axis if end is None else f"{axis}{end}"


def refuse_port(port, benches=None):
    """Raises ValueError for a TinyG bench port or a port listed under "never_open" in the bench file."""
    benches = bench_file.load() if benches is None else benches
    forbidden = {k for k, v in benches.items() if isinstance(v, dict) and k.startswith("/dev/")}
    forbidden |= set(benches.get("never_open", []))
    real = os.path.realpath(port)
    for f in forbidden:
        if port == f or real == os.path.realpath(f):
            raise ValueError(f"refusing to open {port} as a switchbox: it is a TinyG bench port or listed in "
                             f"never_open (the bench file, tinyg_hil.benches)")


def resolve_port(explicit=None, tinyg_port=None, benches=None):
    """The switchbox port: an explicit one; else the switchbox entry of the TinyG's bench; else, for a run
    without a TinyG, the bench file's top-level switchbox entry; else None."""
    if explicit:
        return explicit
    benches = bench_file.load() if benches is None else benches
    if tinyg_port:
        entry = benches.get(tinyg_port)
        sb = entry.get("switchbox") if isinstance(entry, dict) else None
    else:
        sb = benches.get("switchbox")
    return sb.get("port") if isinstance(sb, dict) else None


# WiFi credentials never go into traffic logs or run records. Every password is masked. Every SSID is
# masked too, except the switchbox's own access-point name, and only as an "ssid" value.
AP_PREFIX = "tinyg-switchbox-"
# A "ssid"/"password" member in text: a terminated string value; else, once the value opens with a
# quote, everything to the end of the text (a cut-off value, a dangling escape: never a shorter
# match); else a bare non-string value.
_SECRET = re.compile(r'"(ssid|password)"\s*:\s*("(?:[^"\\]|\\.)*"|".*|[^"\s,}\]][^,}\]\s]*)', re.S)
_AP_VALUE = re.compile(r'"' + re.escape(AP_PREFIX) + r'(?:[^"\\]|\\.)*"')


def _keep(key, value):
    """True only for the box's own access-point name as an SSID."""
    if key != "ssid":
        return False
    if isinstance(value, str):
        return value.startswith(AP_PREFIX)
    return isinstance(value, re.Match) and _AP_VALUE.fullmatch(value.group(2)) is not None


def redact_text(text):
    """Masks credentials in a line of text. Redact whole lines; cut them short only afterwards."""
    return _SECRET.sub(lambda m: m.group(0) if _keep(m.group(1), m) else f'"{m.group(1)}":"***"', text)


def redact(obj):
    """A copy of a reply/result with WiFi SSIDs and passwords masked (for records)."""
    if isinstance(obj, dict):
        return {k: ("***" if k in ("ssid", "password") and not _keep(k, v) else redact(v)) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [redact(v) for v in obj]
    if isinstance(obj, str):
        return redact_text(obj)
    return obj


def decode_mask(mask, names):
    """A levels/driven mask ("0111...") as {channel name: 0/1}."""
    return {n: int(c) for n, c in zip(names, mask)}


def _first_id():
    """A random first request id: positive and well inside the box's 32-bit request ids."""
    return random.SystemRandom().randrange(1 << 16, 1 << 30)


class SwitchBox:
    def __init__(self, port, baud=115200, timeout=2.0, lock=True, lock_wait=0.0, purpose="switchbox",
                 serial_factory=serial.Serial, allow_reset=False, benches=None, lines="release-after-open"):
        self.port, self.baud, self.timeout = port, baud, timeout
        self.lock, self.lock_wait, self.purpose = lock, lock_wait, purpose
        self.serial_factory = serial_factory
        self.allow_reset = allow_reset
        if lines not in LINE_STRATEGIES:
            raise ValueError(f"lines must be one of {LINE_STRATEGIES}")
        self.lines = lines
        self.benches = benches
        self.ser = None
        self._lock = None
        self._rx = bytearray()
        self._next_id = _first_id()
        self.last_id = None
        self.traffic = collections.deque(maxlen=5000)    # (monotonic, direction, text)
        self.traffic_count = 0
        self.garbage = collections.deque(maxlen=50)      # non-JSON lines (ROM banner, noise)
        self.garbage_count = 0
        self.stale = collections.deque(maxlen=50)        # replies to requests that already timed out
        self.received = collections.deque(maxlen=2000)   # event lines (after subscribe, and boot lines)
        self.boot_seen = False
        self.reset_on_connect = None
        self.rebooted_between_connections = None
        self.boot_id = None
        self.info_at_connect = None
        self.last_events_page = None

    # ---- connection ---------------------------------------------------------------------

    def connect(self):
        refuse_port(self.port, self.benches)
        if self.lock:
            self._lock = BoardLock(self.port).acquire(self.purpose, wait=self.lock_wait)
        try:
            ser = self.serial_factory()          # unopened, so DTR/RTS are set before the port opens
            self.ser = ser                       # owned from here: disconnect() closes it whatever fails next
            ser.port, ser.baudrate = self.port, self.baud
            ser.timeout, ser.write_timeout = 0.05, 2.0
            ser.dtr = ser.rts = self.lines != "deasserted"      # set on the unopened port, applied by open()
            ser.open()
            if self.lines == "release-after-open":
                ser.rts = False                  # RTS first: DTR alone only pulls GPIO0, never EN
                ser.dtr = False
            ser.reset_input_buffer()             # bytes from before this open belong to an earlier connection
            self._rx.clear()
            self._next_id = _first_id()
            opened = time.monotonic()
            self.boot_seen = False
            self.info_at_connect = info = self._first_info()
            elapsed_ms = (time.monotonic() - opened) * 1000
            previous, self.boot_id = self.boot_id, info.get("boot_id")
            self.rebooted_between_connections = (previous is not None and self.boot_id is not None
                                                 and self.boot_id != previous)
            # An uptime shorter than the time since the port opened means it booted after we opened it. This holds
            # on the ESP8266, whose reset-on-open is the case it guards; on the ESP32-S2 uptime survives a software
            # reset, so rebooted_since_connect() and rebooted_between_connections compare boot_id instead.
            self.reset_on_connect = self.boot_seen or info["uptime_ms"] < elapsed_ms + 300
            if self.reset_on_connect and not self.allow_reset:
                raise SwitchBoxRebooted("rebooted", f"opening {self.port} reset the switchbox "
                                        f"(uptime {info['uptime_ms']} ms, boot line seen: {self.boot_seen})")
        except BaseException:
            with contextlib.suppress(Exception):     # the original failure is the one to report
                self.disconnect()
            raise
        return self

    def rebooted_since_connect(self):
        """True if the box restarted since connect(): its getInfo boot_id changed (uptime alone can miss it)."""
        info = self.call("getInfo")
        return self.boot_id is not None and info.get("boot_id") != self.boot_id

    def check_spec(self):
        """Raises SwitchBoxSpecMismatch unless the box's getInfo matches the vendored spec."""
        mismatch = spec_mismatch(self.info_at_connect or {})
        if mismatch:
            raise SwitchBoxSpecMismatch("spec_mismatch", mismatch)

    def _first_info(self):
        """getInfo, retried for a few seconds: if opening did reset the board, it is still booting."""
        deadline = time.monotonic() + 4.0
        while True:
            try:
                return self.call("getInfo", timeout=0.8)
            except SwitchBoxTimeout:
                if time.monotonic() >= deadline:
                    raise

    def disconnect(self):
        """Closes the port and releases the board lock; the lock is released even if closing fails."""
        try:
            if self.ser is not None:
                ser, self.ser = self.ser, None
                ser.close()
        finally:
            if self._lock is not None:
                lock, self._lock = self._lock, None
                lock.release()

    def __enter__(self):
        if self.ser is None:
            self.connect()
        return self

    def __exit__(self, *exc):
        self.disconnect()

    # ---- framing ------------------------------------------------------------------------

    def _log(self, direction, text, limit=2000):
        """Records a line: redacted whole first, then shortened (cutting first could split a credential
        from its closing quote)."""
        self.traffic.append((time.monotonic(), direction, redact_text(text)[:limit]))
        self.traffic_count += 1

    def _readline(self, deadline):
        while True:
            nl = self._rx.find(b"\n")
            if nl >= 0:
                line = bytes(self._rx[:nl])
                del self._rx[:nl + 1]
                return line
            if time.monotonic() >= deadline:
                return None
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            if chunk:
                self._rx.extend(chunk)

    def _parse(self, raw):
        """A JSON object from one line, or None (recorded as garbage)."""
        text = raw.decode("utf-8", errors="replace").strip()
        if not text:
            return None
        msg = None
        if text.startswith("{"):
            try:
                msg = json.loads(text)
            except ValueError:
                msg = None
        if not isinstance(msg, dict):
            self.garbage.append(raw)
            self.garbage_count += 1
            self._log("??", text, 200)
            return None
        return msg

    def _dispatch(self, msg, text):
        """Routes a line that is not the awaited reply. Returns True if it was an event."""
        if "event" in msg and "id" not in msg:
            if msg["event"] == "boot":
                self.boot_seen = True
            self.received.append(msg)
            self._log("ev", text, 400)
            return True
        self.stale.append(msg)
        self._log("<<", text, 400)
        return False

    def call(self, op_name, timeout=None, **params):
        """Sends one request and returns its result; raises SwitchBoxError / SwitchBoxTimeout."""
        if self.ser is None:
            raise RuntimeError("switchbox not connected")
        rid = self.last_id = self._next_id      # events this request causes carry it as "rid"
        self._next_id += 1
        params = {k: v for k, v in params.items() if v is not None}
        line = json.dumps({"id": rid, "op": op_name, **params}, separators=(",", ":"))
        if len(line) > LINE_MAX:
            raise ValueError(f"request is {len(line)} bytes; the switchbox takes at most {LINE_MAX}")
        self.ser.write(line.encode() + b"\n")
        self._log(">>", line)
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while True:
            raw = self._readline(deadline)
            if raw is None:
                raise SwitchBoxTimeout("timeout", f"no reply to {op_name} (id {rid}) from {self.port}")
            msg = self._parse(raw)
            if msg is None:
                continue
            if msg.get("id") != rid:
                self._dispatch(msg, raw.decode(errors="replace"))
                continue
            self._log("<<", raw.decode(errors="replace"))
            if msg.get("ok"):
                return msg.get("result")
            err = msg.get("error") or {}
            raise SwitchBoxError(err.get("code", "?"), err.get("msg", ""), msg)

    def poll(self, seconds=0.1):
        """Reads lines for `seconds` and returns the events among them (useful after subscribe())."""
        got, deadline = [], time.monotonic() + seconds
        while True:
            raw = self._readline(deadline)
            if raw is None:
                return got
            msg = self._parse(raw)
            if msg is not None and self._dispatch(msg, raw.decode(errors="replace")):
                got.append(msg)

    # ---- operations (one per spec operationId) ------------------------------------------

    @op("getInfo")
    def info(self):
        return self.call("getInfo")

    @op("getState")
    def state(self):
        return self.call("getState")

    @op("setChannel")
    def set_channel(self, name, state=None, action=None, ttl_ms=None):
        return self.call("setChannel", name=name, state=state, action=action, ttl_ms=ttl_ms)

    @op("pulseChannel")
    def pulse_channel(self, name, ms=None, us=None, action=None, state=None, wait=True):
        r = self.call("pulseChannel", name=name, ms=ms, us=us, action=action, state=state)
        if wait:
            self._wait_done(name, r["end_us"] - r["start_us"])
        return r

    @op("bounceChannel")
    def bounce_channel(self, name, edges, interval_us, final_state=None, final_action=None, ttl_ms=None, wait=True):
        r = self.call("bounceChannel", name=name, edges=edges, interval_us=interval_us, final_state=final_state,
                      final_action=final_action, ttl_ms=ttl_ms)
        if wait:
            self._wait_done(name, r["end_us"] - r["start_us"])
        return r

    @op("releaseAll")
    def release_all(self):
        """Every contact open, nothing driven (the power-on state); cancels pulses, bounces and ttls."""
        return self.call("releaseAll")

    @op("idleAll")
    def idle(self):
        """Every switch released for its type: NO channels open, NC channels closed."""
        return self.call("idleAll")

    @op("getConfig")
    def config(self):
        return self.call("getConfig")

    @op("setConfig")
    def set_config(self, channels=None, defaults=False):
        """Replace the mapping: channels=[{"name":"xmin","gpio":4,"type":"no"}, ...] or defaults=True."""
        if defaults:
            return self.call("setConfig", defaults=True)
        return self.call("setConfig", channels=[{k: c[k] for k in ("name", "gpio", "type") if k in c}
                                                for c in channels])

    @op("getEvents")
    def events(self, since=0, limit=None, all=False):
        """Events with seq >= since. With all=True, pages until the newest; returns the list either way."""
        page = self.call("getEvents", since=since, limit=limit)
        events = page["events"]
        while all and page["more"]:
            page = self.call("getEvents", since=page["next"], limit=limit)
            events += page["events"]
        self.last_events_page = page
        return events

    def head(self):
        """The seq the next board event will get: pass it to events(since=...) to see only what follows."""
        return self.call("getEvents", since=0xFFFFFFFF)["head"]

    @op("setWifi")
    def set_wifi(self, ssid, password=""):
        """Store station credentials on the board (never echoed; redacted in .traffic)."""
        return self.call("setWifi", ssid=ssid, password=password)

    @op("scanWifi")
    def scan_wifi(self):
        """Networks the board sees (blocks the board ~2-4 s). SSIDs are masked in .traffic."""
        return self.call("scanWifi", timeout=12.0)

    def match_ssid(self, ssid):
        """The one scanned SSID equal to `ssid` ignoring case and surrounding spaces.

        Raises LookupError when none or more than one distinct SSID matches; the message gives counts
        only, never names."""
        want = ssid.strip().casefold()
        names = {n["ssid"] for n in self.scan_wifi()["networks"] if n["ssid"].strip().casefold() == want}
        if len(names) != 1:
            raise LookupError(f"{len(names)} scanned SSIDs match ignoring case and spaces; need exactly one")
        return names.pop()

    @op("getSpec")
    def spec(self):
        return self.call("getSpec", timeout=SPEC_TIMEOUT)

    @op("getBoard")
    def board(self):
        """The carrier/Feather/target profiles the firmware was built for and the derived terminal map."""
        return self.call("getBoard", timeout=SPEC_TIMEOUT)

    @op("subscribe")
    def subscribe(self, on=True):
        """Event lines on this serial connection from now on; read them with poll()."""
        return self.call("subscribe", on=on)

    # ---- SoftSwitches-compatible helpers ------------------------------------------------

    def close(self, axis, end=None, ttl_ms=None):
        """Trip the switch for its type (SoftSwitches: close = trip). NO: contact closed; NC: open."""
        return self.set_channel(channel_name(axis, end), action="trip", ttl_ms=ttl_ms)

    def open(self, axis, end=None):
        """Release (un-trip) the switch for its type. NO: contact open; NC: closed."""
        return self.set_channel(channel_name(axis, end), action="release")

    def trip(self, axis, end=None, ttl_ms=None):
        return self.close(axis, end, ttl_ms=ttl_ms)

    def release(self, axis, end=None):
        return self.open(axis, end)

    def contact(self, axis, end=None, state="closed", ttl_ms=None):
        """Set the literal contact: "closed" sinks the line to GND, "open" leaves it high-Z."""
        return self.set_channel(channel_name(axis, end), state=state, ttl_ms=ttl_ms)

    def pulse(self, axis, end=None, ms=None, us=None, action="trip", wait=True):
        """Trip (or `action`) for ms/us, then back; both edges are timed on the board."""
        return self.pulse_channel(channel_name(axis, end), ms=ms, us=us, action=action, wait=wait)

    def bounce(self, axis, end=None, edges=5, interval_us=200, final_state=None, final_action=None, ttl_ms=None,
               wait=True):
        if final_state is None and final_action is None:
            final_action = "trip"
        return self.bounce_channel(channel_name(axis, end), edges, interval_us, final_state=final_state,
                                   final_action=final_action, ttl_ms=ttl_ms, wait=wait)

    def channel(self, axis, end=None):
        name = channel_name(axis, end)
        for c in self.state()["channels"]:
            if c["name"] == name:
                return c
        raise KeyError(f"no switchbox channel {name}")

    def _wait_done(self, name, duration_us):
        """Sleeps through a pulse/bounce, then waits until the board reports it finished."""
        time.sleep(duration_us / 1e6)
        deadline = time.monotonic() + 1.0 + duration_us / 1e6
        while True:
            if self.channel(name)["pending"] is None:
                return
            if time.monotonic() >= deadline:
                raise SwitchBoxTimeout("timeout", f"{name}: pulse/bounce still pending")
            time.sleep(0.005)

    # ---- records ------------------------------------------------------------------------

    def identity(self):
        info = redact(self.info_at_connect or {})
        return {"port": self.port, "reset_on_connect": self.reset_on_connect,
                **{k: info.get(k) for k in ("fw_version", "api_version", "spec_sha256", "boot_id", "chip_id",
                                            "reset_reason", "uptime_ms")},
                "spec_matches_checkout": info.get("spec_sha256") == spec_digest(load_spec())}

    def mark(self):
        """A point to collect traffic and board events from (see since())."""
        return {"traffic": self.traffic_count, "seq": self.head()}

    def since(self, mark):
        """Traffic and board events after mark(), for a run record."""
        n = self.traffic_count - mark["traffic"]
        traffic = list(self.traffic)[-n:] if n > 0 else []
        t0 = traffic[0][0] if traffic else time.monotonic()
        events = self.events(since=mark["seq"], limit=32, all=True)
        return {"traffic": [{"t": round(t - t0, 4), "dir": d, "line": s} for t, d, s in traffic],
                "events": redact(events), "garbage_lines": self.garbage_count}


# ---- pytest fixtures (conftest.py) -------------------------------------------------------------

@contextlib.contextmanager
def session_box(port, identity=None, **client_kwargs):
    """A connected SwitchBox for a whole test session.

    identity(dict) receives the box's identity for the run record, even when its spec does not match.
    The spec is checked before any contact operation: a box serving a different API is never driven.
    However setup ends, the port is closed and the lock released; contacts are released first if any
    operation was allowed."""
    sb = SwitchBox(port, **client_kwargs)
    sb.connect()
    usable = False
    try:
        if identity is not None:
            identity(sb.identity())
        sb.check_spec()
        usable = True
        yield sb
    finally:
        try:
            if usable:
                sb.release_all()
        finally:
            sb.disconnect()


@contextlib.contextmanager
def prepared(sb, record):
    """One test's use of a session box: every contact released and every channel idle at the start,
    everything released at the end (also when the setup itself fails), traffic and board events
    recorded once the starting mark exists."""
    mark = None
    try:
        sb.release_all()
        sb.idle()
        mark = sb.mark()
        yield sb
    finally:
        try:
            sb.release_all()
        finally:
            if mark is not None:
                record("switchbox", sb.since(mark))


# ---- CLI: python -m tinyg_hil.switchbox <command> --port P -------------------------------------

def main(argv=None, **client_kwargs):
    ap = argparse.ArgumentParser(prog="python -m tinyg_hil.switchbox",
                                 description="Switchbox over USB serial (takes the board lock).")
    ap.add_argument("command", choices=["info", "state", "events", "config", "defaults", "release-all", "wifi",
                                        "scan-wifi", "spec-check"])
    ap.add_argument("--port", required=True, help="switchbox serial port (never guessed)")
    ap.add_argument("--ssid", help="for wifi: network name ('' clears); the password is prompted for")
    ap.add_argument("--password-stdin", action="store_true",
                    help="for wifi: read the password from the first line of stdin instead of prompting")
    ap.add_argument("--match-ignore-case", action="store_true",
                    help="for wifi: store the one scanned SSID equal to --ssid ignoring case and surrounding spaces "
                         "(refused if none or several match)")
    ap.add_argument("--show-names", action="store_true", help="for scan-wifi: print SSIDs instead of ***")
    ap.add_argument("--since", type=int, default=0)
    ap.add_argument("--lock-wait", type=float, default=0.0)
    args = ap.parse_args(argv)
    with SwitchBox(args.port, lock_wait=args.lock_wait, purpose=f"cli {args.command}", **client_kwargs) as sb:
        if args.command == "info":
            out = sb.info()
        elif args.command == "state":
            out = sb.state()
        elif args.command == "events":
            out = sb.events(since=args.since, all=True)
        elif args.command == "config":
            out = sb.config()
        elif args.command == "defaults":
            out = sb.set_config(defaults=True)
        elif args.command == "release-all":
            out = sb.release_all()
        elif args.command == "scan-wifi":
            out = sb.scan_wifi()
            out = out if args.show_names else redact(out)
        elif args.command == "spec-check":
            board, local = sb.spec(), load_spec()
            out = {"same": board == local, "board_sha256": spec_digest(board), "checkout_sha256": spec_digest(local)}
        else:
            if args.ssid is None:
                ap.error("wifi needs --ssid")
            ssid = args.ssid
            if ssid and args.match_ignore_case:
                ssid = sb.match_ssid(ssid)
            if not ssid:
                password = ""
            elif args.password_stdin:
                password = sys.stdin.readline().rstrip("\r\n")
            else:
                password = getpass.getpass(f"password for {args.ssid!r} (empty = open network): ")
            out = redact(sb.set_wifi(ssid, password))
            out["matched"] = "exact" if ssid == args.ssid else "ignoring case/spaces"
    json.dump(out, sys.stdout, indent=1)
    print()


if __name__ == "__main__":
    main()
