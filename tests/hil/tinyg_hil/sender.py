"""Stream a G-code file to a TinyG the way hosts do, with each flow-control scheme.

Modes (the board must already be configured for them; see MODES):
  line   rxm=1 packet mode: send while the last footer's free-slot count, less the lines sent since,
         leaves RESERVE slots for control commands (wiki: RX packet mode)
  xon    rxm=0, ex=1: the host tty honours the board's XON/XOFF (termios IXON) and never sends its own
         (IXOFF off). pyserial's xonxoff sets both: with IXOFF the macOS tty sent the board an XOFF and
         never the XON, the board's TX stayed paused (fc_state_tx FC_IN_XOFF), _sync_to_tx_buffer held
         the main loop and both ends waited forever. Gating
         writes in host software is not safe behind a USB-serial bridge: after XOFF arrives, bytes
         already queued in the bridge's TX FIFO (up to 128 on an FT232R) still go out, more than the
         50 bytes of headroom above the board's XOFF mark (80% of 250), and the RX ring overflows.
  rts    rxm=0, ex=2: the serial port enforces CTS in hardware (pyserial rtscts)
  count  rxm=0, ex=0: Grbl-style character counting - unacknowledged bytes stay under the RX buffer

In JSON mode (ej=1) every non-blank line sent gets one {"r":...,"f":[...]} response; responses are
matched to lines in order. A response with no line outstanding, or without a valid footer, fails the
stream (StreamProtocolError): it must not be taken as credit. Status reports are collected with their
arrival time for path reconstruction. After the last answer the stream waits for the machine to stop;
StreamResult.completion says how that ended, and only "idle" is a finished program.
"""
from dataclasses import dataclass, field
import time

MODES = {"line": {"rxm": 1, "ex": 1}, "g2": {"rxm": 1, "ex": 1}, "xon": {"rxm": 0, "ex": 1},
         "xon-app": {"rxm": 0, "ex": 1}, "rts": {"rxm": 0, "ex": 2}, "count": {"rxm": 0, "ex": 0},
         "count-crlf": {"rxm": 0, "ex": 0}}
# count-crlf: a byte-counting host fed a Windows file - CRLF endings and a blank line after every 7th
# line - that keeps (bytes sent - bytes the footers report removed) under RX_COUNT_LIMIT. Exact only if
# every byte read is reported, answered line or not (E164); otherwise it drifts until it stalls.
G2_PRIME = 4                # g2 mode: g2core's line-mode protocol - send 4, then one line per {r:} response
# "blast": no flow control and no pacing - a misbehaving host, used to force RX overruns on purpose
BLAST = {"rxm": 0, "ex": 0}
RESERVE = 2                 # line mode: slots kept free for control commands
RX_COUNT_LIMIT = 200        # count mode: bytes in flight (RX ring is 250, leave headroom)
CHUNK = 16                  # xon mode: write granularity, so an XOFF stops us within a few bytes
XOFF, XON = 0x13, 0x11
STALL = 5.0                 # s with lines outstanding and neither a response nor a status report: hung
TTY_ROOM = 256              # rts/xon: bytes allowed in the host driver's output queue before a write
TTY_WRITE_TIMEOUT = 1.0     # rts/xon: a write that still blocks (queue full anyway) fails after this
IDLE_STATS = (1, 3, 4)      # READY, PROGRAM_STOP, PROGRAM_END: the program finished
END_STATS = {2: "alarm", 12: "shutdown", 13: "panic"}   # nothing more will run


class StreamFailed(AssertionError):
    """A stream that did not run through; .result holds the partial StreamResult."""


class StreamStalled(StreamFailed):
    pass


class StreamRunaway(StreamFailed):
    """A status report put the machine outside the program's bounding box: it was stopped at once."""


class StreamProtocolError(StreamFailed):
    """A response that cannot be matched to a sent line (none outstanding, or no valid footer)."""


class StreamLineTooLong(ValueError):
    """Count mode: a line longer than RX_COUNT_LIMIT can never be sent inside the byte budget."""


# Firmware state worth reading over the Atmel-ICE when a stream hangs (one halt, consistent snapshot)
# USB is ds[XIO_DEV_USB] and USBu is us[XIO_DEV_USB - XIO_DEV_USART_OFFSET]: index 0 on the v8 (xio.h)
STALL_SYMBOLS = ["rtc.sys_ticks", "xio.buf_state", "cm.queue_flush_input", "cm.queue_flush_input_due", "us[0].rx_buf_head", "us[0].rx_buf_tail", "us[0].rx_buf_count", "us[0].fc_state_rx", "us[0].fc_state_tx",
                 "ds[0].flag_in_line", "ds[0].len", "ds[0].flag_xoff", "xio.rx_mode", "xio.enable_flow_control",
                 "mb.buffers_available", "cm.machine_state", "cm.cycle_state", "cm.motion_state", "cm.hold_state",
                 "cm.queue_flush_state", "cs.controller_state"]


def _ixon_only(ser):
    """Obey XOFF/XON from the board (IXON) without ever sending them to it (no IXOFF, no IXANY)."""
    import termios
    attrs = termios.tcgetattr(ser.fd)
    saved = list(attrs)
    attrs[0] = (attrs[0] | termios.IXON) & ~(termios.IXOFF | termios.IXANY)
    termios.tcsetattr(ser.fd, termios.TCSANOW, attrs)
    return saved


def _restore_tty(ser, saved):
    import termios
    termios.tcsetattr(ser.fd, termios.TCSANOW, saved)
    termios.tcflow(ser.fd, termios.TCOON)     # an XOFF that arrived under IXON must not leave output stopped


FC = {0: "disabled", 1: "XON (flowing)", 2: "XOFF (paused)"}


def diagnose(snap):
    """Plain-language findings from a stall_snapshot()."""
    out = []
    if snap.get("us[0].fc_state_tx") == 2:
        out.append("board TX is paused by an XOFF it received from the host; nothing it sends can get out")
    if snap.get("us[0].fc_state_rx") == 2:
        out.append("board has sent XOFF to the host (its RX ring is above the high-water mark)")
    count = snap.get("us[0].rx_buf_count")
    if count is not None and count >= 200:
        out.append(f"RX ring nearly full ({count}/250 bytes) - input is waiting but not being read")
    if snap.get("xio.buf_state") == 3:
        out.append("char-mode reader holds a completed DATA line that only the command dispatcher can take")
    if snap.get("mb.buffers_available") == 32 and snap.get("cm.cycle_state") == 0:
        out.append("planner is empty and no cycle is running: the board is idle, not busy")
    if snap.get("cm.machine_state") in (2, 12, 13):
        out.append(f"machine state {snap['cm.machine_state']} (ALARM/SHUTDOWN/PANIC) rejects input")
    if snap.get("cm.queue_flush_input") or snap.get("cm.queue_flush_input_due"):
        out.append("a host '%' flush is pending: the dispatchers skip input until it completes")
    return out or ["no known stall signature"]


def stall_snapshot(ice):
    """Halt once and read STALL_SYMBOLS plus the bytes waiting in the USB RX ring; unknown names are skipped."""
    names = [n for n in STALL_SYMBOLS if _resolvable(ice, n)]
    snap = ice.peek_many(names)
    try:
        head, tail = snap.get("us[0].rx_buf_head"), snap.get("us[0].rx_buf_tail")
        ring = ice.peek("us[0].rx_buf") if _resolvable(ice, "us[0].rx_buf") else None
        snap["rx_ring"] = ring.hex() if isinstance(ring, (bytes, bytearray)) else ring
        snap["rx_head_tail"] = (head, tail)
        snap["diagnosis"] = diagnose(snap)
    except Exception as e:  # noqa: BLE001 - a partial snapshot is still worth keeping
        snap["rx_ring_error"] = str(e)
    return snap


def _resolvable(ice, name):
    try:
        ice.symbols.resolve(name)
        return True
    except Exception:  # noqa: BLE001
        return False


@dataclass
class StreamResult:
    mode: str
    lines: int = 0
    acked: int = 0
    errors: list = field(default_factory=list)      # (line index, status, text)
    status: list = field(default_factory=list)      # (t, merged sr dict) after each report
    queue: list = field(default_factory=list)       # (t, free planner buffers) from queue reports (qv=1)
    footers: list = field(default_factory=list)     # (f[2], status) of each line response, in order
    exceptions: list = field(default_factory=list)
    started: float = 0.0
    sent_done: float = 0.0
    finished: float = 0.0
    max_inflight: int = 0
    xoff_count: int = 0
    min_free: int | None = None
    completion: str = ""            # idle | alarm | shutdown | panic | timeout (after every line was answered)

    @property
    def ok(self):
        """Every line answered with status 0, no exception report, and the machine stopped normally."""
        return (self.acked == self.lines and not self.errors and not self.exceptions
                and self.completion == "idle")


def _with(res, exc):
    exc.result = res              # the partial result, for diagnosis
    return exc


def splices(res, lines):
    """Char mode only: footer f[2] is the processed line's length + 1. Walk responses against the sent
    lines and report each response whose length is not its line's - two lines run together (bytes lost
    in between) - with its status: status 0 means the corrupted line was executed."""
    todo = _payload(lines)
    want = [len(t) + 1 for t in todo]
    got = [(ev_len, st) for ev_len, st in res.footers]
    out, k = [], 0
    for idx, (length, status) in enumerate(got):
        if k < len(want) and length == want[k]:
            k += 1
            continue
        # find how many sent lines this response swallowed (best effort: up to 4)
        out.append({"response": idx, "line": k, "len": length, "expected": want[k] if k < len(want) else None,
                    "status": status, "text": todo[k] if k < len(todo) else None})
        k += 2 if k + 1 < len(want) else 1
    return out


def _outside(sr, bounds):
    (x0, x1), (y0, y1) = bounds
    x, y = sr.get("posx"), sr.get("posy")
    return (x is not None and not x0 <= x <= x1) or (y is not None and not y0 <= y <= y1)


def queue_profile(res, buffers=32):
    """Planner occupancy while lines were being sent: time-weighted share of the send phase with the
    queue full (<= 4 free), empty (>= buffers - 4 free) and the median free count."""
    q = [(t, v) for t, v in res.queue if res.started <= t <= (res.sent_done or res.finished)]
    if len(q) < 2:
        return {"samples": len(q)}
    spans = [(b[0] - a[0], a[1]) for a, b in zip(q, q[1:])]
    total = sum(d for d, _ in spans) or 1.0
    ordered = sorted(v for _, v in q)
    return {"samples": len(q), "median_free": ordered[len(ordered) // 2],
            "full_share": round(sum(d for d, v in spans if v <= 4) / total, 3),
            "empty_share": round(sum(d for d, v in spans if v >= buffers - 4) / total, 3)}


def _payload(lines):
    """Non-blank lines as sent: comments and blank lines are kept (hosts send them); blank ones get no
    response, so only lines with content are counted."""
    out = []
    for line in lines:
        text = line.rstrip("\r\n")
        if text.strip() and text.strip() != "%":
            out.append(text)
    return out


def _footer(ev):
    """(status, count) from a response's footer, or None when it has no usable footer."""
    f = ev.data.get("f") if isinstance(ev.data, dict) else None
    if not isinstance(f, list) or len(f) < 3:
        return None
    if not all(isinstance(v, int) and not isinstance(v, bool) for v in f[:3]):
        return None
    return f[1], f[2]


def _wire(text, i, mode):
    if mode == "count-crlf":
        return (text + "\r\n").encode() + (b"\r\n" if i % 7 == 6 else b"")
    return (text + "\n").encode()


def stream(tg, lines, mode, timeout=600.0, on_status=None, bounds=None):
    """Send `lines` with `mode`; return a StreamResult once every line is answered and the machine stops.

    The result's completion records how the wait for the machine ended; res.ok is the whole verdict.
    bounds = ((xmin, xmax), (ymin, ymax)) in work coordinates: a status report outside them sends a
    feedhold and a queue flush immediately and raises StreamRunaway (a corrupted line must not run on)."""
    tr = tg.transport
    todo = _payload(lines)
    if mode in ("count", "count-crlf"):
        for k, text in enumerate(todo):
            if len(_wire(text, k, mode)) > RX_COUNT_LIMIT:
                raise StreamLineTooLong(f"{mode}: line {k} is {len(_wire(text, k, mode))} bytes, more than the "
                                        f"{RX_COUNT_LIMIT}-byte budget: {text[:60]!r}")
    res = StreamResult(mode=mode, lines=len(todo))
    sr = {k: tg.get(k) for k in ("posx", "posy", "posz")}   # filtered reports carry only what changed: start known
    inflight = []                    # byte lengths (count mode) or 1s (line mode), oldest first
    free = tg.get("rx") if mode == "line" else None   # free line slots before the first footer
    credit = G2_PRIME                # g2 mode: lines the host may send; +1 per response, ignoring the footer
    sent_bytes = reported = 0        # count-crlf: bytes written, bytes the footers reported removed
    flow_seen = len(tr.flow)
    xoff = False
    res.started = time.monotonic()
    deadline = res.started + timeout
    progress = res.started           # last response or status report
    i = 0
    saved_write_timeout = tr.ser.write_timeout
    if mode == "rts":
        tr.ser.rtscts = True
    elif mode == "xon":
        saved_tty = _ixon_only(tr.ser)   # the tty now consumes the board's XON/XOFF; the reader no longer sees them
    if mode in ("rts", "xon"):
        # The board may hold host output back for as long as it is full. Lines are written only while the
        # driver's queue has room, so a write does not block and this loop keeps reading responses and
        # status reports and running the stall watchdog; a write that blocks anyway fails after the timeout.
        tr.ser.write_timeout = TTY_WRITE_TIMEOUT

    def _report(ev):
        """Status, queue and exception reports, the same for the whole stream including the final wait:
        a status outside bounds stops the machine at once. Returns False for anything else."""
        if ev.kind == "status" and isinstance(ev.body, dict):
            sr.update(ev.body)
            if bounds and _outside(sr, bounds):
                tr.write(b"!", drain_timeout=None)
                time.sleep(0.3)
                tr.write(b"%", drain_timeout=None)
                raise _with(res, StreamRunaway(f"{mode}: machine left the program's box at line {res.acked} of {res.lines} "
                                    f"(last sent {todo[i - 1] if i else None!r}): status {sr}; feedhold and flush sent"))
            res.status.append((ev.t, dict(sr, _raw=ev.raw, _damaged=bool(getattr(ev, "damaged", False)))))
            if on_status:
                on_status(ev.t, sr)
        elif ev.kind == "queue" and isinstance(ev.data.get("qr"), (int, float)):
            res.queue.append((ev.t, ev.data["qr"]))
        elif ev.kind == "exception":
            res.exceptions.append(ev.raw)
        else:
            return False
        return True

    try:
        while res.acked < res.lines:
            now = time.monotonic()
            if now > deadline:
                raise TimeoutError(f"{mode}: {res.acked}/{res.lines} answered after {timeout}s")
            if now - progress > STALL:
                raise _with(res, StreamStalled(f"{mode}: stalled {STALL:.0f} s with {len(inflight)} line(s) unanswered at line "
                                    f"{res.acked} of {res.lines} ({todo[res.acked]!r}); {i} sent, last status {sr}"))
            # flow bytes from the board (the reader thread strips them from the line stream)
            while flow_seen < len(tr.flow):
                xoff = tr.flow[flow_seen][1] == XOFF
                res.xoff_count += xoff
                flow_seen += 1
            # send as much as the scheme allows
            while i < res.lines:
                data = _wire(todo[i], i, mode)
                if mode == "line":
                    room = free - len(inflight)
                    if room <= RESERVE:
                        break
                elif mode == "count":
                    if sum(inflight) + len(data) > RX_COUNT_LIMIT:
                        break
                elif mode == "count-crlf":
                    if sent_bytes - reported + len(data) > RX_COUNT_LIMIT:
                        break
                elif mode == "g2" and credit <= 0:
                    break
                elif mode == "xon-app" and xoff:
                    break                   # the application honours XOFF, as terminal programs do
                elif mode in ("rts", "xon") and tr.ser.out_waiting + len(data) > TTY_ROOM:
                    break                   # the board is holding output back: read and watch, write later
                tr.write(data, drain_timeout=None if mode in ("rts", "xon") else 2.0)
                inflight.append(len(data) if mode == "count" else 1)
                credit -= 1
                sent_bytes += len(data)
                res.max_inflight = max(res.max_inflight, len(inflight))
                i += 1
                if i == res.lines:
                    res.sent_done = time.monotonic()
                if mode in ("xon", "xon-app", "rts"):
                    break                   # one line per pass so responses are read promptly
            # read whatever arrived
            ev = tr.next_event(0.005)
            while ev is not None:
                if ev.kind in ("response", "status", "queue"):
                    progress = ev.t
                if not _report(ev) and ev.kind == "response":
                    footer = _footer(ev)
                    if i <= res.acked:
                        raise _with(res, StreamProtocolError(f"{mode}: response with no line outstanding "
                                                             f"({res.acked} answered, {i} sent): {ev.raw[:120]}"))
                    if footer is None:
                        raise _with(res, StreamProtocolError(f"{mode}: response to line {res.acked} "
                                                             f"({todo[res.acked]!r}) has no valid footer: {ev.raw[:120]}"))
                    status, count = footer
                    res.footers.append((count, status))
                    if status != 0:
                        res.errors.append((res.acked, status, todo[res.acked]))
                    inflight.pop(0)
                    credit += 1
                    reported += count
                    if mode == "line":
                        free = count
                        res.min_free = free if res.min_free is None else min(res.min_free, free)
                    res.acked += 1
                ev = tr.next_event(0)
        # all answered: wait for motion to finish, polling stat and still collecting status reports
        asked = 0.0
        res.completion = "timeout"          # unless the machine is seen to stop
        while time.monotonic() < deadline:
            if time.monotonic() - asked > 0.3:
                tr.write(b'{"stat":null}\n')
                asked = time.monotonic()
            ev = tr.next_event(0.05)
            if ev is None:
                continue
            if _report(ev) or ev.kind != "response":
                continue
            # only stat polls are outstanding now: each answer needs a valid footer with status 0
            footer = _footer(ev)
            stat = ev.body.get("stat") if isinstance(ev.body, dict) else None
            if footer is None or footer[0] != 0 or not isinstance(stat, int):
                raise _with(res, StreamProtocolError(f"{mode}: stat poll after the last line got an invalid "
                                                     f"or failed answer: {ev.raw[:120]}"))
            if stat in IDLE_STATS:
                res.completion = "idle"
                break
            if stat in END_STATS:
                res.completion = END_STATS[stat]
                break                       # nothing more will run: don't wait it out
        res.finished = time.monotonic()
    finally:
        tr.ser.write_timeout = saved_write_timeout
        if mode == "rts":
            tr.ser.rtscts = False
        elif mode == "xon":
            _restore_tty(tr.ser, saved_tty)
    return res
