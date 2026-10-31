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

Every non-blank line gets exactly one {"r":...} response in JSON mode (ej=1); responses are matched to
lines in order. Status reports are collected with their arrival time for path reconstruction.
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


class StreamStalled(AssertionError):
    pass


class StreamRunaway(AssertionError):
    """A status report put the machine outside the program's bounding box: it was stopped at once."""


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


def stream(tg, lines, mode, timeout=600.0, on_status=None, bounds=None):
    """Send `lines` with `mode`; return a StreamResult once every line is answered and motion stops.

    bounds = ((xmin, xmax), (ymin, ymax)) in work coordinates: a status report outside them sends a
    feedhold and a queue flush immediately and raises StreamRunaway (a corrupted line must not run on)."""
    tr = tg.transport
    todo = _payload(lines)
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
        tr.ser.write_timeout = None     # CTS may hold writes back for as long as the board is full
    elif mode == "xon":
        saved_tty = _ixon_only(tr.ser)   # the tty now consumes the board's XON/XOFF; the reader no longer sees them
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
                data = (todo[i] + "\n").encode()
                if mode == "count-crlf":
                    data = (todo[i] + "\r\n").encode() + (b"\r\n" if i % 7 == 6 else b"")
                if mode == "line":
                    room = free - len(inflight)
                    if room <= RESERVE:
                        break
                elif mode == "count":
                    if inflight and sum(inflight) + len(data) > RX_COUNT_LIMIT:
                        break
                elif mode == "count-crlf":
                    if sent_bytes - reported and sent_bytes - reported + len(data) > RX_COUNT_LIMIT:
                        break
                elif mode == "g2" and credit <= 0:
                    break
                elif mode == "xon-app" and xoff:
                    break                   # the application honours XOFF, as terminal programs do
                # rts/xon: the board may hold host output back for a long time; the stall watchdog covers them
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
                if ev.kind == "status" and isinstance(ev.body, dict):
                    sr.update(ev.body)
                    if bounds and _outside(sr, bounds):
                        tr.write(b"!", drain_timeout=None)
                        time.sleep(0.3)
                        tr.write(b"%", drain_timeout=None)
                        raise _with(res, StreamRunaway(f"{mode}: machine left the program's box at line {res.acked} of {res.lines} "
                                            f"(last sent {todo[i - 1]!r}): status {sr}; feedhold and flush sent"))
                    res.status.append((ev.t, dict(sr, _raw=ev.raw, _damaged=bool(getattr(ev, "damaged", False)))))
                    if on_status:
                        on_status(ev.t, sr)
                elif ev.kind == "queue" and isinstance(ev.data.get("qr"), (int, float)):
                    res.queue.append((ev.t, ev.data["qr"]))
                elif ev.kind == "exception":
                    res.exceptions.append(ev.raw)
                elif ev.kind == "response":
                    f = ev.data.get("f", [0, 0, 0])
                    res.footers.append((f[2] if len(f) > 2 else None, f[1]))
                    if f[1] != 0:
                        res.errors.append((res.acked, f[1], todo[res.acked] if res.acked < res.lines else ev.raw))
                    if inflight:
                        inflight.pop(0)
                    credit += 1
                    if len(f) > 2 and isinstance(f[2], int):
                        reported += f[2]
                    if mode == "line" and len(f) > 2:
                        free = f[2]
                        res.min_free = free if res.min_free is None else min(res.min_free, free)
                    res.acked += 1
                ev = tr.next_event(0)
        # all answered: wait for motion to finish, polling stat and still collecting status reports
        asked = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() - asked > 0.3:
                tr.write(b'{"stat":null}\n')
                asked = time.monotonic()
            ev = tr.next_event(0.05)
            if ev is None:
                continue
            if ev.kind == "status" and isinstance(ev.body, dict):
                sr.update(ev.body)
                res.status.append((ev.t, dict(sr)))
            elif ev.kind == "queue" and isinstance(ev.data.get("qr"), (int, float)):
                res.queue.append((ev.t, ev.data["qr"]))
            elif ev.kind == "response" and isinstance(ev.body, dict) and ev.body.get("stat") in (1, 3, 4):
                break
            elif ev.kind == "response" and isinstance(ev.body, dict) and ev.body.get("stat") in (2, 12, 13):
                break                       # alarmed, shutdown or panicked: nothing more will run, don't wait it out
        res.finished = time.monotonic()
    finally:
        tr.ser.write_timeout = saved_write_timeout
        if mode == "rts":
            tr.ser.rtscts = False
        elif mode == "xon":
            _restore_tty(tr.ser, saved_tty)
    return res
