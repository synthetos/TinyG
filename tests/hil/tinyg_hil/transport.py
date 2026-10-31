"""Serial transport: a reader thread that timestamps bytes, strips XON/XOFF and yields parsed lines."""
import queue
import threading
import time

import serial

from .protocol import XOFF, XON, Event, parse_line


def restart_output(ser):
    """Resume tty output. A board XOFF received while IXON was on leaves the macOS tty's output
    stopped, and that state outlives the process and a board reset: every later write blocks."""
    try:
        import termios
        termios.tcflow(ser.fd, termios.TCOON)
    except (ImportError, OSError, AttributeError):
        pass


class HostOutputStalled(RuntimeError):
    """Bytes accepted by the host serial driver are not leaving it: the host side is wedged, not the board."""


class Transport:
    def __init__(self, port: str, baud: int = 115200, transcript=None):
        self.port, self.baud = port, baud
        self.transcript = transcript        # open text file or None
        self.events: "queue.Queue[Event]" = queue.Queue()
        self.history: list[Event] = []
        self.sent: list[tuple[float, bytes]] = []
        self.flow: list[tuple[float, int]] = []   # (time, XON/XOFF byte) sent by the board
        self.ser = None
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        # Held while bytes are travelling to the board. An Atmel-ICE halt stops the RX interrupt, and
        # the USART keeps only two bytes, so Ice.halted() takes this lock first (Ice(wire=...)).
        self.wire = threading.Lock()
        self.wire_margin = 0.0                  # extra hold after a write; Ice sets it (USB bridge FIFO + 1 ms frames)

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.02, xonxoff=False, rtscts=False, write_timeout=5)
        restart_output(self.ser)
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, name="tinyg-reader", daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)
        if self.ser:
            self.ser.close()
        self.ser = None

    @property
    def is_open(self):
        return self.ser is not None

    def write(self, data: bytes, drain_timeout: float | None = 2.0):
        self.sent.append((time.monotonic(), data))
        self._log(">>", data)
        with self.wire:
            try:
                self.ser.write(data)
            except serial.SerialTimeoutException as e:
                raise HostOutputStalled(self._stall_text()) from e
            if drain_timeout is not None and not self._drained(drain_timeout):
                raise HostOutputStalled(self._stall_text())
            if self.wire_margin:
                time.sleep(len(data) * 10 / self.baud + self.wire_margin)

    def _drained(self, timeout: float) -> bool:
        """Wait until the driver's output queue is empty (the bytes reached the USB bridge)."""
        end = time.monotonic() + timeout
        while self.ser.out_waiting:
            if time.monotonic() > end:
                return False
            time.sleep(0.002)
        return True

    def _stall_text(self) -> str:
        try:
            waiting, cts = self.ser.out_waiting, self.ser.cts
        except (serial.SerialException, OSError):
            waiting, cts = "?", "?"
        return (f"{self.port}: {waiting} byte(s) stuck in the host serial driver (CTS {cts}). The board is not "
                "receiving: the macOS tty/FTDI driver output is stopped (seen after an IXON stream met a board "
                "XOFF). Replug the USB cable; check the board itself with an ICE snapshot.")

    def cts(self) -> bool:
        return self.ser.cts

    def _log(self, direction, data):
        if self.transcript:
            with self._lock:
                self.transcript.write(f"{time.monotonic():.4f} {direction} {data!r}\n")
                self.transcript.flush()

    def _reader(self):
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError, TypeError):
                break   # port closed underneath us (reset, bootloader)
            if not chunk:
                continue
            now = time.monotonic()
            self._log("<<", chunk)
            for b in chunk:
                if b in (XON, XOFF):
                    self.flow.append((now, b))
                elif b == 0x0A:
                    line = buf.decode(errors="replace").rstrip("\r")
                    buf.clear()
                    if line.strip():
                        ev = parse_line(line, now)
                        self.history.append(ev)
                        self.events.put(ev)
                else:
                    buf.append(b)

    def next_event(self, timeout: float) -> Event | None:
        try:
            return self.events.get(timeout=max(timeout, 0))
        except queue.Empty:
            return None

    def drain(self, quiet: float = 0.15, limit: float = 5.0) -> list[Event]:
        """Collect lines until nothing arrives for `quiet` seconds (or `limit` elapses)."""
        out, hard = [], time.monotonic() + limit
        while time.monotonic() < hard:
            ev = self.next_event(quiet)
            if ev is None:
                break
            out.append(ev)
        return out
