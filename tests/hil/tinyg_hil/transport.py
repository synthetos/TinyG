"""Serial transport: a reader thread that timestamps bytes, strips XON/XOFF and yields parsed lines."""
import queue
import threading
import time

import serial

from .protocol import XOFF, XON, Event, parse_line


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

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.02, xonxoff=False, rtscts=False)
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

    def write(self, data: bytes):
        self.sent.append((time.monotonic(), data))
        self._log(">>", data)
        self.ser.write(data)
        self.ser.flush()

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
