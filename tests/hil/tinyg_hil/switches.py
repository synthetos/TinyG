"""Software switches: drive the TinyG's unwired limit/homing/probe inputs through the Atmel-ICE.

Each switch input idles high on its internal pull-up. Switching it to the internal pull-down makes a
real edge on the pin, so the firmware's pin-change ISR, lockout, limit and homing logic run exactly as
with a physical switch (see Ice.set_switch_level).

    sw = SoftSwitches(ice, normally_closed=False)
    sw.close("x", "min"); sw.open("x", "min")
    sw.pulse("z", "min", ms=20)
    sw.mount("z", "min", at=-5.0)        # switch bolted at machine Z=-5: closed while Z <= -5
    ...run G28.2 Z0 over serial...
    sw.unmount_all()

Mounted switches are followed by a background thread that samples the runtime machine position
(board file firmware_symbols.runtime_position, mm) over the ICE. Each sample halts the CPU for a few
milliseconds; the trip lags the crossing by about one sample period (interval + halt), which at homing search speeds is a few hundredths of a mm.
"""
import threading
import time


class SoftSwitches:
    def __init__(self, ice, normally_closed=False, interval=0.02):
        self.ice = ice
        self.normally_closed = normally_closed
        self.interval = interval
        self.closed = {}                    # (axis, end) -> bool, as last driven
        self.mounts = {}                    # (axis, end) -> machine position of the switch
        self.trips = []                     # (time, axis, end, closed, position) for assertions
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    # ---- direct control -------------------------------------------------------------

    def _drive(self, axis, end, closed, position=None):
        with self._lock:
            self.ice.set_switch(axis, end, closed, self.normally_closed)
            self.closed[(axis, end)] = closed
            self.trips.append((time.monotonic(), axis, end, closed, position))

    def close(self, axis, end):
        self._drive(axis, end, True)

    def open(self, axis, end):
        self._drive(axis, end, False)

    def pulse(self, axis, end, ms):
        self.close(axis, end)
        time.sleep(ms / 1000.0)
        self.open(axis, end)

    def release_all(self):
        """Back to the firmware default (pull-up) on every input; stops mounted switches."""
        self.unmount_all()
        with self._lock:
            self.ice.release_all_switches()
            self.closed.clear()

    # ---- mounted switches -----------------------------------------------------------

    def mount(self, axis, end, at):
        """A switch fixed at machine position `at`: a min switch is closed while pos <= at, a max while pos >= at."""
        self.mounts[(axis, end)] = at
        if self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._follow, name="soft-switches", daemon=True)
            self._thread.start()

    def unmount_all(self):
        self.mounts.clear()
        if self._thread is not None:
            self._stop.set()
            self._thread.join(timeout=2)
            self._thread = None

    def position(self, axis):
        with self._lock:
            board = self.ice.board
            return self.ice.peek(f"{board.symbols['runtime_position']}[{board.axis_index(axis)}]")

    def _follow(self):
        while not self._stop.is_set():
            for (axis, end), at in list(self.mounts.items()):
                pos = self.position(axis)
                closed = pos <= at if end == "min" else pos >= at
                if self.closed.get((axis, end)) != closed:
                    self._drive(axis, end, closed, pos)
            self._stop.wait(self.interval)
