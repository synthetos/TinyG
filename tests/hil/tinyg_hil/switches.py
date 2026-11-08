"""Software switches: drive the TinyG's unwired limit/homing/probe inputs through the Atmel-ICE.

A closed switch is simulated by driving its input low (or, for normally-closed switches, an open one
by releasing it); see Ice.set_switch_level. On a test-hooks build (make TEST_HOOKS=1) the running
firmware makes the change, so the port's edge detector, the switch ISR, lockout, limit and homing
logic run exactly as with a physical switch.

    sw = SoftSwitches(ice, normally_closed=False, geometry=fn)
    sw.idle()                             # every switch untripped for its type
    sw.close("x", "min"); sw.open("x", "min")     # close = trip, open = untrip
    sw.pulse("z", "min", ms=20)
    sw.mount("z", "min", at=-5.0)        # switch bolted 5 mm below the current machine zero
    ...run G28.2 Z0 over serial...
    sw.mount_status("z", "min")           # {"closed", "trips", "trip_at"} in the mount's frame
    sw.unmount_all()

Mounts on a test-hooks build live in the firmware: it counts every motor step since power-up (a
count homing and G92 never rewrite) and flips the pin from the main loop when the count crosses
the switch, so the switch stays put in physical space while homing redefines machine coordinates,
and nothing halts the CPU during motion. `at` is in machine units relative to the machine position
when mount() is called; geometry(axis) -> (motor index, steps per unit) places it.

Without the hook, mounted switches fall back to a background thread that samples the runtime machine
position (board file firmware_symbols.runtime_position) over the ICE; each sample halts the CPU, the
trip lags the crossing by about one sample period, and homing's position rewrite moves the switch.
"""
import threading
import time


class SoftSwitches:
    def __init__(self, ice, normally_closed=False, interval=0.02, geometry=None):
        self.ice = ice
        self.geometry = geometry            # axis -> (motor index 0.., steps per unit), for firmware mounts
        self.frames = {}                    # (axis, end) -> (steps at mount, position at mount, steps per unit, motor)
        self.normally_closed = normally_closed
        self.interval = interval
        self.closed = {}                    # (axis, end) -> bool, as last driven
        self.mounts = {}                    # (axis, end) -> machine position of the switch
        self.trips = []                     # (time, axis, end, closed, position) for assertions
        self.samples = []                   # (time, axis, position) read by the mounted-switch follower
        self.error = None                   # exception that stopped the follower thread
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

    def idle(self):
        """Every switch untripped for the configured type: NO released (high), NC closed (driven low)."""
        with self._lock:
            for axis in self.ice.board.switches:
                for end in ("min", "max"):
                    self.ice.set_switch(axis, end, False, self.normally_closed)
                    self.closed[(axis, end)] = False

    def release_all(self):
        """Back to the board's own input configuration on every pin; stops mounted switches."""
        self.unmount_all()
        with self._lock:
            self.ice.release_all_switches()
            self.closed.clear()

    # ---- mounted switches -----------------------------------------------------------

    def _firmware_mounts(self):
        return self.ice._uses_hook() and self.geometry is not None and \
            "test_hook" in self.ice.symbols.types and self._has_member("mount")

    def _has_member(self, name):
        try:
            self.ice.symbols.resolve(f"test_hook.{name}")
            return True
        except KeyError:
            return False

    def _mount_index(self, axis, end):
        return list(self.ice.board.switches).index(axis) * 2 + (1 if end == "max" else 0)

    def mount(self, axis, end, at):
        """A switch fixed at machine position `at`: a min switch is closed while pos <= at, a max while pos >= at."""
        if self._firmware_mounts():
            self._mount_in_firmware(axis, end, at)
            return
        self.mounts[(axis, end)] = at
        if self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._follow, name="soft-switches", daemon=True)
            self._thread.start()

    def _mount_in_firmware(self, axis, end, at):
        motor, spu = self.geometry(axis)
        board, i = self.ice.board, self._mount_index(axis, end)
        here = self.ice.peek_many([f"test_hook.steps[{motor}]", f"en.en[{motor}].steps_run",
                                   f"{board.symbols['runtime_position']}[{board.axis_index(axis)}]"])
        accumulated, running, pos = here.values()
        steps = accumulated + running
        tripped, untripped = (1, 0) if self.normally_closed else (0, 1)
        action = {0: self.ice.HOOK_DRIVE_LOW, 1: self.ice.HOOK_RELEASE}
        m = f"test_hook.mount[{i}]"
        with self.ice.halted():
            self.ice.poke(f"{m}.enabled", 0)
            self.ice.poke(f"{m}.motor", motor)
            self.ice.poke(f"{m}.sense", -1 if end == "min" else 1)
            self.ice.poke(f"{m}.closed_action", action[tripped])
            self.ice.poke(f"{m}.open_action", action[untripped])
            self.ice.poke(f"{m}.closed", 0xFF)          # unknown: the firmware applies the current state
            self.ice.poke(f"{m}.trips", 0)
            self.ice.poke(f"{m}.at", round(steps + (at - pos) * spu))
            self.ice.poke(f"{m}.enabled", 1)
        self.frames[(axis, end)] = (steps, pos, spu, motor)
        time.sleep(0.02)

    def mount_status(self, axis, end):
        """Firmware mount state; trip_at is the last trip position in the frame mount() was given."""
        steps0, pos0, spu, _ = self.frames[(axis, end)]
        m = f"test_hook.mount[{self._mount_index(axis, end)}]"
        v = self.ice.peek_many([f"{m}.closed", f"{m}.trips", f"{m}.trip_steps"])
        trips = v[f"{m}.trips"]
        result = {"closed": bool(v[f"{m}.closed"]), "trips": trips,
                  "trip_at": pos0 + (v[f"{m}.trip_steps"] - steps0) / spu if trips else None}
        if self._has_member(f"mount[{self._mount_index(axis, end)}].min_steps"):
            extrema = self.ice.peek_many([f"{m}.min_steps", f"{m}.max_steps"])
            result.update(min_position=pos0 + (extrema[f"{m}.min_steps"] - steps0) / spu,
                          max_position=pos0 + (extrema[f"{m}.max_steps"] - steps0) / spu)
        return result

    def physical_position(self, axis):
        """Position in the mount frame from the never-rewritten step count (test-hooks builds)."""
        steps0, pos0, spu, motor = next(f for (a, _), f in self.frames.items() if a == axis)
        v = self.ice.peek_many([f"test_hook.steps[{motor}]", f"en.en[{motor}].steps_run"])
        return pos0 + (sum(v.values()) - steps0) / spu  # one halt; include the last segment's pulses

    def unmount_all(self):
        if self.frames:
            with self.ice.halted():
                for (axis, end) in self.frames:
                    self.ice.poke(f"test_hook.mount[{self._mount_index(axis, end)}].enabled", 0)
            self.frames.clear()
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
        try:
            self._follow_loop()
        except Exception as e:              # surfaced to the test via .error (a thread cannot raise into it)
            self.error = e

    def _follow_loop(self):
        while not self._stop.is_set():
            for (axis, end), at in list(self.mounts.items()):
                pos = self.position(axis)
                self.samples.append((time.monotonic(), axis, pos))
                closed = pos <= at if end == "min" else pos >= at
                if self.closed.get((axis, end)) != closed:
                    self._drive(axis, end, closed, pos)
            self._stop.wait(self.interval)
