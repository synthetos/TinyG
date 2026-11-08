"""Keep a serial stream awake; invalidate its credits after forced host suspension."""
import ctypes
from functools import lru_cache
import os
import subprocess
import sys
import time


class HostSuspended(TimeoutError):
    """USB delivery across host suspend is unknown; the stream must be abandoned."""


@lru_cache(maxsize=1)
def _continuous_clock():
    if sys.platform == "darwin":
        # Apple's mach_time.h: like mach_absolute_time, but advances during sleep.
        lib = ctypes.CDLL(None)

        class Timebase(ctypes.Structure):
            _fields_ = [("numer", ctypes.c_uint32), ("denom", ctypes.c_uint32)]

        info = Timebase()
        lib.mach_timebase_info.argtypes = [ctypes.POINTER(Timebase)]
        lib.mach_timebase_info.restype = ctypes.c_int
        if lib.mach_timebase_info(ctypes.byref(info)) != 0 or not info.denom:
            raise RuntimeError("cannot read the host continuous clock")
        clock = lib.mach_continuous_time
        clock.argtypes = []
        clock.restype = ctypes.c_uint64
        scale = info.numer / info.denom / 1e9
        return lambda: clock() * scale
    if hasattr(time, "CLOCK_BOOTTIME"):
        return lambda: time.clock_gettime(time.CLOCK_BOOTTIME)
    return None


class AwakeSession:
    def __init__(self):
        self._child = None
        self._clock = None
        self._offset = None

    def start(self):
        if sys.platform == "darwin":
            # The pid lease also ends if the test process is killed. This does not
            # override forced sleep (thermal protection, lid, or explicit sleep).
            self._child = subprocess.Popen(
                ["/usr/bin/caffeinate", "-is", "-w", str(os.getpid())],
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._clock = _continuous_clock()
        if self._clock is not None:
            self._offset = self._clock() - time.monotonic()

    def check(self):
        if self._child is not None and self._child.poll() is not None:
            raise HostSuspended("host sleep protection exited; serial stream is invalid")
        if self._clock is not None:
            gap = (self._clock() - time.monotonic()) - self._offset
            if gap > 1.0:
                raise HostSuspended(f"host suspended for approximately {gap:.1f} s; "
                                    "serial credits are invalid, reset before continuing")

    def close(self):
        child, self._child = self._child, None
        if child is not None:
            if child.poll() is None:
                child.terminate()
            try:
                child.wait(timeout=1)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=1)
