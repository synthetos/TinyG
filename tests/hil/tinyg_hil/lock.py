"""Exclusive use of a board: one process at a time per serial port (pytest, flashing, snapshots).

The lock is an flock on /tmp/tinyg-hil-<port name>.lock, so it is released automatically if the
holder dies. The file records who holds it and why; a second user waits or gets that message.
Set TINYG_OWNER (e.g. your name) so the other side knows who is on the board.
"""
import datetime
import fcntl
import json
import os
import pathlib
import time


class BoardBusy(RuntimeError):
    pass


class BoardLock:
    def __init__(self, port: str):
        self.path = pathlib.Path("/tmp") / f"tinyg-hil-{pathlib.Path(port).name}.lock"
        self.fd = None

    def holder(self) -> str:
        try:
            return self.path.read_text().strip() or "unknown"
        except OSError:
            return "unknown"

    def acquire(self, purpose: str, wait: float = 0.0):
        self.fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o666)
        end = time.monotonic() + wait
        while True:
            try:
                fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if time.monotonic() >= end:
                    holder = self.holder()
                    os.close(self.fd)
                    self.fd = None
                    raise BoardBusy(f"board in use: {holder}") from None
                time.sleep(1.0)
        info = {"owner": os.environ.get("TINYG_OWNER", "unknown"), "pid": os.getpid(), "purpose": purpose,
                "since": datetime.datetime.now().isoformat(timespec="seconds")}
        os.ftruncate(self.fd, 0)
        os.write(self.fd, json.dumps(info).encode())
        return self

    def release(self):
        if self.fd is not None:
            os.ftruncate(self.fd, 0)
            fcntl.flock(self.fd, fcntl.LOCK_UN)
            os.close(self.fd)
            self.fd = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.release()
