"""Exclusive use of a board: one process at a time per serial port (pytest, flashing, snapshots).

Both persistent and legacy /tmp locks are held during the transition, so old and new
worktrees exclude each other. Locks are released automatically if the holder dies. The file records who holds it and why; a second user waits or gets that message.
Set TINYG_OWNER (e.g. your name) so the other side knows who is on the board.
"""
import datetime
import fcntl
import json
import os
import time

from .state import port_path


class BoardBusy(RuntimeError):
    pass


class BoardLock:
    def __init__(self, port: str):
        self.path = port_path(port, "lock")
        self.legacy_path = port_path(port, "lock", legacy=True)
        self.fd = None
        self._fds = []
        self._blocked_path = self.path

    def holder(self) -> str:
        try:
            return self._blocked_path.read_text().strip() or "unknown"
        except OSError:
            return "unknown"

    def acquire(self, purpose: str, wait: float = 0.0):
        if self._fds:
            raise RuntimeError("this BoardLock is already held")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        end = time.monotonic() + wait
        # Never unlink a lock file: an old process may still hold its inode.
        paths = list(dict.fromkeys((self.legacy_path, self.path)))
        try:
            for path in paths:
                fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o666)
                try:
                    while True:
                        try:
                            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                            break
                        except BlockingIOError:
                            self._blocked_path = path
                            remaining = end - time.monotonic()
                            if remaining <= 0:
                                raise BoardBusy(f"board in use: {self.holder()}") from None
                            time.sleep(min(0.1, remaining))
                except BaseException:
                    os.close(fd)
                    raise
                self._fds.append(fd)
            self.fd = self._fds[-1]
            info = {"owner": os.environ.get("TINYG_OWNER", "unknown"), "pid": os.getpid(), "purpose": purpose,
                    "since": datetime.datetime.now().isoformat(timespec="seconds")}
            for fd in self._fds:
                os.ftruncate(fd, 0)
                os.write(fd, json.dumps(info).encode())
            self._blocked_path = self.path
            return self
        except BaseException:
            self.release()
            raise

    def release(self):
        while self._fds:
            fd = self._fds.pop()
            try:
                os.ftruncate(fd, 0)
            finally:
                os.close(fd)  # closing releases flock even on an error
        self.fd = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.release()
