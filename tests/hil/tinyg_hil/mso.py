"""Saleae Logic MSO captures (bench 3) through saleae-mso-api, in a worker process.

Logic 2's automation API and MCP server do not drive the Logic MSO, so this module does not use them. Saleae's
MSO package needs Python 3.11+ and numpy and is published on Saleae's own index, so it is not a harness
dependency: tinyg_hil.mso runs tinyg_hil/mso_worker.py with `uv run --no-project --with <wheel> --with numpy`.

    with Capture(bench["mso"], digital=["M1 step", "M1 dir"], analog=["spindle PWM"], seconds=6,
                 save_as=stem) as cap:
        ...move the machine...
    edges = cap.edges()                  # {name: [(t_seconds, level), ...]}, like tinyg_hil.saleae
    pwm = cap.pwm("spindle PWM")         # [{"t", "duty", "freq_hz"}, ...] per window (computed in the worker)

Logic 2 opens every Saleae device it sees, the MSO too, unless it was started with
SALEAE_IGNORE_DEVICE="Logic MSO" (~/.config/tinyg-hil/start-logic2.sh does that); otherwise the worker fails with
"exclusive access and device already open". The MSO captures for a fixed time from when it starts recording.
Each capture re-initializes the MSO and waits for its Smart Cables (8 s on 2026-11-04), so entering the block
waits for the worker's STARTED line, taken from the device log; `started_wall` is that time (local clock, within
tens of ms). A short pre-roll inside the block is still wise.
"""
import json
import os
import pathlib
import subprocess
import tempfile
import time

WORKER = pathlib.Path(__file__).resolve().parent / "mso_worker.py"
DEFAULT_WHEEL = "~/.config/tinyg-hil/wheels/saleae_mso_api-0.5.8-py3-none-any.whl"
EXCLUSIVE_HINT = ("the MSO is held by another program; start Logic 2 with SALEAE_IGNORE_DEVICE=\"Logic MSO\" "
                  "(~/.config/tinyg-hil/start-logic2.sh)")


class MsoError(RuntimeError):
    pass


def _lookup(mapping, wanted):
    """Names from the bench map ({"p0c0": "M1 step"} or {"0": "spindle PWM"}) or raw keys, to raw keys."""
    rev = {v: k for k, v in (mapping or {}).items()}
    out = []
    for w in wanted:
        key = rev.get(w, w)
        if key not in (mapping or {}) and w not in rev:
            raise MsoError(f"MSO channel {w!r} is not in the bench map {sorted(mapping or {})}")
        out.append(key)
    return out


def worker_command(wheel, request_json):
    override = os.environ.get("TINYG_MSO_WORKER_CMD")      # offline tests replace the worker
    if override:
        return [*override.split(), request_json]
    return ["uv", "run", "--no-project", "--python", "3.12", "--with", str(pathlib.Path(wheel).expanduser()),
            "--with", "numpy", "python", "-I", str(WORKER), request_json]   # -I: the worker's folder holds tinyg_hil/saleae.py,
                                                                         # which would shadow Saleae's own package


class _Server:
    """The persistent worker (mso_worker.py --serve): one MSO session for the whole test process, so only the first
    capture pays the device setup (~8 s); later ones start in tens of milliseconds. Off with TINYG_MSO_PERSIST=0."""
    proc = None
    next_id = 0

    @classmethod
    def get(cls, wheel, cfg=None):
        if cls.proc is None or cls.proc.poll() is not None:
            import atexit
            cmd = worker_command(wheel, "--serve")
            cls.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, bufsize=1)
            atexit.register(cls.stop)
            end = time.monotonic() + 120
            while time.monotonic() < end:
                line = cls.proc.stdout.readline()
                if not line:
                    raise MsoError("MSO server exited before READY")
                if line.startswith("READY"):
                    break
            cls._warm_up(cfg or {})
        return cls.proc

    @classmethod
    def _warm_up(cls, cfg):
        """A 50 ms capture right after start: it pays the cold setup (FPGA, Smart Cables), and measured on bench 3
        (2026-11-05) a cold capture's data started over 0.26 s after STARTED, a warm one's about 27 ms after."""
        keys = list(cfg.get("digital") or {})
        if not keys:
            return
        out = pathlib.Path(tempfile.mkdtemp(prefix="mso-warm-")) / "warm.json"
        req = {"id": "warmup", "serial": cfg.get("serial"), "seconds": 0.05, "rate": 1_000_000, "volts": 1.6,
               "digital": [[int(k[1]), int(k[3])] for k in keys], "analog": [], "pwm_window": 0.05, "pwm_threshold": 1.5,
               "out": str(out)}
        cls.proc.stdin.write(json.dumps(req) + "\n")
        cls.proc.stdin.flush()
        end = time.monotonic() + 120
        while time.monotonic() < end:
            line = cls.proc.stdout.readline()
            if not line:
                raise MsoError("MSO server exited during its warm-up capture")
            words = line.split()
            if len(words) >= 2 and words[1] == "warmup" and words[0] in ("DONE", "ERROR"):
                if words[0] == "ERROR":
                    raise MsoError(f"MSO warm-up failed: {line.strip()}")
                return

    @classmethod
    def stop(cls):
        if cls.proc is not None and cls.proc.poll() is None:
            try:
                cls.proc.stdin.close()
                cls.proc.wait(timeout=10)
            except Exception:  # noqa: BLE001
                cls.proc.kill()
        cls.proc = None


def _persistent():
    return os.environ.get("TINYG_MSO_PERSIST", "1") != "0" and not os.environ.get("TINYG_MSO_WORKER_CMD")


class Capture:
    """A timed MSO capture: the worker starts on enter, its result is read on finish() or exit."""

    def __init__(self, mso, digital=(), analog=(), seconds=5.0, rate=2_000_000, volts=1.6, save_as=None,
                 arm_timeout=60.0, pwm_window=0.5, pwm_threshold=1.5):
        self.cfg = mso
        self.digital_keys = _lookup(mso.get("digital"), digital)            # "p0c0" ...
        self.analog_keys = _lookup(mso.get("analog"), analog)                # "0" ...
        self.names = {k: (mso.get("digital") or {}).get(k, k) for k in self.digital_keys}
        self.anames = {f"a{k}": (mso.get("analog") or {}).get(k, k) for k in self.analog_keys}
        self.seconds, self.rate, self.volts, self.arm_timeout = seconds, rate, volts, arm_timeout
        self.pwm_window, self.pwm_threshold = pwm_window, pwm_threshold
        self.save_as = pathlib.Path(save_as).resolve() if save_as else None
        self._tmp = None
        self.proc = None
        self.armed_wall = None
        self.started_wall = None
        self.result = None

    def __enter__(self):
        if self.save_as is None:
            self._tmp = tempfile.TemporaryDirectory(prefix="mso-")
            out = pathlib.Path(self._tmp.name) / "capture.json"
        else:
            out = pathlib.Path(str(self.save_as) + "-mso.json")
        self.out = out
        req = {"serial": self.cfg.get("serial"), "seconds": self.seconds, "rate": self.rate, "volts": self.volts,
               "digital": [[int(k[1]), int(k[3])] for k in self.digital_keys],
               "analog": [int(k) for k in self.analog_keys], "pwm_window": self.pwm_window,
               "pwm_threshold": self.pwm_threshold, "out": str(out)}
        self._seen = []
        if _persistent():
            return self._enter_server(req)
        self.proc = subprocess.Popen(worker_command(self.cfg.get("wheel", DEFAULT_WHEEL), json.dumps(req)),
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        end = time.monotonic() + self.arm_timeout
        while time.monotonic() < end:
            line = self.proc.stdout.readline()
            if not line:
                break
            self._seen.append(line)
            if line.startswith("ARMED"):
                self.armed_wall = float(line.split()[1])
            elif line.startswith("STARTED"):
                self.started_wall = float(line.split()[1])
                return self
        self._fail("did not start recording" if self.armed_wall else "did not arm")

    def _enter_server(self, req):
        self.server = _Server.get(self.cfg.get("wheel", DEFAULT_WHEEL), self.cfg)
        _Server.next_id += 1
        self.req_id = str(_Server.next_id)
        req["id"] = self.req_id
        self.server.stdin.write(json.dumps(req) + "\n")
        self.server.stdin.flush()
        end = time.monotonic() + self.arm_timeout
        while time.monotonic() < end:
            line = self.server.stdout.readline()
            if not line:
                _Server.proc = None
                self._fail("server exited")
            words = line.split()
            if len(words) >= 2 and words[1] == self.req_id:
                self._seen.append(line)
                if words[0] == "ARMED":
                    self.armed_wall = float(words[2])
                elif words[0] == "STARTED":
                    self.started_wall = float(words[2])
                    return self
                elif words[0] == "ERROR":
                    raise MsoError(f"MSO capture failed: {line.strip()}")
        self._fail("did not start recording" if self.armed_wall else "did not arm")

    def _fail(self, what):
        rest = ""
        if self.proc is not None:
            try:
                rest, _ = self.proc.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                rest, _ = self.proc.communicate()
        rest = "".join(getattr(self, "_seen", [])) + (rest or "")
        hint = EXCLUSIVE_HINT if "exclusive access" in rest else ""
        raise MsoError(f"MSO worker {what}{': ' + hint if hint else ''}\n{rest[-1500:]}")

    def finish(self):
        if self.result is None and getattr(self, "server", None) is not None:
            end = time.monotonic() + self.seconds + 120
            while time.monotonic() < end:
                line = self.server.stdout.readline()
                if not line:
                    _Server.proc = None
                    raise MsoError("MSO server exited during a capture")
                words = line.split()
                if len(words) >= 2 and words[1] == self.req_id:
                    if words[0] == "DONE":
                        self.result = json.loads(self.out.read_text())
                        return self.result
                    if words[0] == "ERROR":
                        raise MsoError(f"MSO capture failed: {line.strip()}")
            raise MsoError("MSO capture timed out")
        if self.result is None:
            try:
                rest, _ = self.proc.communicate(timeout=self.seconds + 120)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self._fail("timed out")
            if self.proc.returncode != 0 or not self.out.exists():
                self.proc.returncode = self.proc.returncode or 1
                hint = EXCLUSIVE_HINT if "exclusive access" in (rest or "") else ""
                raise MsoError(f"MSO worker failed (rc {self.proc.returncode}){': ' + hint if hint else ''}\n"
                               f"{(rest or '')[-1500:]}")
            self.result = json.loads(self.out.read_text())
        return self.result

    def __exit__(self, *exc):
        if exc[0] is None:
            self.finish()
        elif self.proc is not None and self.proc.poll() is None:
            self.proc.kill()
            self.proc.communicate()
        if self._tmp is not None and exc[0] is not None:
            self._tmp.cleanup()

    def edges(self):
        """{bench name: [(t, level), ...]} for the digital channels (seconds from the capture start)."""
        r = self.finish()
        return {self.names.get(k, k): [tuple(x) for x in v] for k, v in r["transitions"].items()}

    def initial(self):
        r = self.finish()
        return {self.names.get(k, k): v for k, v in r["initial"].items()}

    def analog_edges(self, name):
        """An analog channel read as a logic line at the capture's threshold: (initial level, [(t, level), ...])."""
        key = next(k for k, v in self.anames.items() if v == name or k == name or k == f"a{name}")
        e = self.finish()["analog"][key]["edges"]
        return e["initial"], [tuple(x) for x in e["transitions"]]

    def analog_path(self, name):
        """Where the worker saved the analog trace (.npy, float32 volts at the capture rate)."""
        return self.finish()["analog"][self._akey(name)]["npy"]

    def pwm(self, name):
        """[{"t", "duty", "freq_hz"}, ...] per window, computed by the worker (the harness needs no numpy)."""
        return self.finish()["analog"][self._akey(name)]["pwm"]

    def _akey(self, name):
        return next(k for k, v in self.anames.items() if v == name or k == name)


def pwm_windows(y, rate, window=0.5, threshold=1.5):
    """Frequency and duty of a logic-level signal per window, at a fixed threshold (volts). A fixed threshold keeps a
    steady line steady: a threshold taken from the trace's own extremes turns a quiet line's noise into pulses."""
    import numpy as np
    d = (y > threshold).astype(np.int8)
    n = int(window * rate)
    rows = []
    for i in range(0, len(d) - n + 1, n):
        w = d[i:i + n]
        rises = np.flatnonzero(np.diff(w) == 1)
        freq = (len(rises) - 1) / ((rises[-1] - rises[0]) / rate) if len(rises) > 2 else 0.0
        rows.append({"t": round(i / rate, 3), "duty": round(float(w.mean()), 4), "freq_hz": round(float(freq), 2)})
    return rows
