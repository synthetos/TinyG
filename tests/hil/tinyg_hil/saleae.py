"""Saleae Logic 2 through its MCP server (JSON-RPC over HTTP), for timing measurements on step/dir pins.

    with Capture(device_id, channels=range(8), seconds=6) as cap:
        ...move the machine...
    edges = cap.edges()          # {channel: [(t_seconds, level), ...]} transitions, from the CSV export

The server URL defaults to http://127.0.0.1:10530/mcp (the bare root answers 400).
"""
import csv
import itertools
import json
import pathlib
import subprocess
import tempfile
import time
import urllib.request

URL = "http://127.0.0.1:10530/mcp"
_ids = itertools.count(1)


def call(tool, url=URL, timeout=600, **arguments):
    body = {"jsonrpc": "2.0", "id": next(_ids), "method": "tools/call",
            "params": {"name": tool, "arguments": arguments}}
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    reply = json.load(urllib.request.urlopen(req, timeout=timeout))
    if "error" in reply:
        raise RuntimeError(f"saleae {tool}: {reply['error']}")
    result = reply["result"]
    if result.get("isError"):
        raise RuntimeError(f"saleae {tool}: {result.get('content')}")
    return result.get("structuredContent") or json.loads(result["content"][0]["text"] or "{}")


_KEPT = pathlib.Path(tempfile.gettempdir()) / "tinyg-hil-saleae-kept-capture"
# The capture in progress. A test killed mid-capture leaves it recording, and Logic 2 then refuses every new
# capture ("Cannot switch sessions while recording") and has no call to list captures; this file is how the
# next capture finds and stops it.
_ACTIVE = pathlib.Path(tempfile.gettempdir()) / "tinyg-hil-saleae-active-capture"
_LOGIC = "Saleae Logic.app/Contents/MacOS/Logic"


def _close(url, capture_id):
    try:
        call("close_capture", url, captureId=capture_id)
    except Exception:  # noqa: BLE001 - already closed (e.g. by hand in Logic 2)
        pass


def _close_kept(url):
    if _KEPT.exists():
        _close(url, int(_KEPT.read_text().strip()))
        _KEPT.unlink()


def _stop_leftover(url):
    if _ACTIVE.exists():
        capture_id = int(_ACTIVE.read_text().strip())
        try:
            call("stop_capture", url, captureId=capture_id)
        except Exception:  # noqa: BLE001 - already stopped or finished
            pass
        _close(url, capture_id)
        _ACTIVE.unlink()


def _restart_logic(url):
    """Last resort for a recording no capture ID reaches: quit and relaunch Logic 2 (macOS), wait for the device."""
    subprocess.run(["osascript", "-e", 'quit app "Saleae Logic"'], capture_output=True, timeout=30)
    for _ in range(30):
        if subprocess.run(["pgrep", "-f", _LOGIC], capture_output=True).returncode:
            break
        time.sleep(1)
    else:
        subprocess.run(["pkill", "-f", _LOGIC])
        time.sleep(3)
    subprocess.run(["open", "-a", "Saleae Logic"], check=True)
    deadline = time.monotonic() + 90
    while time.monotonic() < deadline:
        try:
            if devices(url):
                return
        except Exception:  # noqa: BLE001 - server not up yet
            pass
        time.sleep(1)
    raise RuntimeError("saleae: Logic 2 did not come back after a restart")


def devices(url=URL):
    return call("get_devices", url, includeSimulationDevices=False).get("devices", [])


class Capture:
    """A timed digital capture; start on enter, wait and export on exit (or call finish())."""

    def __init__(self, device_id, channels=range(8), seconds=5.0, rate=12_000_000, volts=None, url=URL,
                 keep_open=False, save_as=None):
        """save_as: a path stem; finish() keeps <stem>.sal and <stem>-edges.json (sample rate, initial levels,
        transitions) next to the run record, so the evidence outlives the capture tab."""
        self.device_id, self.channels, self.seconds = device_id, list(channels), seconds
        self.rate, self.volts, self.url, self.keep_open = rate, volts, url, keep_open
        self.save_as = save_as
        self.capture_id = None
        self._edges = None

    def __enter__(self):
        _close_kept(self.url)               # one harness capture tab at a time in Logic 2
        config = {"logicChannels": {"digitalChannels": self.channels}, "digitalSampleRate": self.rate}
        if self.volts is not None:          # only Logic Pro models have a configurable threshold
            config["digitalThresholdVolts"] = self.volts
        _stop_leftover(self.url)
        args = dict(deviceId=self.device_id, logicDeviceConfiguration=config,
                    captureConfiguration=({"manualCaptureMode": {}} if self.seconds is None else
                                          {"timedCaptureMode": {"durationSeconds": self.seconds}}))
        try:
            r = call("start_capture", self.url, **args)
        except RuntimeError as e:
            if "while recording" not in str(e):
                raise
            _restart_logic(self.url)
            r = call("start_capture", self.url, **args)
        self.capture_id = r["captureId"]
        _ACTIVE.write_text(str(self.capture_id))
        return self

    def finish(self):
        if self._edges is None:
            try:
                if self.seconds is None:    # manual mode: stop now
                    call("stop_capture", self.url, captureId=self.capture_id)
                call("wait_capture", self.url, captureId=self.capture_id)
                with tempfile.TemporaryDirectory(prefix="saleae-") as d:
                    call("export_raw_data_csv", self.url, captureId=self.capture_id, directory=d,
                         logicChannels={"digitalChannels": self.channels}, analogDownsampleRatio=1)
                    self.initial, self._edges = _parse_csv(pathlib.Path(d), self.channels)
                if self.save_as is not None:
                    stem = pathlib.Path(self.save_as).resolve()
                    stem.parent.mkdir(parents=True, exist_ok=True)
                    call("save_capture", self.url, captureId=self.capture_id, filepath=str(stem) + ".sal", timeout=30)
                    pathlib.Path(str(stem) + "-edges.json").write_text(json.dumps(
                        {"sample_rate": self.rate, "channels": self.channels, "initial": self.initial,
                         "transitions": self._edges}) + "\n")
            finally:
                _ACTIVE.unlink(missing_ok=True)
                if self.keep_open:          # left on screen for a person; closed by the next capture
                    _KEPT.write_text(str(self.capture_id))
                else:
                    _close(self.url, self.capture_id)
        return self._edges

    def __exit__(self, *exc):
        if exc[0] is None:
            self.finish()
        elif self.capture_id is not None:
            try:
                call("stop_capture", self.url, captureId=self.capture_id)
            except Exception:  # noqa: BLE001 - cleanup must not mask the original error
                pass
            _close(self.url, self.capture_id)
            _ACTIVE.unlink(missing_ok=True)

    def edges(self):
        return self.finish()


def _parse_csv(directory, channels):
    """digital.csv: 'Time [s]', 'Channel 0', ... one row per change. Returns ({ch: initial level}, {ch: transitions})."""
    path = next(directory.glob("*.csv"))
    out = {c: [] for c in channels}
    with path.open() as f:
        rows = csv.reader(f)
        header = next(rows)
        cols = {int(h.split()[-1]): i for i, h in enumerate(header) if h.strip().lower().startswith("channel")}
        last = {}
        for row in rows:
            t = float(row[0])
            for c in channels:
                v = int(row[cols[c]])
                if last.get(c) != v:
                    if c in last:
                        out[c].append((t, v))
                    last[c] = v
    return {c: last.get(c) if not out[c] else 1 - out[c][0][1] for c in channels}, out


def rising(transitions, start=0.0, end=float("inf")):
    return [t for t, v in transitions if v == 1 and start <= t < end]


def level_at(initial, transitions, t):
    """Logic level of a channel at time t."""
    level = initial
    for when, v in transitions:
        if when > t:
            break
        level = v
    return level
