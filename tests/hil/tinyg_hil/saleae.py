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
import tempfile
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


def _close(url, capture_id):
    try:
        call("close_capture", url, captureId=capture_id)
    except Exception:  # noqa: BLE001 - already closed (e.g. by hand in Logic 2)
        pass


def _close_kept(url):
    if _KEPT.exists():
        _close(url, int(_KEPT.read_text().strip()))
        _KEPT.unlink()


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
        r = call("start_capture", self.url, deviceId=self.device_id, logicDeviceConfiguration=config,
                 captureConfiguration=({"manualCaptureMode": {}} if self.seconds is None else
                                       {"timedCaptureMode": {"durationSeconds": self.seconds}}))
        self.capture_id = r["captureId"]
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
