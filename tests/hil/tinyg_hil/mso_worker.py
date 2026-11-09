"""Capture worker for tinyg_hil.mso: runs inside its own environment with saleae-mso-api and numpy.

Standalone on purpose (no tinyg_hil imports): the harness environment does not carry saleae-mso-api, so
tinyg_hil.mso starts this file with `uv run --with <wheel> --with numpy`. It reads a JSON request on argv[1],
prints "ARMED <wall time>" just before the capture call, then "STARTED <wall time>" once the device log says
recording started (each capture re-initializes the MSO and waits for the Smart Cables, which took 8 s on
2026-11-04), and writes the result JSON to request["out"].
"""
import datetime
import json
import re
import sys
import threading
import time
from pathlib import Path

STARTED_RE = re.compile(r"^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\].*recording started", re.M)


def watch_start(wrapper, done, tag="", since=0.0):
    """Prints STARTED with the device log's own timestamp (local time) as soon as recording starts. Only a log line
    stamped after `since` counts: in a persistent worker the previous capture's line can still be in the buffer."""
    while not done.is_set():
        for m in STARTED_RE.finditer(wrapper.last_stdout or ""):
            t = datetime.datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S.%f").timestamp()
            if t >= since:
                print("STARTED", tag + repr(t), flush=True)
                return
        time.sleep(0.005)


_DEVICES = {}


def _device(serial):
    """One MSO session per serial for the life of the worker: only the first capture pays the FPGA setup (~8 s)."""
    from saleae import mso_api
    if serial not in _DEVICES:
        _DEVICES[serial] = mso_api.MSO(serial_number=serial) if serial else mso_api.MSO()
    return _DEVICES[serial]


def main(req, tag=""):
    import numpy as np
    from saleae import mso_api

    chans = [mso_api.DigitalChannel(port=p, channel=c, name=f"p{p}c{c}", threshold_volts=req.get("volts", 1.6))
             for p, c in req.get("digital", [])]
    chans += [mso_api.AnalogChannel(channel=a, name=f"a{a}") for a in req.get("analog", [])]
    mso = _device(req.get("serial"))
    cfg = mso_api.CaptureConfig(enabled_channels=chans,
                                analog_settings=mso_api.AnalogSettings(sample_rate=req.get("rate", 2e6)),
                                capture_settings=mso_api.TimedCapture(capture_length_seconds=req["seconds"]))
    out = Path(req["out"])
    out.parent.mkdir(parents=True, exist_ok=True)
    done = threading.Event()
    armed = time.time()
    threading.Thread(target=watch_start, args=(mso.wrapper, done, tag, armed - 0.05), daemon=True).start()
    print("ARMED", tag + repr(armed), flush=True)
    try:
        cap = mso.capture(cfg, save_dir=out.parent / (out.stem + "-raw"))
    finally:
        done.set()
    res = {"rate": req.get("rate", 2e6), "initial": {}, "transitions": {}, "analog": {}}
    for name, d in cap.digital_data.items():
        t = [float(x) for x in np.asarray(d.transition_times)]
        level, trans = int(d.initial_state), []
        for x in t:
            level ^= 1
            trans.append((x, level))
        res["initial"][name] = int(d.initial_state)
        res["transitions"][name] = trans
    for name, a in (getattr(cap, "analog_data", None) or {}).items():
        y = np.asarray(a.voltages, dtype=np.float32)
        npy = out.parent / f"{out.stem}-{name}.npy"
        np.save(npy, y)
        res["analog"][name] = {"npy": str(npy), "n": int(len(y)), "min": float(y.min()), "max": float(y.max()),
                               "pwm": pwm_windows(y, req.get("rate", 2e6), req.get("pwm_window", 0.5),
                                                  req.get("pwm_threshold", 1.5)),
                               "edges": threshold_edges(y, req.get("rate", 2e6), req.get("pwm_threshold", 1.5))}
    out.write_text(json.dumps(res))
    if not req.get("keep_raw"):                 # the instrument's own binary save (~70 MB a capture) is parsed above; nothing
        import shutil                            # reads it later, so it is not kept unless asked for
        shutil.rmtree(out.parent / (out.stem + "-raw"), ignore_errors=True)
    print("DONE", tag.strip(), flush=True)


def threshold_edges(y, rate, threshold=1.5, limit=20000):
    """An analog line read as a logic line: (time, level) at each crossing of the threshold, first `limit` of them."""
    import numpy as np
    d = (y > threshold).astype(np.int8)
    idx = np.flatnonzero(np.diff(d)) + 1
    return {"initial": int(d[0]) if len(d) else 0,
            "transitions": [(round(float(i) / rate, 9), int(d[i])) for i in idx[:limit]], "count": int(len(idx))}


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


def serve():
    """Persistent mode: one JSON request per stdin line; replies ARMED/STARTED/DONE/ERROR lines tagged with its id."""
    print("READY", flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        req = json.loads(line)
        tag = f"{req.get('id', 0)} "
        try:
            main(req, tag)
        except Exception as e:  # noqa: BLE001 - reported to the harness, the server keeps running
            print("ERROR", tag + repr(e).replace("\n", " ")[:800], flush=True)


if __name__ == "__main__":
    if sys.argv[1:] == ["--serve"]:
        serve()
    else:
        main(json.loads(sys.argv[1]))
