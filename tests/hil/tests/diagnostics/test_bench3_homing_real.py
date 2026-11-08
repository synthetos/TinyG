"""Bench 3 (switchbox + MSO): Y homing with real switch edges (Homing and Limits wiki; N15).

  test_homing_real_switch    G28.2 Y0 searches toward Y min; the switchbox closes Y min 1 s in and releases it 400 ms
                             later, during the latch backoff. Homing must finish homed, without an alarm, at Y=0, with
                             both Y motors in step.
  test_homing_stuck_switch   the switch stays closed through latch and backoff: homing must fail (status 240,
                             HOMING_CYCLE_FAILED) and leave Y unhomed (N15).
"""
import time

import pytest

from tinyg_hil import benches, mso
from tinyg_hil.switchbox import SwitchBox

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(300)]

SETTINGS = {"ysn": 1, "ysx": 0, "ytn": 0, "ytm": 30, "ysv": 300, "ylv": 60, "ylb": 2, "yzb": 1}


def _setup(tg, keep):
    bench = benches.bench(tg.transport.port) or {}
    cfg, sbcfg = bench.get("mso") or {}, bench.get("switchbox") or {}
    if not cfg.get("digital") or not sbcfg.get("inputs", {}).get("ymin"):
        pytest.skip("needs the MSO and a switchbox driving Y min (bench 3)")
    keep(*SETTINGS)
    for k, v in SETTINGS.items():
        tg.set(k, v, allow="small Y homing envelope")
    tg.send_gcode("G21 G90", allow="units")
    tg.send_gcode("G28.3 Y15", allow="start Y away from the switch")
    return cfg, sbcfg


def _run(tg, request, name, cfg, sbcfg, ttl_ms):
    sb = SwitchBox(sbcfg["port"], lines=sbcfg.get("lines", "asserted"), allow_reset=True).connect()
    mark = len(tg.transport.history)
    try:
        sb.release_all()
        with mso.Capture(cfg, digital=["M2 step", "M2 dir", "M3 step"], seconds=12, rate=1_000_000,
                         save_as=request.config._tinyg_recorder.dir / name) as cap:
            time.sleep(0.2)
            tg.send_gcode("G28.2 Y0", allow="Y homing against the switchbox")
            time.sleep(1.0)
            sb.set_channel(sbcfg["inputs"]["ymin"], action="trip", ttl_ms=ttl_ms)
            end = time.monotonic() + 10
            while time.monotonic() < end:
                time.sleep(0.3)
                if tg.stat() in tg.dialect.idle_stats | tg.dialect.stats("ALARM"):
                    break
    finally:
        sb.release_all()
        sb.disconnect()
    tg.drain()
    hist = tg.transport.history[mark:]
    exc = [e.raw for e in hist if e.kind == "exception"]
    statuses = [e.data.get("f", [0, 0])[1] for e in hist if e.kind == "response" and isinstance(e.data, dict)]
    e = cap.edges()
    rises = lambda k: [t for t, v in e[k] if v == 1]          # noqa: E731
    return {"stat": tg.stat(), "homed_y": tg.get("homy"), "posy": tg.get("posy"), "exceptions": exc,
            "statuses": statuses, "m2_steps": len(rises("M2 step")), "m3_steps": len(rises("M3 step")),
            "dir_changes": len(e["M2 dir"])}


def test_homing_real_switch(tg, keep, record, request):
    cfg, sbcfg = _setup(tg, keep)
    s = _run(tg, request, "homing-real", cfg, sbcfg, ttl_ms=400)
    record("homing_real", s)
    print("HOMING", s)
    if s["stat"] == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    assert s["m2_steps"] == s["m3_steps"], f"the two Y motors stepped differently: {s}"
    assert s["stat"] in tg.dialect.idle_stats, f"homing did not finish idle: {s}"
    assert s["homed_y"] == 1 and abs(s["posy"]) < 0.001, f"Y not homed at 0: {s}"


def test_homing_stuck_switch(tg, keep, record, request):
    cfg, sbcfg = _setup(tg, keep)
    s = _run(tg, request, "homing-stuck", cfg, sbcfg, ttl_ms=8000)
    record("homing_stuck", s)
    print("STUCK", s)
    if s["stat"] == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    assert s["m2_steps"] == s["m3_steps"], f"the two Y motors stepped differently: {s}"
    assert s["homed_y"] == 0, f"homing accepted a switch that never released: {s}"
    failed = tg.dialect.status("HOMING_CYCLE_FAILED")
    assert any(f'"st":{failed}' in x for x in s["exceptions"]) or failed in s["statuses"], \
        f"the failed homing was not reported: {s}"
