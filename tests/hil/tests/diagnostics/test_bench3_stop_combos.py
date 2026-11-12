"""Bench 3 (switchbox + MSO): stops that combine events, on real edges and real pulses.

  test_limit_during_hold     a feedhold is decelerating when the A min limit closes: the stop escalates to the fast
                             (homing jerk) stop, ALARM follows, both Y motors agree and the report matches the steps.
  test_limit_during_dwell    a limit closes during a lone G4 dwell: ALARM, no step pulses, the board answers.
  test_probe_already_closed  the probe input is already active when G38.2 starts: ALARM 250, no motion (Probing wiki).
  test_limit_at_pulse_cap    Y at the 45 kHz pulse cap when the limit closes: stop distance against the homing-jerk
                             stop, position kept, both motors agree.
"""
import time

import pytest

from tinyg_hil import benches, mso
from tinyg_hil.switchbox import SwitchBox

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(300)]


def _setup(tg):
    bench = benches.bench(tg.transport.port) or {}
    cfg, sbcfg = bench.get("mso") or {}, bench.get("switchbox") or {}
    if not cfg.get("digital") or not sbcfg.get("inputs"):
        pytest.skip("needs the MSO and the switchbox (bench 3)")
    amin = next(k for k, v in (cfg.get("analog") or {}).items() if "min" in v.lower())
    return cfg, sbcfg, amin


def _spm(tg):
    return 360.0 * tg.get("2mi") / (tg.get("2sa") * tg.get("2tr"))


def _rises(edges):
    return [t for t, v in edges if v == 1]


def _box(sbcfg):
    return SwitchBox(sbcfg["port"], lines=sbcfg.get("lines", "asserted"), allow_reset=True).connect()


def _finish(tg):
    if tg.stat() == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    tg.send_gcode("G90 G0 Y0", allow="return Y")
    tg.wait_idle(timeout=60)


def _limit_run(tg, request, name, line, trigger, cfg, sbcfg, amin, hold_first=False):
    spm = _spm(tg)
    tg.send_gcode("G21 G90", allow="units")
    tg.send_gcode("G28.3 Y0", allow="declare Y")
    tg.guard.check_gcode(line, allow=name)
    y0 = tg.get("mpoy")
    sb = _box(sbcfg)
    mark = len(tg.transport.history)
    try:
        sb.release_all()
        with mso.Capture(cfg, digital=["M2 step", "M3 step"], analog=[amin], seconds=10, rate=1_000_000,
                         save_as=request.config._tinyg_recorder.dir / name) as cap:
            time.sleep(3.5)
            tg.send_line(line)
            time.sleep(trigger)
            if hold_first:
                tg.send_raw(b"!")
                time.sleep(0.15)                              # the hold is decelerating
            sb.pulse_channel(sbcfg["inputs"]["amin"], ms=100)
            time.sleep(2.0)
    finally:
        sb.release_all()
        sb.disconnect()
    tg.drain()
    stat, y1 = tg.stat(), tg.get("mpoy")
    exc = [e.raw for e in tg.transport.history[mark:] if e.kind == "exception"]
    e = cap.edges()
    m2, m3 = _rises(e["M2 step"]), _rises(e["M3 step"])
    _, line_edges = cap.analog_edges(amin)
    falls = [t for t, v in line_edges if v == 0]
    t_edge = falls[0] if falls else None
    after = [t for t in m2 if t_edge is not None and t >= t_edge]
    before = [t for t in m2 if t_edge is not None and t < t_edge][-400:]
    v = (len(before) - 1) / (before[-1] - before[0]) / spm if len(before) > 10 else None
    return {"steps_m2": len(m2), "steps_m3": len(m3), "reported_mm": round(y1 - y0, 4),
            "step_mm": round(len(m2) / spm, 4), "stat": stat, "exceptions": exc, "edge_s": t_edge,
            "v_at_edge_mm_s": round(v, 3) if v else None, "stop_after_edge_mm": round(len(after) / spm, 4),
            "edge_to_last_step_ms": round((after[-1] - t_edge) * 1e3, 2) if after else 0.0, "spm": spm}


def _check_kept(tg, s):
    assert s["edge_s"] is not None, f"the limit line never went low on the MSO: {s}"
    assert s["stat"] == tg.dialect.stat("ALARM"), f"a limit hit must alarm: {s}"
    assert s["steps_m2"] == s["steps_m3"], f"the two Y motors stepped differently: {s}"
    assert abs(s["reported_mm"] - s["step_mm"]) <= 1.5 / s["spm"], f"position not kept: {s}"


def test_limit_during_hold(tg, keep, record, request):
    cfg, sbcfg, amin = _setup(tg)
    keep("asn")
    tg.set("asn", 2, allow="A min as a limit")
    s = _limit_run(tg, request, "limit-during-hold", "G91 G1 Y30 F1200", 1.4, cfg, sbcfg, amin, hold_first=True)
    s["yjm"], s["yjh"] = tg.get("yjm"), tg.get("yjh")
    record("limit_during_hold", s)
    print("HOLDLIMIT", s)
    _finish(tg)
    _check_kept(tg, s)


def test_limit_at_pulse_cap(tg, keep, record, request):
    cfg, sbcfg, amin = _setup(tg)
    keep("asn", "yvm", "yfr")
    tg.set("asn", 2, allow="A min as a limit")
    tg.set("yvm", 5000, allow="run at the pulse cap")
    tg.set("yfr", 5000, allow="run at the pulse cap")
    s = _limit_run(tg, request, "limit-at-cap", "G91 G1 Y60 F5000", 1.6, cfg, sbcfg, amin)
    jh = tg.get("yjh") * 1e6 / 216000.0
    v = s["v_at_edge_mm_s"]
    s["homing_jerk_stop_mm"] = round(v * (v / jh) ** 0.5, 4) if v else None
    record("limit_at_cap", s)
    print("CAPLIMIT", s)
    _finish(tg)
    _check_kept(tg, s)


def test_limit_during_dwell(tg, keep, record, request):
    cfg, sbcfg, amin = _setup(tg)
    keep("asn")
    tg.set("asn", 2, allow="A min as a limit")
    sb = _box(sbcfg)
    mark = len(tg.transport.history)
    try:
        sb.release_all()
        with mso.Capture(cfg, digital=["M2 step", "M3 step", "M1 step"], analog=[amin], seconds=8, rate=500_000,
                         save_as=request.config._tinyg_recorder.dir / "limit-during-dwell") as cap:
            time.sleep(3.5)
            tg.send_line("G4 P3")
            time.sleep(1.0)
            sb.pulse_channel(sbcfg["inputs"]["amin"], ms=100)
            time.sleep(0.5)
            t0 = time.monotonic()
            stat = tg.stat()
            answer_s = time.monotonic() - t0
    finally:
        sb.release_all()
        sb.disconnect()
    tg.drain()
    exc = [e.raw for e in tg.transport.history[mark:] if e.kind == "exception"]
    e = cap.edges()
    s = {"stat": stat, "answer_s": round(answer_s, 3), "exceptions": exc,
         "steps": {k: len(_rises(e[k])) for k in ("M1 step", "M2 step", "M3 step")}}
    record("limit_during_dwell", s)
    print("DWELLLIMIT", s)
    if tg.stat() == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    assert stat == tg.dialect.stat("ALARM"), f"a limit hit during a dwell must alarm: {s}"
    assert not any(s["steps"].values()), f"step pulses during a dwell: {s}"
    assert answer_s < 1.0, f"the board answered slowly after the limit: {s}"


def test_probe_already_closed(tg, keep, record, request):
    cfg, sbcfg, _ = _setup(tg)
    keep("ysn", "asn")
    tg.set("asn", 0, allow="no other limit or probe")
    tg.set("ysn", 4, allow="Y min as the probe input")
    tg.send_gcode("G21 G90", allow="units")
    tg.send_gcode("G28.3 Y20", allow="declare Y")
    line = "G91 G38.2 Y-5 F300"
    tg.guard.check_gcode(line, allow="probe with the input already active")
    sb = _box(sbcfg)
    mark = len(tg.transport.history)
    try:
        sb.release_all()
        sb.set_channel(sbcfg["inputs"]["ymin"], action="trip", ttl_ms=6000)
        time.sleep(0.2)
        with mso.Capture(cfg, digital=["M2 step", "M3 step"], seconds=6, rate=500_000,
                         save_as=request.config._tinyg_recorder.dir / "probe-already-closed") as cap:
            time.sleep(3.5)
            tg.send_line(line)
            time.sleep(1.0)
    finally:
        sb.release_all()
        sb.disconnect()
    tg.drain()
    hist = tg.transport.history[mark:]
    prb = [e.data["prb"] for e in hist if isinstance(e.data, dict) and "prb" in e.data]
    exc = [e.raw for e in hist if e.kind == "exception"]
    stat = tg.stat()
    e = cap.edges()
    s = {"stat": stat, "prb": prb, "exceptions": exc, "m2_steps": len(_rises(e["M2 step"])),
         "posy": tg.get("posy")}
    record("probe_already_closed", s)
    print("PROBECLOSED", s)
    if stat == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    tg.send_gcode("G90", allow="absolute")
    assert s["m2_steps"] == 0, f"the machine moved although the probe was already active: {s}"
    assert stat == tg.dialect.stat("ALARM"), f"expected ALARM (probe already tripped): {s}"
    assert any('"st":250' in x for x in exc), f"expected status 250: {s}"
