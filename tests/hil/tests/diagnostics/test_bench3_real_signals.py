"""Bench 3 (switchbox + MSO): behaviour that needs real electrical edges and real step pulses.

  test_probe_real_edge      G38.2 toward Y min (probe input, $ysn=4) closed by the switchbox mid-move. The probe
                            report's stop position must match the real steps of both Y motors (Probing wiki: the
                            report gives where the machine stopped), with e=1.
  test_bouncing_limit       a chattering A min limit (20 edges, 200 us apart) mid-move: one stop, ALARM, the two Y
                            motors in step, the report matching the real steps (Homing and Limits wiki, E050).
  test_step_signal_timing   step pulse width, direction set-up and hold around reversals, and enable lead before the
                            first step, on M2. Driver datasheets need about 1 us high/low and 200 ns set-up; the
                            firmware is meant to give a fixed pulse and change direction only between pulses.
"""
import time

import pytest

from tinyg_hil import benches, mso
from tinyg_hil.switchbox import SwitchBox

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.timeout(300)]


def _cfg(tg):
    bench = benches.bench(tg.transport.port) or {}
    cfg, sbcfg = bench.get("mso") or {}, bench.get("switchbox") or {}
    if not cfg.get("digital") or not sbcfg.get("inputs"):
        pytest.skip("needs the MSO and the switchbox (bench 3)")
    return cfg, sbcfg


def _spm(tg, m):
    return 360.0 * tg.get(f"{m}mi") / (tg.get(f"{m}sa") * tg.get(f"{m}tr"))


def _rises(edges):
    return [t for t, v in edges if v == 1]


def _signed_steps(step_edges, dir_edges, dir_initial):
    """Net steps with the direction level at each rising step edge (dir high = +)."""
    level, di, net = dir_initial, 0, 0
    for t in _rises(step_edges):
        while di < len(dir_edges) and dir_edges[di][0] <= t:
            level = dir_edges[di][1]
            di += 1
        net += 1 if level else -1
    return net


def _box(sbcfg):
    return SwitchBox(sbcfg["port"], lines=sbcfg.get("lines", "asserted"), allow_reset=True).connect()


def test_probe_real_edge(tg, keep, record, request):
    cfg, sbcfg = _cfg(tg)
    spm = _spm(tg, 2)
    keep("ysn", "asn")
    tg.set("asn", 0, allow="no other limit or probe input")
    tg.set("ysn", 4, allow="Y min as the probe input")
    tg.send_gcode("G21 G90", allow="units")
    tg.send_gcode("G28.3 Y20", allow="declare Y at 20")
    line = "G91 G38.2 Y-15 F300"
    tg.guard.check_gcode(line, allow="probe move")
    sb = _box(sbcfg)
    stem = request.config._tinyg_recorder.dir / "probe-real-edge"
    mark = len(tg.transport.history)
    try:
        sb.release_all()
        with mso.Capture(cfg, digital=["M2 step", "M2 dir", "M3 step"], seconds=10, rate=1_000_000,
                         save_as=stem) as cap:
            time.sleep(3.5)
            tg.send_line(line)
            time.sleep(1.5)                                    # 7.5 mm at 5 mm/s
            sb.set_channel(sbcfg["inputs"]["ymin"], action="trip", ttl_ms=1500)
            tg.wait_idle(timeout=10)
        time.sleep(1.6)
    finally:
        sb.release_all()
        sb.disconnect()
    tg.drain()
    prb = [e.data["prb"] for e in tg.transport.history[mark:] if isinstance(e.data, dict) and "prb" in e.data]
    stat = tg.stat()
    y_after = tg.get("posy")
    e = cap.edges()
    net = _signed_steps(e["M2 step"], e["M2 dir"], cap.initial()["M2 dir"])
    summary = {"prb": prb, "stat": stat, "posy_after": y_after, "m2_steps": len(_rises(e["M2 step"])),
               "m3_steps": len(_rises(e["M3 step"])), "m2_net_mm": round(net / spm, 4), "steps_per_mm": spm}
    record("probe_real_edge", summary)
    print("PROBE", summary)
    if stat == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    tg.send_gcode("G90", allow="absolute")
    assert prb and prb[-1].get("e") == 1, f"the probe did not report a trip: {summary}"
    assert summary["m2_steps"] == summary["m3_steps"], f"the two Y motors stepped differently: {summary}"
    # the probe only moves toward -Y, so the stop is 20 minus the steps taken; the direction pin's level for +Y depends
    # on the motor's wiring and $2po (low on bench 3's M2), so only its constancy is checked, not its meaning
    assert abs(net) == summary["m2_steps"], f"M2 changed direction during a one-way probe move: {summary}"
    stop_from_steps = 20 - abs(net) / spm
    assert abs(prb[-1]["y"] - stop_from_steps) <= 2.0 / spm + 0.0005, \
        f"probe report {prb[-1]['y']} vs real stop {stop_from_steps:.4f}: {summary}"


def test_bouncing_limit(tg, keep, record, request):
    cfg, sbcfg = _cfg(tg)
    amin_ch = next(k for k, v in (cfg.get("analog") or {}).items() if "min" in v.lower())
    spm = _spm(tg, 2)
    keep("asn")
    tg.set("asn", 2, allow="A min as a limit input")
    tg.send_gcode("G21 G90", allow="units")
    tg.send_gcode("G28.3 Y0", allow="declare Y")
    line = "G91 G1 Y30 F1200"
    tg.guard.check_gcode(line, allow="bouncing limit move")
    y0 = tg.get("mpoy")
    sb = _box(sbcfg)
    stem = request.config._tinyg_recorder.dir / "limit-bounce"
    mark = len(tg.transport.history)
    try:
        sb.release_all()
        with mso.Capture(cfg, digital=["M2 step", "M3 step"], analog=[amin_ch], seconds=10, rate=1_000_000,
                         save_as=stem) as cap:
            time.sleep(3.5)
            tg.send_line(line)
            time.sleep(1.2)                                   # at cruise
            sb.bounce_channel(sbcfg["inputs"]["amin"], edges=20, interval_us=200, final_state="open")
            time.sleep(1.5)
    finally:
        sb.release_all()
        sb.disconnect()
    tg.drain()
    stat = tg.stat()
    y1 = tg.get("mpoy")
    alarms = [e.raw for e in tg.transport.history[mark:] if e.kind == "exception"]
    e = cap.edges()
    m2, m3 = _rises(e["M2 step"]), _rises(e["M3 step"])
    _, line_edges = cap.analog_edges(amin_ch)
    summary = {"steps_m2": len(m2), "steps_m3": len(m3), "reported_mm": round(y1 - y0, 4),
               "step_mm": round(len(m2) / spm, 4), "stat": stat, "exceptions": alarms,
               "line_edges_seen": len(line_edges)}
    record("limit_bounce", summary)
    print("BOUNCE", summary)
    if stat == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    tg.send_gcode("G90 G0 Y0", allow="return Y")
    tg.wait_idle(timeout=30)
    assert stat == tg.dialect.stat("ALARM"), f"a chattering limit must still alarm: {summary}"
    assert len(m2) == len(m3), f"the two Y motors stepped differently: {summary}"
    assert abs(summary["reported_mm"] - summary["step_mm"]) <= 1.5 / spm, f"position not kept: {summary}"


def test_step_signal_timing(tg, keep, record, request):
    cfg, _ = _cfg(tg)
    tg.send_gcode("G21 G90 G64", allow="units")
    tg.send_gcode("G28.3 Y0", allow="declare Y")
    lines = ["G91 G1 Y2 F1500", "G1 Y-2", "G1 Y2", "G1 Y-2", "G90"]
    for ln in lines:
        tg.guard.check_gcode(ln, allow="reversals for step timing")
    tg.wait_idle()
    time.sleep(tg.get("mt") + 0.5)                            # motors power down, so the enable lead is visible
    stem = request.config._tinyg_recorder.dir / "step-timing"
    with mso.Capture(cfg, digital=["M2 step", "M2 dir", "M2 enable"], seconds=12, rate=1_000_000,
                     save_as=stem) as cap:
        time.sleep(3.5)
        tg.transport.write("".join(ln + "\n" for ln in lines).encode())
        tg.wait_idle(timeout=20)
    e = cap.edges()
    st, dr, en = e["M2 step"], e["M2 dir"], e["M2 enable"]
    highs = [b[0] - a[0] for a, b in zip(st, st[1:]) if a[1] == 1 and b[1] == 0]
    lows = [b[0] - a[0] for a, b in zip(st, st[1:]) if a[1] == 0 and b[1] == 1]
    rises, falls = _rises(st), [t for t, v in st if v == 0]
    setup, hold = [], []
    for t, _v in dr:
        nxt = next((r for r in rises if r > t), None)
        prev_fall = max((f for f in falls if f <= t), default=None)
        if nxt is not None:
            setup.append(nxt - t)
        if prev_fall is not None:
            hold.append(t - prev_fall)
    en_active = [t for t, v in en if v == 0]                  # enable is active low on this bench
    lead = (rises[0] - en_active[0]) if (rises and en_active and en_active[0] < rises[0]) else None
    us = lambda xs: [round(x * 1e6, 3) for x in xs]          # noqa: E731
    summary = {"pulses": len(rises), "dir_changes": len(dr), "high_us_min_med_max": us([min(highs), sorted(highs)[len(highs) // 2], max(highs)]) if highs else None,
               "low_us_min": us([min(lows)])[0] if lows else None,
               "dir_setup_us_min": us([min(setup)])[0] if setup else None,
               "dir_hold_us_min": us([min(hold)])[0] if hold else None,
               "enable_lead_ms": round(lead * 1e3, 3) if lead is not None else None,
               "enable_initial": cap.initial()["M2 enable"]}
    record("step_timing", summary)
    print("TIMING", summary)
    assert highs and min(highs) >= 1e-6, f"step pulse shorter than 1 us: {summary}"
    assert lows and min(lows) >= 1e-6, f"step low time shorter than 1 us: {summary}"
    assert setup and min(setup) >= 2e-7, f"direction changed less than 200 ns before a step: {summary}"
    assert hold and min(hold) >= 2e-7, f"direction changed less than 200 ns after a step: {summary}"
