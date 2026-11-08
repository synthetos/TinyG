"""Bench 3 (MSO): path timing on the real step trains of Y (M2) and Z (M1), both 1280 steps/mm.

  test_dwell_accuracy     G1, G4 P0.5, G1: the pause between the two moves' steps, against 0.5 s.
  test_arc_closure_speed  a full G2 circle in the YZ plane (G19, radius 2 mm, F300): both axes return exactly to
                          their start, and the path speed stays near F away from the ends (no dips at segment joins).
  test_dense_chain_speed  the same circle as 360 G1 segments: closure exact; path speed recorded (planner behaviour on
                          short segments, E015/N39 territory).
"""
import math
import time

import pytest

from tinyg_hil import benches, mso

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.dialect("d449"), pytest.mark.timeout(300)]

F = 300.0


def _cfg(tg):
    cfg = (benches.bench(tg.transport.port) or {}).get("mso") or {}
    if not cfg.get("digital"):
        pytest.skip("needs the MSO (bench 3)")
    return cfg


def _signed(step_edges, dir_edges, init):
    """[(t, +1|-1)] for each rising step edge, with the dir level at that moment."""
    out, level, di = [], init, 0
    for t, v in step_edges:
        if v != 1:
            continue
        while di < len(dir_edges) and dir_edges[di][0] <= t:
            level = dir_edges[di][1]
            di += 1
        out.append((t, 1 if level else -1))
    return out


def _speed(y, z, spm, w=0.02):
    """Path speed (mm/s) in w-second bins from the step counts of two axes."""
    if not y and not z:
        return []
    t0 = min(([y[0][0]] if y else []) + ([z[0][0]] if z else []))
    t1 = max(([y[-1][0]] if y else []) + ([z[-1][0]] if z else []))
    n = int((t1 - t0) / w) + 1
    cy, cz = [0] * n, [0] * n
    for t, _ in y:
        cy[int((t - t0) / w)] += 1
    for t, _ in z:
        cz[int((t - t0) / w)] += 1
    return [math.hypot(a, b) / spm / w for a, b in zip(cy, cz)]


def _circle(tg, request, cfg, name, lines):
    for ln in lines:
        tg.guard.check_gcode(ln, allow=name)
    tg.send_gcode("G21 G90 G64", allow="units")
    tg.send_gcode("G28.3 Y0 Z0", allow="declare Y and Z")
    with mso.Capture(cfg, digital=["M2 step", "M2 dir", "M1 step", "M1 dir"], seconds=14, rate=1_000_000,
                     save_as=request.config._tinyg_recorder.dir / name) as cap:
        time.sleep(0.2)
        tg.transport.write("".join(ln + "\n" for ln in lines).encode()) if len(lines) < 8 else None
        if len(lines) >= 8:
            from tinyg_hil import sender
            sender.stream(tg, lines, "line")
        tg.wait_idle(timeout=30)
    e, init = cap.edges(), cap.initial()
    spm = 360.0 * tg.get("2mi") / (tg.get("2sa") * tg.get("2tr"))
    y = _signed(e["M2 step"], e["M2 dir"], init["M2 dir"])
    z = _signed(e["M1 step"], e["M1 dir"], init["M1 dir"])
    sp = _speed(y, z, spm)
    mid = sp[len(sp) // 8: -len(sp) // 8] if len(sp) > 16 else sp
    s = {"y_steps": len(y), "z_steps": len(z), "y_net": sum(d for _, d in y), "z_net": sum(d for _, d in z),
         "speed_mid_min": round(min(mid), 3) if mid else None, "speed_mid_median": round(sorted(mid)[len(mid) // 2], 3) if mid else None,
         "duration_s": round(max(y[-1][0], z[-1][0]) - min(y[0][0], z[0][0]), 3) if y and z else None,
         "posy": tg.get("posy"), "posz": tg.get("posz"), "spm": spm}
    tg.send_gcode("G90 G17", allow="back to XY plane")
    return s


def test_dwell_accuracy(tg, record, request):
    cfg = _cfg(tg)
    lines = ["G21 G91 G1 Y2 F600", "G4 P0.5", "G1 Y2", "G90"]
    for ln in lines:
        tg.guard.check_gcode(ln, allow="dwell timing")
    tg.send_gcode("G28.3 Y0", allow="declare Y")
    with mso.Capture(cfg, digital=["M2 step"], seconds=6, rate=1_000_000,
                     save_as=request.config._tinyg_recorder.dir / "dwell") as cap:
        time.sleep(0.2)
        tg.transport.write("".join(ln + "\n" for ln in lines).encode())
        tg.wait_idle(timeout=20)
    r = [t for t, v in cap.edges()["M2 step"] if v == 1]
    gap = max(b - a for a, b in zip(r, r[1:]))
    s = {"steps": len(r), "gap_s": round(gap, 4)}
    record("dwell_accuracy", s)
    print("DWELL", s)
    tg.send_gcode("G90 G0 Y0", allow="return Y")
    tg.wait_idle(timeout=30)
    assert 0.495 <= gap <= 0.53, f"the 0.5 s dwell measured {gap:.4f} s between the moves: {s}"


def test_arc_closure_speed(tg, record, request):
    cfg = _cfg(tg)
    s = _circle(tg, request, cfg, "arc-yz", ["G19 G2 Y0 Z0 J2 K0 F300", "G17"])
    record("arc_closure", s)
    print("ARC", s)
    assert s["y_net"] == 0 and s["z_net"] == 0, f"the circle did not close on the steps: {s}"
    assert abs(s["posy"]) < 0.001 and abs(s["posz"]) < 0.001, f"reported position not back at 0: {s}"
    assert s["speed_mid_min"] >= 0.85 * F / 60, f"path speed dipped mid-arc: {s}"


def test_dense_chain_speed(tg, record, request):
    cfg = _cfg(tg)
    n, r = 360, 2.0
    pts = [(r - r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n)) for k in range(1, n + 1)]
    lines = [f"G1 Y{y:.4f} Z{z:.4f} F300" for y, z in pts[:-1]] + ["G1 Y0 Z0"]
    s = _circle(tg, request, cfg, "dense-yz", lines)
    record("dense_chain", s)
    print("DENSE", s)
    assert s["y_net"] == 0 and s["z_net"] == 0, f"the polygon did not close on the steps: {s}"
    assert abs(s["posy"]) < 0.001 and abs(s["posz"]) < 0.001, f"reported position not back at 0: {s}"


def test_repeated_arcs_drift(tg, record, request):
    """Ten full YZ circles with a short dwell between them: the net step count after each circle shows whether a
    one-step closure residual stays bounded (DDA phase carry) or accumulates (position drift)."""
    cfg = _cfg(tg)
    lines = ["G21 G90 G64 G19"] + [ln for _ in range(10) for ln in ("G2 Y0 Z0 J2 K0 F300", "G4 P0.2")] + ["G17"]
    for ln in lines:
        tg.guard.check_gcode(ln, allow="repeated circles")
    tg.send_gcode("G28.3 Y0 Z0", allow="declare Y and Z")
    with mso.Capture(cfg, digital=["M2 step", "M2 dir", "M1 step", "M1 dir"], seconds=40, rate=1_000_000,
                     save_as=request.config._tinyg_recorder.dir / "arcs-x10") as cap:
        time.sleep(0.2)
        from tinyg_hil import sender
        sender.stream(tg, lines, "line")
        tg.wait_idle(timeout=60)
    e, init = cap.edges(), cap.initial()
    y = _signed(e["M2 step"], e["M2 dir"], init["M2 dir"])
    z = _signed(e["M1 step"], e["M1 dir"], init["M1 dir"])
    times = sorted([t for t, _ in y] + [t for t, _ in z])
    pauses = [b for a, b in zip(times, times[1:]) if b - a > 0.15]          # first step after each dwell
    marks = pauses + [times[-1] + 1]
    nets = [(sum(d for t, d in y if t < m), sum(d for t, d in z if t < m)) for m in marks]
    s = {"after_each_circle": nets, "posy": tg.get("posy"), "posz": tg.get("posz")}
    record("repeated_arcs", s)
    print("REPEAT", s)
    tg.send_gcode("G17", allow="XY plane")
    worst = max(max(abs(a), abs(b)) for a, b in nets)
    assert worst <= 1, f"the closure residual grew past one step over ten circles: {s}"
