"""Bench 3 (MSO): tool length offset (G43.1 / G49) checked on the real step pulses.

Documented behaviour (Gcode-Support wiki "Tool length"; canonical_machine.c cm_tool_offset_set, RS274NGC v3
2.1.2.3): G43.1 sets the offset from axis words and moves nothing; a positive offset moves the controlled point out
along the axis, so with Z at 0, G43.1 Z10 makes Z read -10 and a following G0 Z0 raises the spindle 10 mm. G49 cancels
it. G53 moves are in machine coordinates (no offset); G91 distances are not shifted. Every move here is checked by
counting Z (motor 1) or Y (motors 2 and 3) steps on the MSO, so the offset is proven in the pulses, not only in the
status report. M2 and reset are recorded (the wiki's M2 list does not mention the tool offset).
"""
import time

import pytest

from tinyg_hil import benches, mso

pytestmark = [pytest.mark.diagnostic, pytest.mark.motion, pytest.mark.persists, pytest.mark.resets_board,
              pytest.mark.dialect("d449"), pytest.mark.timeout(600)]


def _cfg(tg):
    cfg = (benches.bench(tg.transport.port) or {}).get("mso") or {}
    if not cfg.get("digital"):
        pytest.skip("needs the MSO (bench 3)")
    return cfg


def _spm(tg, m):
    return 360.0 * tg.get(f"{m}mi") / (tg.get(f"{m}sa") * tg.get(f"{m}tr"))


def _steps(tg, cfg, request, name, lines, channels=("M1 step", "M2 step", "M3 step")):
    """Run lines inside one MSO capture; return the step count per channel and the positions after."""
    with mso.Capture(cfg, digital=list(channels), seconds=8, rate=1_000_000,
                     save_as=request.config._tinyg_recorder.dir / name) as cap:
        time.sleep(0.15)
        for ln in lines:
            tg.guard.check_gcode(ln, allow="tool offset check")
            tg.send_gcode(ln, check=True, allow="tool offset check")
        tg.wait_idle(timeout=30)
    e = cap.edges()
    counts = {k: sum(1 for _, v in e[k] if v == 1) for k in channels}
    return counts, {"posy": tg.get("posy"), "posz": tg.get("posz"), "mpoy": tg.get("mpoy"), "mpoz": tg.get("mpoz")}


def test_tool_offset_on_the_pulses(tg, keep, record, request):
    cfg = _cfg(tg)
    z_spm, y_spm = _spm(tg, 1), _spm(tg, 2)
    keep("g54y", "g54z")
    tg.send_gcode("G21 G90 G54 G49 G92.1", check=True, allow="clean modal state")
    tg.send_gcode("G10 L2 P1 Y0 Z0", check=True, allow="G54 at machine zero for the test")
    tg.send_gcode("G28.3 Y0 Z0", check=True, allow="declare Y and Z")
    tg.wait_idle()
    rows, bad = {}, []

    def check(case, counts, pos, want, z_steps=None, y_steps=None):
        rows[case] = {"steps": counts, "pos": pos, "want": want}
        for k, v in want.items():
            if abs(pos[k] - v) > 0.001:
                bad.append(f"{case}: {k} {pos[k]} != {v}")
        if z_steps is not None and abs(counts["M1 step"] - z_steps) > 1:
            bad.append(f"{case}: Z stepped {counts['M1 step']}, expected {z_steps}")
        if y_steps is not None:
            for k in ("M2 step", "M3 step"):
                if abs(counts[k] - y_steps) > 1:
                    bad.append(f"{case}: {k} stepped {counts[k]}, expected {y_steps}")

    c, p = _steps(tg, cfg, request, "tlo-set", ["G43.1 Z5"])
    check("G43.1 Z5 alone", c, p, {"posz": -5, "mpoz": 0}, z_steps=0, y_steps=0)
    c, p = _steps(tg, cfg, request, "tlo-g0", ["G0 Z0"])
    check("G0 Z0 with the offset", c, p, {"posz": 0, "mpoz": 5}, z_steps=round(5 * z_spm))
    c, p = _steps(tg, cfg, request, "tlo-change", ["G43.1 Z7", "G0 Z0"])
    check("offset 5 -> 7, G0 Z0", c, p, {"posz": 0, "mpoz": 7}, z_steps=round(2 * z_spm))
    c, p = _steps(tg, cfg, request, "tlo-g91", ["G91 G1 Z-2 F600", "G90"])
    check("G91 Z-2 with the offset", c, p, {"posz": -2, "mpoz": 5}, z_steps=round(2 * z_spm))
    c, p = _steps(tg, cfg, request, "tlo-g53", ["G53 G0 Z1"])
    check("G53 Z1 ignores the offset", c, p, {"posz": -6, "mpoz": 1}, z_steps=round(4 * z_spm))
    c, p = _steps(tg, cfg, request, "tlo-g54", ["G10 L2 P1 Z1", "G0 Z0"])
    check("G54 Z1 plus offset 7", c, p, {"posz": 0, "mpoz": 8}, z_steps=round(7 * z_spm))
    c, p = _steps(tg, cfg, request, "tlo-y", ["G43.1 Y3", "G0 Y0"])
    check("Y offset 3 (two motors)", c, p, {"posy": 0, "mpoy": 3}, y_steps=round(3 * y_spm))
    c, p = _steps(tg, cfg, request, "tlo-g49", ["G49", "G0 Y0 Z0"])
    check("G49 then G0 Y0 Z0", c, p, {"posy": 0, "posz": 0, "mpoy": 0, "mpoz": 1},
          z_steps=round(7 * z_spm), y_steps=round(3 * y_spm))
    tg.send_gcode("G10 L2 P1 Z0", check=True, allow="G54 back to machine zero")

    # recorded only: what M2 and a reset do to an active offset
    tg.send_gcode("G43.1 Z4", check=True, allow="offset before M2")
    tg.send_gcode("M2", check=True, allow="program end")
    after_m2 = tg.get("posz")
    tg.reset()
    tg.send_gcode("G21 G90", allow="units")
    after_reset = tg.get("posz")
    rows["M2 and reset"] = {"posz_after_M2": after_m2, "posz_after_reset": after_reset,
                            "mpoz": tg.get("mpoz"), "note": "offset 4 kept if posz = mpoz - 4"}
    tg.send_gcode("G49", allow="cancel")
    tg.send_gcode("G90 G0 Y0 Z0", allow="return")
    tg.wait_idle(timeout=30)
    record("tool_offset", {"z_spm": z_spm, "y_spm": y_spm, "rows": rows, "bad": bad})
    for k, v in rows.items():
        print("TLO", k, v)
    assert not bad, bad
