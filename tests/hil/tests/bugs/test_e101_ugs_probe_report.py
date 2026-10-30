"""E101: UGS mostly works on edge, but the G38.2 completion report is an unsolicited {"r":{"prb":...}}.

_probing_finish runs json_parser("{\\"prb\\":null}") (cycle_probing.c:208-209), which prints a full
r/f response; UGS's single-step flow (TinyGGcodeCommand.java:38-50) and SPJS's byte count take it as
the reply to whatever command is in flight. g2core reports the probe as a top-level object. Master
does the same (cycle_probing.c:250).
"""
import time

import pytest

from tinyg_hil import snapshot

# TinyGController.java:290-318, 408 (the ^X/SYSTEM READY step is covered by the harness's own reset)
UGS_CONNECT = ["{ej:1}",
               "{sr:{posx:t, posy:t, posz:t, mpox:t, mpoy:t, mpoz:t, plan:t, vel:t, unit:t, stat:t, dist:t, "
               "frmo:t, coor:t}}",
               "{jv:4}", "{qv:0}", "{sv:1}", "{sr:n}", "{si:200}"]


@pytest.mark.persists
def test_ugs_connect_sequence(tg, keep):
    keep("ej", "jv", "qv", "sv", "si")
    original_sr = list(tg.get("sr"))
    assert tg.identity["fv"] <= 0.97        # TinyGController.java:239 skips init above 0.97
    try:
        replies = [tg.send_json(line) for line in UGS_CONNECT]
    finally:
        if list(tg.get("sr")) != original_sr:
            snapshot.set_status_report(tg, original_sr)
    for line, resp in zip(UGS_CONNECT, replies):
        assert resp.raw.startswith('{"r"') and resp.status == 0, (line, resp.raw)
    assert "mpox" in replies[UGS_CONNECT.index("{sr:n}")].body["sr"]


# st=0 (normally open) leaves an unwired probe input inactive, so G38.2 really moves 1 mm and fails
# to trigger; that is not an error and still produces the probe report.
@pytest.mark.motion
@pytest.mark.persists
@pytest.mark.bug("E101", on=["d440", "d449"])
def test_probe_report_is_not_an_r_response(tg, keep):
    keep("st", "zsn")
    tg.set("st", 0)
    tg.set("zsn", 4)                        # Z-min input in probe mode
    z0 = tg.get("posz")
    try:
        resp = tg.send_gcode(f"G38.2 Z{z0 - 1:.3f} F100", allow="1 mm probe move")
        seen = [resp.event] + resp.events
        report = next((e for e in seen if '"prb"' in e.raw), None)
        if report is None:
            report = tg.wait_for(lambda e: '"prb"' in e.raw, 10.0)
    finally:
        tg.wait_idle(timeout=15)
        tg.send_gcode(f"G90 G1 Z{z0:.3f} F300", check=True, allow="return to start")
        tg.wait_idle()
    assert not report.raw.startswith('{"r"'), report.raw
    assert "prb" in report.data
