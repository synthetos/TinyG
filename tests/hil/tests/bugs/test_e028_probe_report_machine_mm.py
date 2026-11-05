"""E028: the probe report gives the machine position in millimetres, whatever G5x offset or G20 is active.

Edge stored cm_get_work_position(ACTIVE_MODEL, axis) in cm.probe_results (cycle_probing.c _probe_finish): work
coordinates, in inches under G20. Master and g2core (cycle_probing.cpp) store cm_get_absolute_position(ACTIVE_MODEL,
axis), the machine position in mm, which hosts written for 440.x (autolevelers, OpenPnP) read; GitHub #135 asks for
G53 machine coordinates in mm. Maintainer 2026-11-05: do what g2core does.

No switch needed: Z min is the probe input with nothing connected, so the probe runs its full travel and the report
gives where it stopped. Machine zero, a G54 Z offset of 5 mm, G20, G38.2 Z-0.3: the report's z must equal mpoz
(about -2.62 mm), not the work position in inches (-0.3). The probe ends in ALARM (it never trips), which the
test clears.
"""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"), pytest.mark.timeout(120),
              pytest.mark.bug("E028", on=[])]


def _body(ev):
    return ev.body if isinstance(ev.body, dict) else {}


def _prb(tg, timeout=15.0):
    return tg.wait_for(lambda e: e.kind in ("probe", "response") and "prb" in _body(e), timeout).body["prb"]


def test_probe_report_is_machine_mm(tg, keep, record):
    for token in ("xsn", "xsx", "ysn", "ysx", "zsx", "asn", "asx"):
        if tg.get(token) == 4:
            keep(token)
            tg.set(token, 0, allow="only one probe input")
    keep("zsn", "g54z")
    tg.set("zsn", 4, allow="Z min is the probe input")
    tg.send_gcode("G21 G90 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="machine zero here")
    tg.send_gcode("G10 L2 P1 Z5", check=True, allow="G54 Z offset 5 mm")
    try:
        tg.send_gcode("G20", check=True)
        line = "G38.2 Z-0.3 F10"
        tg.guard.check_gcode(line, allow="probe 2.6 mm down; nothing trips it")
        tg.transport.write((line + "\n").encode())
        prb = _prb(tg)
        time.sleep(0.5)
        mpoz = tg.get("mpoz")                            # GETs answer in ALARM: a probe that never trips is an
        mpox = tg.get("mpox")                            # error (RS274NGC 3.5.9.1) and leaves the board in ALARM
    finally:
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
        tg.wait_idle(timeout=30)
        tg.send_gcode("G21", check=True)
        tg.send_gcode("G10 L2 P1 Z0", check=True, allow="G54 back")
        tg.send_gcode("G90 G0 Z0", allow="return")
        tg.wait_idle(timeout=30)
    record("e028", {"prb": prb, "mpoz": mpoz, "mpox": mpox})
    print("E028", prb, mpoz)
    assert prb["z"] == pytest.approx(mpoz, abs=0.002), f"probe z {prb['z']} is not the machine position {mpoz} mm"
    assert prb["x"] == pytest.approx(mpox, abs=0.002)
