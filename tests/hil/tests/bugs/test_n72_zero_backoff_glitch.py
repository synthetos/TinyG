"""N72 (GitHub #37): a switch glitch during the zero backoff must not set the homing zero short.

While CYCLE_HOMING runs, any switch edge requests a feedhold (switch.c _dispatch_switch), and each homing phase ends
when its move stops. A glitch during the zero backoff (a bounce or noise, more than the 50 ms lockout after the latch
release) therefore stopped the backoff early, and _homing_axis_set_zero (cycle_homing.c) set the zero there with no
check that the backoff ran its full $xzb: the machine was homed short and reported success. Homing and Limits wiki:
the axis backs off the switch by the zero backoff and that point becomes zero.

Bench 3, switchbox on Y min, real switch edges. G28.4 Y0 homes without setting coordinates, so the end position is
the real one. A control run (switch closes 1 s into the search, opens 400 ms later during the latch) gives the
reference end; the second run adds a 5 ms closure 800 ms after the release, inside the 5 mm zero backoff. Pass: the
glitch run ends where the control did, or homing reports a failure (status 240, HOMING_CYCLE_FAILED).
"""
import time

import pytest

from tinyg_hil import benches
SwitchBox = pytest.importorskip("tinyg_hil.switchbox", reason="needs the switchbox client").SwitchBox

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449"), pytest.mark.timeout(300),
              pytest.mark.bug("N72", on=["d449"])]

SETTINGS = {"ysn": 1, "ysx": 0, "ytn": 0, "ytm": 30, "ysv": 150, "ylv": 60, "ylb": 2, "yzb": 5}
# A in radius mode: the zero backoff is a linear distance the move converts to degrees (2 mm at ara 10 = 11.46 deg)
SETTINGS_A = {"aam": 3, "ara": 10.0, "asn": 1, "asx": 0, "atn": 0, "atm": 30, "asv": 150, "alv": 60, "alb": 2, "azb": 5}
FAILED = 240


def _home(tg, sb, ch, glitch, axis="y"):
    tg.send_gcode(f"G28.3 {axis.upper()}15", allow="start away from the switch")
    tg.wait_idle()
    mark = len(tg.transport.history)
    sb.release_all()
    tg.send_gcode(f"G28.4 {axis.upper()}0", allow="homing against the switchbox, machine coordinates kept")
    time.sleep(1.0)
    sb.set_channel(ch, action="trip", ttl_ms=400)            # closes in the search, opens during the latch
    if glitch:
        time.sleep(0.4 + 0.8)                                # 800 ms after the release: inside the zero backoff
        sb.set_channel(ch, action="trip", ttl_ms=5)
    end = time.monotonic() + 15
    while time.monotonic() < end:
        time.sleep(0.3)
        if tg.stat() in tg.dialect.idle_stats | tg.dialect.stats("ALARM"):
            break
    tg.drain()
    hist = tg.transport.history[mark:]
    statuses = [e.data.get("f", [0, 0])[1] for e in hist if e.kind == "response" and isinstance(e.data, dict)]
    exc = [e.raw for e in hist if e.kind == "exception"]
    failed = any(f'"st":{FAILED}' in x or f"st:{FAILED}" in x for x in exc) or FAILED in statuses
    mpo = tg.get(f"mpo{axis}")
    time.sleep(1.5)                                          # after a failure nothing may move on by itself
    return {"mpo": mpo, "mpo_later": tg.get(f"mpo{axis}"), "stat": tg.stat(), "failed": failed, "exceptions": exc[:3]}


def _check(tg, keep, record, axis, settings):
    bench = benches.bench(tg.transport.port) or {}
    sbcfg = bench.get("switchbox") or {}
    ch = sbcfg.get("inputs", {}).get(f"{axis}min")
    if not ch:
        pytest.skip(f"needs a switchbox driving {axis.upper()} min (bench 3)")
    keep(*settings)
    for k, v in settings.items():
        tg.set(k, v, allow="homing envelope with a long zero backoff")
    tg.send_gcode("G21 G90", allow="units")
    sb = SwitchBox(sbcfg["port"], lines=sbcfg.get("lines", "asserted"), allow_reset=True).connect()
    try:
        control = _home(tg, sb, ch, glitch=False, axis=axis)
        if control["stat"] == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
        glitched = _home(tg, sb, ch, glitch=True, axis=axis)
        if glitched["stat"] == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
    finally:
        sb.release_all()
        sb.disconnect()
    record(f"n72_{axis}", {"control": control, "glitch": glitched, "settings": settings})
    print("N72", axis, control, glitched)
    assert not control["failed"], f"the control homing failed: {control}"
    short = control["mpo"] - glitched["mpo"]
    assert glitched["failed"] or abs(short) < 0.1, \
        f"homing reported success {short:.3f} short of the control end after a glitch in the zero backoff: {glitched}"
    assert abs(glitched["mpo_later"] - glitched["mpo"]) < 0.001, f"the axis moved on after homing ended: {glitched}"


def test_glitch_in_zero_backoff_does_not_home_short(tg, keep, record):
    _check(tg, keep, record, "y", SETTINGS)


def test_radius_rotary_zero_backoff(tg, keep, record):
    """A radius-mode rotary homes normally (its backoff is checked in degrees), and a glitch is still caught."""
    _check(tg, keep, record, "a", SETTINGS_A)
