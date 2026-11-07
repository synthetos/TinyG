"""N81 (#296): the spindle speed range settings must read back as stored.

config_app.c gives $p1csl, $p1csh, $p1wsl and $p1wsh precision 0, and pwm.c prints them with "%15.0f", so a
fractional range (a laser set up for S 0-1, as LightBurn's GRBL profile writes) reads back rounded in JSON and in
text mode. The board uses the exact value; a tool that backs up settings by reading them restores the wrong one.
$p1cpl/$p1cph use precision 3. The spindle speed readout sps has precision 0 as well, so S0.5 reads back as 0 or 1.

Each range setting is set to a fraction and read back through JSON and text mode; then M3 S0.5 is read back as sps.
"""
import time

import pytest

pytestmark = [pytest.mark.persists, pytest.mark.dialect("d449"), pytest.mark.bug("N81", on=["d449"]),
              pytest.mark.timeout(6000)]   # includes waiting for the board lock

VALUES = {"p1csl": 0.25, "p1csh": 0.8, "p1wsl": 0.125, "p1wsh": 0.75}


def test_speed_range_reads_back_as_stored(tg, keep, record):
    keep(*VALUES)
    got = {}
    for token, value in VALUES.items():
        tg.set(token, value, allow="N81 fractional speed range")
        got[token] = {"json": tg.get(token), "text": tg.get_text(token)}
    record("n81", got)
    print("N81", got)
    bad = {t: g for t, g in got.items() if any(v is None or abs(v - VALUES[t]) > 1e-4 for v in g.values())}
    assert not bad, f"speed range read back rounded: {bad}"


@pytest.mark.motion
def test_spindle_speed_readout_keeps_fractions(tg, keep, record):
    keep("p1csl", "p1csh")
    tg.set("p1csl", 0, allow="N81 laser-style speed range")
    tg.set("p1csh", 1, allow="N81 laser-style speed range")
    try:
        tg.send_gcode("M3 S0.5", check=True, allow="N81 spindle on an unwired output")
        time.sleep(0.2)
        sps = tg.get("sps")
    finally:
        tg.send_gcode("M5", allow="spindle off")
    record("sps", sps)
    print("N81 sps", sps)
    assert sps is not None and abs(sps - 0.5) < 1e-4, f"S0.5 read back as {sps}"
