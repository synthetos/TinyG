"""E157: a jog token sent while a program runs silently drops the queued program.

canonical_machine.c cm_run_jogx() (and y/z/a) start the jogging cycle with no state check; the cycle
waits for the current move, then flushes the planner and jogs. Measured on bench 2: with G1 X10 and
G1 X0 queued, {jogx:3} mid-move answered 0 and the machine ended at X=3; G1 X0 never ran. A jog is
now refused (106) unless the machine is idle or stopped in a feedhold; jogging out of a hold, which
discards the held queue by design, still works.
"""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.dialect("d449")]


def _settle(tg, timeout=30):
    end = time.monotonic() + timeout
    while time.monotonic() < end and tg.get("cycs") != 0:
        time.sleep(0.2)


def test_jog_refused_while_program_runs(tg):
    tg.send_gcode("G90 G21 G0 X0", allow="X to 0")
    _settle(tg)
    try:
        tg.send_gcode("G1 X6 F300", check=True, allow="6 mm at 5 mm/s")
        tg.send_gcode("G1 X0", check=True, allow="and back")
        time.sleep(0.6)
        assert tg.send_json({"jogx": 3}).status == tg.dialect.status("COMMAND_NOT_ACCEPTED")
        _settle(tg)
        assert tg.get("mpox") == pytest.approx(0, abs=0.01), "the queued program did not finish"
    finally:
        _settle(tg)


def test_jog_from_idle_and_from_hold_still_works(tg):
    tg.send_gcode("G90 G21 G0 X0", allow="X to 0")
    _settle(tg)
    try:
        assert tg.send_json({"jogx": 2}).status == 0
        _settle(tg)
        assert tg.get("mpox") == pytest.approx(2, abs=0.01)
        tg.send_gcode("G1 X8 F300", check=True, allow="6 mm")
        time.sleep(0.6)
        tg.send_raw(b"!")
        time.sleep(1.0)
        assert tg.get("hold") == tg.dialect.hold("HOLD")
        assert tg.send_json({"jogx": 1}).status == 0              # jogging out of a hold, as before
        _settle(tg)
        assert tg.get("mpox") == pytest.approx(1, abs=0.01)
    finally:
        tg.send_gcode("G90 G0 X0", allow="back")
        _settle(tg)
