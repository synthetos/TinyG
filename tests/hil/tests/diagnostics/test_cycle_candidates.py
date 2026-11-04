"""Diagnostics for two source-review candidates in the cycles (N14, N15). Needs --ice and a hooks build.

N14 jog clears an alarm: a jog is running when the X min limit switch closes (alarm); when the jog cycle finishes,
    does the machine stay in ALARM (expected) or does the jog's finaliser turn it into PROGRAM_STOP?
N15 homing trusts a stuck switch: during G28.2 X0 the switch closes at the search and never releases; does homing
    still end with the axis marked homed and zeroed (the claim) or fail?
"""
import time

import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice, pytest.mark.motion, pytest.mark.resets_board,
              pytest.mark.timeout(600)]


def _wait(tg, pred, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            if pred():
                return True
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.1)
    return False


@pytest.mark.profile("switches-no", "homing-x")
def test_n14_jog_clears_alarm(tg, switches, ice, record, keep):
    keep("xsn")
    tg.set("xsn", 3, allow="X min as limit + homing so a trip alarms")   # a homing-only switch does not alarm
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    info = {}
    try:
        tg.send_line('{"jogx":30}')                           # a bounded jog
        time.sleep(0.6)
        info["during_jog"] = ice.peek_many(["cm.machine_state", "cm.cycle_state", "cm.motion_state"])
        switches.close("x", "min")                            # a limit trip while jogging: alarm
        time.sleep(0.5)
        info["after_trip"] = ice.peek_many(["cm.machine_state", "cm.cycle_state", "cm.hold_state"])
        _wait(tg, lambda: ice.peek("cm.motion_state") == 0, 20)
        time.sleep(1.0)
        info["after_jog_end"] = ice.peek_many(["cm.machine_state", "cm.cycle_state", "cm.hold_state"])
        info["stat"] = tg.stat()
    finally:
        switches.open("x", "min")
        record("n14", info)
        tg.reset()
        tg.send_gcode("G21 G90 G54", allow="recover")
    assert info["after_trip"]["cm.machine_state"] == 2, f"the trip did not alarm: {info}"
    assert info["after_jog_end"]["cm.machine_state"] == 2 and info["stat"] == 2, \
        f"the alarm was cleared by the jog finishing: {info}"


@pytest.mark.profile("switches-no", "homing-x")
def test_n15_homing_stuck_switch(tg, switches, ice, record):
    tg.send_gcode("G21 G90 G17 G94 G92.1 G54", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0", check=True, allow="declare the position")
    info = {}
    try:
        tg.send_line("G28.2 X0")
        time.sleep(0.8)                                       # into the search move
        info["search"] = ice.peek_many(["cm.homing_state", "cm.cycle_state", "cm.motion_state"])
        switches.close("x", "min")                            # contact... and the switch never releases
        done = _wait(tg, lambda: ice.peek("cm.cycle_state") == 0 or ice.peek("cm.machine_state") == 2, 60)
        time.sleep(0.5)
        info["end"] = {"finished": done, **ice.peek_many(["cm.homed[0]", "cm.homing_state", "cm.machine_state",
                                                          "cm.cycle_state", "mr.position[0]"])}
        info["stat"] = tg.stat()
        try:
            info["posx"] = tg.get("posx")
        except Exception:  # noqa: BLE001
            pass
    finally:
        switches.open("x", "min")
        record("n15", info)
        tg.reset()
        tg.send_gcode("G21 G90 G54", allow="recover")
    assert not info["end"].get("cm.homed[0]"), f"homed with the switch never released: {info}"
