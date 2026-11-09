"""E060: output pause/resume/abort logic and the spin-up delay.

The serial spin-up measurement is timing-pending: it is a coarse diagnostic,
not physical proof. External scope/Saleae capture must confirm the delay.
ICE tests below check static logical states, not pulse or latency guarantees.
"""
import time

import pytest


def _wait_moving(tg, ref: float, timeout: float = 3.0) -> float:
    """Poll posx until it leaves ref; returns the host time it was first seen moved."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if abs(tg.get("posx") - ref) > 0.002:
            return time.monotonic()
        time.sleep(0.05)
    raise AssertionError(f"posx did not move from {ref} within {timeout}s")


def _wait_still(tg, timeout: float = 3.0) -> float:
    """Wait for the actual stopped hold state; equal position samples alone are insufficient."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if tg.stat() == tg.dialect.stat("HOLD") and tg.get("hold") == tg.dialect.hold("HOLD"):
            return tg.get("posx")
        time.sleep(0.03)
    raise AssertionError("feedhold did not reach its stopped HOLD state")


@pytest.mark.motion
@pytest.mark.timing
@pytest.mark.persists
@pytest.mark.dialect("d449")
@pytest.mark.bug("E060", on=[])
def test_resume_waits_for_spindle_spinup(tg, keep):
    keep("spph", "spde")            # g2core token names: pause-on-hold, spin-up delay (s)
    tg.set("spph", 1)
    tg.set("spde", 1.0)
    tg.send_gcode("G21 G94", check=True)
    start = tg.get("posx")
    tg.send_gcode("M3 S1000", check=True, allow="spindle output for hold/resume")
    tg.send_gcode("G91 G1 X5 F120", check=True, allow="5 mm move at 2 mm/s")
    try:
        _wait_moving(tg, start)
        time.sleep(0.3)
        tg.send_raw(b"!")
        held = _wait_still(tg)
        assert held - start < 4.5, "move finished before the feedhold"
        tg.send_raw(b"~")
        resumed = time.monotonic()
        moved = _wait_moving(tg, held, timeout=4.0)
        assert moved - resumed >= 0.8, f"motion resumed {moved - resumed:.2f}s after ~, before the spin-up delay"
    finally:
        tg.send_raw(b"~")
        tg.wait_idle(timeout=15)
        tg.send_gcode("M5", check=True)
        tg.send_gcode(f"G90 G1 X{start:.3f} F300", check=True, allow="return to start")
        tg.wait_idle()


@pytest.mark.persists
@pytest.mark.dialect("d449")
@pytest.mark.bug("E060", on=[])
def test_hold_settings(tg, keep):
    keep("spph", "spde", "coph")
    for token in ("spph", "coph"):
        for value in (0, 1):
            tg.set(token, value)
            assert tg.get(token) == value
        assert tg.send_json({token: 2}).status == tg.dialect.status("INPUT_VALUE_RANGE_ERROR")
    for value in (0, 0.25, 3600):
        tg.set("spde", value)
        assert tg.get("spde") == pytest.approx(value)
    for value in (-1, 3601):
        assert tg.send_json({"spde": value}).status == tg.dialect.status("INPUT_VALUE_RANGE_ERROR")


def _ice_wait(ice, expr, expected, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if ice.peek(expr) == expected:
            return
        time.sleep(0.02)
    pytest.fail(f"{expr} did not become {expected}")


@pytest.mark.motion
@pytest.mark.ice
@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.parametrize("abort", [False, True])
@pytest.mark.dialect("d449")
@pytest.mark.bug("E060", on=[])
def test_hold_outputs_restore_or_abort(tg, ice, keep, abort):
    """Static output state after HOLD/resume or flush; not a timing verdict."""
    keep("spph", "spde", "coph")
    tg.set("spph", 1)
    tg.set("coph", 1)
    tg.set("spde", 0)
    tg.send_gcode("G21 G91 G94", check=True)
    try:
        tg.send_gcode("M3 S1000 M8", check=True, allow="bare-board spindle/coolant latch test")
        _ice_wait(ice, "cm.gm.spindle_mode", 1)
        tg.send_gcode("G1 X4 F120", check=True, allow="4 mm unloaded test move")
        start = tg.get("posx")
        _wait_moving(tg, start)
        tg.send_raw(b"!")
        _ice_wait(ice, "hold_outputs.state", 1)
        assert ice.peek("cm.gm.spindle_mode") == 0
        assert ice.peek("cm.gm.flood_coolant") == 0
        tg.send_raw(b"%" if abort else b"~")
        _ice_wait(ice, "hold_outputs.state", 0)
        if abort:
            assert ice.peek("cm.gm.spindle_mode") == 0
            assert ice.peek("cm.gm.flood_coolant") == 0
        else:
            _ice_wait(ice, "cm.gm.spindle_mode", 1)
            assert ice.peek("cm.gm.flood_coolant") == 1
    finally:
        # Reset cancels any failed resume before restoring settings.
        tg.reset()


@pytest.mark.motion
@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.dialect("d449")
@pytest.mark.bug("E060", on=[])
@pytest.mark.parametrize("finish", [b"~", b"%", b"\x04"], ids=["resume", "flush", "kill"])
def test_second_hold_cancels_dwell_spinup(tg, keep, finish):
    """Serial state checks only; the external capture measures the physical output timing."""
    keep("spph", "spde", "coph")
    tg.set("spph", 1)
    tg.set("spde", 1)
    tg.set("coph", 1)

    def wait_value(token, value, timeout=2):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if tg.get(token) == value:
                return
            time.sleep(0.02)
        pytest.fail(f"{token} did not become {value}")

    try:
        tg.send_gcode("G21 G94 M3 S1000 M8", check=True, allow="stationary hold output test")
        wait_value("spe", 1)
        dwell_start = time.monotonic()
        tg.send_json({"gc": "G4 P4"}, check=True, allow="bounded stationary dwell")
        time.sleep(0.15)
        tg.send_raw(b"!")
        _wait_still(tg)
        wait_value("spe", 0)
        assert tg.get("cof") == 0
        tg.send_raw(b"~")
        wait_value("spe", 1)
        time.sleep(0.15)
        tg.send_raw(b"!")
        wait_value("spe", 0)
        time.sleep(1.1)  # the canceled deadline must not restart outputs or release the hold
        assert tg.get("spe") == 0
        assert tg.get("hold") == tg.dialect.hold("HOLD")
        tg.send_raw(finish)
        if finish == b"~":
            wait_value("spe", 1)
            wait_value("hold", tg.dialect.hold("OFF"), timeout=3)
            assert tg.get("cof") == 1
            # A lone dwell can report idle while its timer runs: wait it out before restoration.
            time.sleep(max(0, dwell_start + 4.2 - time.monotonic()))
            tg.wait_idle()
        else:
            wait_value("stat", tg.dialect.stat("ALARM") if finish == b"\x04" else tg.dialect.stat("PROGRAM_STOP"))
            assert tg.get("qr") == tg.planner_buffers
            assert tg.get("spe") == 0 and tg.get("cof") == 0
    finally:
        tg.reset()
