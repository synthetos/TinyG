"""E051: stepper_reset() on AVR zeroes the DDA downcount without stopping TIMER_DDA.

stepper.c:267-282 stops the timers only under #ifdef __ARM; on AVR it sets
st_run.dda_ticks_downcount = 0 while the DDA ISR keeps running. The ISR's
`if (--st_run.dda_ticks_downcount != 0) return;` (stepper.c:544) wraps the uint32_t to 0xFFFFFFFF,
so _load_move never runs again: the runtime stays busy, the planner never drains and later moves
never execute, while the last segment keeps stepping until $mt de-energizes the motor.
A motor-geometry SET ($1tr/$1sa/$1mi) is a control line processed during motion
(controller.c:157) and calls stepper_reset() via _set_motor_steps_per_unit (stepper.c:1165-1171).

Safety: motors mapped to X are switched from always-on ($Npm=1) to powered-in-cycle for the test,
so a runaway stops stepping electrically after $mt.
"""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.persists, pytest.mark.dialect("d449")]


def power_x_motors_in_cycle(tg, keep):
    for m in range(1, 5):
        if tg.get(f"{m}ma") == 0 and tg.get(f"{m}pm") == 1:
            keep(f"{m}pm")
            tg.set(f"{m}pm", 2)


@pytest.mark.resets_board
def test_motor_setting_during_motion_does_not_hang(tg, keep):
    power_x_motors_in_cycle(tg, keep)
    keep("1tr")
    tr = tg.get("1tr")
    x0 = tg.get("mpox")
    hung = True
    try:
        tg.send_gcode("G91 G1 X5 F100", check=True, allow="5 mm move (3 s)")
        time.sleep(1.0)
        response = tg.send_json({"1tr": tr})       # geometry cannot change during queued motion
        assert response.status == 117               # N60: a persisted setting is refused in a cycle
        tg.wait_idle(timeout=15)                  # buggy: the runtime never goes idle
        hung = False
        x1 = tg.get("mpox")
        tg.send_gcode("G91 G1 X1 F600", check=True, allow="1 mm move after the SET")
        tg.wait_idle(timeout=10)
        assert tg.get("mpox") == pytest.approx(x1 + 1, abs=0.01)
    finally:
        if hung:
            tg.reset()
        else:
            tg.send_gcode(f"G90 G53 G1 X{x0:.3f} F600", check=True, allow="return to start")
            tg.wait_idle(timeout=10)


def test_motor_setting_when_idle(tg, keep):
    keep("1tr")
    tg.set("1tr", tg.get("1tr"))
    x0 = tg.get("mpox")
    tg.send_gcode("G91 G1 X1 F600", check=True, allow="1 mm move")
    try:
        tg.wait_idle(timeout=10)
        assert tg.get("mpox") == pytest.approx(x0 + 1, abs=0.01)
    finally:
        tg.send_gcode(f"G90 G53 G1 X{x0:.3f} F600", check=True, allow="return to start")
        tg.wait_idle(timeout=10)


@pytest.mark.ice
@pytest.mark.resets_board
@pytest.mark.parametrize("command", ["G91 G1 X1 F120", "G4 P5"])
def test_shutdown_stops_stepper_tick_counter(tg, ice, record, command):
    try:
        tg.send_gcode(command, check=True, allow="at most 1 mm motion before shutdown")
        time.sleep(0.2)
        tg.send_json({"shutd": None}, check=True)
        # Shutdown blocks command dispatch; inspect RAM without sending serial queries.
        # MACHINE_SHUTDOWN is 7 (the combined status-report SHUTDOWN value is 12).
        samples = []
        for _ in range(3):
            time.sleep(0.05)
            sample = ice.peek_many(["cm.machine_state", "st_run.dda_ticks_downcount"])
            samples.append(sample)
            record("shutdown_samples", samples)
            assert sample["cm.machine_state"] == 7
            assert sample["st_run.dda_ticks_downcount"] == 0
    finally:
        tg.reset()
