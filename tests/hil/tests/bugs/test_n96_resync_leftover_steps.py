"""N96: a step-counter resync (G28.3 here; flush and homing use the same call) keeps the last segment's steps.

mp_set_steps_to_runtime_position() (planner.c) writes the new step position into en.en[m].encoder_steps but leaves
en.en[m].steps_run, the steps of the last segment that ran, which the stepper folds into encoder_steps only when the
next segment loads (ACCUMULATE_ENCODER, stepper.c). After the resync the firmware's own step count is off by those
steps, and the next move's following error (plan_exec.c) sees them: above STEP_CORRECTION_THRESHOLD (2 steps) the step
correction moves the motor by up to that many steps.

Seen in a merge gate: after test_n39's dense excerpt, test_n51 read encoder_steps 0, steps_run 1 on motor 1 right after
G28.3 (record 20261108T070300Z). An ordinary move ends on a segment with no steps, so the residual is rare and the test
plants it: with the board at rest it sets motor 1's steps_run to 3 through the ICE (enough to cross the correction
threshold), resyncs with G28.3, checks the firmware's step count (encoder_steps + steps_run) against the new origin,
then counts the real step pulses of a 5 mm move with the chip's own timer: they must match the distance.
Needs --ice.
"""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.ice, pytest.mark.dialect("d449"), pytest.mark.timeout(120)]

LEFTOVER = 3            # steps planted in the last segment's count; above STEP_CORRECTION_THRESHOLD (2)
DISTANCE = 5.0          # mm


@pytest.mark.bug("N96", on=["d449"])
def test_resync_drops_the_last_segment_steps(tg, ice, steps, record):
    axis = "xyzabc"[tg.get("1ma")]
    spm = 360 / tg.get("1sa") * tg.get("1mi") / tg.get("1tr")     # steps per mm (per degree on a rotary axis)
    want = round(DISTANCE * spm)
    if want > 60000:
        pytest.skip(f"{want} steps would overflow the 16-bit counter")
    feed = min(800, tg.get(f"{axis}fr"))
    tg.send_gcode("G21 G90 G94 G92.1 G54 G49", check=True)
    tg.send_gcode("G28.3 X0 Y0 Z0 A0", check=True, allow="seed the origin")
    tg.wait_idle()
    ice.poke("en.en[0].steps_run", LEFTOVER)           # the residual a last segment can leave (at rest: no ISR steps)
    tg.send_gcode("G28.3 X0 Y0 Z0 A0", check=True, allow="the resync under test")
    tg.wait_idle()
    after = ice.peek_many(["en.en[0].encoder_steps", "en.en[0].steps_run"])
    steps.arm(1)
    tg.send_gcode(f"G1 {axis.upper()}{DISTANCE:.3f} F{feed}", check=True)
    tg.wait_idle()
    time.sleep(0.1)
    pulses = steps.count()
    steps.disarm()
    end = ice.peek_many(["en.en[0].encoder_steps", "en.en[0].steps_run"])
    record("n96", {"axis": axis, "steps_per_mm": spm, "after_resync": after, "pulses": pulses, "want": want, "end": end})
    tg.send_gcode("G28.3 X0 Y0 Z0 A0", check=True, allow="leave the origin at rest")
    tg.wait_idle()
    count = after["en.en[0].encoder_steps"] + after["en.en[0].steps_run"]
    assert count == 0, f"after G28.3 the step count is {count}, not 0 (the last segment's {LEFTOVER} steps were kept)"
    assert abs(pulses - want) <= 1, f"{pulses} step pulses for {DISTANCE} mm, want {want}: the step correction moved the motor"
