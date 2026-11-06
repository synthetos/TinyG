"""N79: changing a motor's polarity ($Npo) between moves must not change how many steps it takes.

stepper.c st_prep_line() stores each motor's direction with the polarity folded in (DIRECTION_CW/CCW ^ polarity), and
the segment loader mirrors the substep accumulator whenever that value differs from the previous segment's
(substep_accumulator = -(dda_ticks_X_substeps + substep_accumulator)), which is right for a reversal of travel. After a
$Npo change the next move in the same travel direction looks like a reversal, the accumulator is mirrored, the motor's
fractional step phase moves and it ends the move up to a step off (wiki Configuring-Motors: $Npo only reverses the
motor's rotation). Found on bench 3's MSO (2026-11-06): two Y motors with the same settings, M3 one step ahead of M2
after a job that flipped $3po and back.

Motors 2 and 3 both drive Y with motor 2's geometry; from a reset, four moves of 400 + frac steps run with $3po set to
1 after the first and back to 0 after the second. The firmware's own pulse count (en.en[m].encoder_steps, incremented
for every pulse in the DDA ISR) must be the same for both motors.
"""
import pytest

pytestmark = [pytest.mark.motion, pytest.mark.ice, pytest.mark.persists, pytest.mark.resets_board,
              pytest.mark.dialect("d449"), pytest.mark.bug("N79", on=["d449"])]


@pytest.mark.parametrize("frac", [0.25, 0.4, 0.7])
def test_n79_polarity_change_keeps_step_count(tg, keep, record, ice, frac):
    keep("2ma", "3ma", "2po", "3po", "3sa", "3mi", "3tr", "3pm")
    for tok, val in (("2ma", 1), ("3ma", 1), ("2po", 0), ("3po", 0), ("3sa", tg.get("2sa")), ("3mi", tg.get("2mi")),
                     ("3tr", tg.get("2tr")), ("3pm", tg.get("2pm"))):
        tg.set(tok, val, allow="two motors on Y with the same geometry")
    tg.reset()                                             # clean substep accumulators
    tg.send_gcode("G21 G91", check=True, allow="relative")
    spm = 360.0 * tg.get("2mi") / (tg.get("2sa") * tg.get("2tr"))
    line = f"G1 Y{(400 + frac) / spm:.6f} F300"
    before = ice.peek_many(["en.en[1].encoder_steps", "en.en[2].encoder_steps"])
    for po in (None, 1, 0, None):
        if po is not None:
            tg.set("3po", po, allow="flip motor 3's polarity between moves")
        tg.send_gcode(line, check=True, allow="N79 Y move")
        tg.wait_idle(timeout=30)
    after = ice.peek_many(["en.en[1].encoder_steps", "en.en[2].encoder_steps"])
    tg.send_gcode("G90", check=True)
    m2 = after["en.en[1].encoder_steps"] - before["en.en[1].encoder_steps"]
    m3 = after["en.en[2].encoder_steps"] - before["en.en[2].encoder_steps"]
    r = {"frac": frac, "steps_per_mm": spm, "commanded": 4 * (400 + frac), "m2": m2, "m3": m3}
    record("n79", r)
    assert m2 == m3, f"motor 3 took {m3} steps and motor 2 {m2} after the $3po flips: {r}"
