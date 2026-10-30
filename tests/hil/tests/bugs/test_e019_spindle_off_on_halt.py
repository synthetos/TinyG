"""E019: ALARM, SHUTDOWN and PANIC never stop the spindle or coolant.

edge canonical_machine.c:769-777 cm_alarm leaves `cm_spindle_control_immediate(SPINDLE_OFF)` and
`cm_coolant_off_immediate()` commented out; cm_shutdown/cm_panic (:805-808, :843-846) comment out
spindle_reset()/coolant_reset(), and spindle_reset() is `return;` (spindle.c:53-56). The only off path is
a queued M5/M9, which the alarm's queue flush discards. In SHUTDOWN the indicator LED toggled every
100 ms (controller.c:363) is SPINDLE_DIR (hardware.h:192), so the direction output flips at 5 Hz.

Serial has no spindle-state readback, so the output pins are read with the Atmel-ICE (PORTx.OUT).
"""
import time

import pytest

pytestmark = [pytest.mark.ice, pytest.mark.motion, pytest.mark.resets_board, pytest.mark.dialect("d449")]
OUTPUTS = "spindle and coolant outputs (no motion)"


def _outputs_on(tg, ice):
    tg.send_gcode("M3 S1000", check=True, allow=OUTPUTS)
    tg.send_gcode("M8", check=True, allow=OUTPUTS)
    end = time.monotonic() + 2
    while True:
        out = ice.outputs()
        if out["spindle_enable"] and out["coolant"]:
            return out
        assert time.monotonic() < end, f"M3/M8 did not switch the outputs on: {out}"
        time.sleep(0.05)


@pytest.mark.bug("E019", on=["d449"])
def test_alarm_turns_spindle_and_coolant_off(tg, ice):
    _outputs_on(tg, ice)
    try:
        tg.send_json({"alarm": None})              # a GET of alarm raises it (E138)
        time.sleep(0.3)
        assert tg.stat() == tg.dialect.stat("ALARM")
        out = ice.outputs()
        assert (out["spindle_enable"], out["coolant"]) == (0, 0), f"outputs still on in ALARM: {out}"
    finally:
        tg.clear_alarm()
        tg.send_gcode("M5 M9", allow=OUTPUTS)


@pytest.mark.bug("E019", on=["d449"])
def test_shutdown_turns_outputs_off_and_holds_direction(tg, ice):
    _outputs_on(tg, ice)
    try:
        tg.send_json({"shutd": None})              # a GET of shutd performs it (E138)
        time.sleep(0.3)
        samples = []
        for _ in range(8):                         # ~0.8 s: the 100 ms indicator toggle would show
            samples.append(ice.outputs())
            time.sleep(0.1)
        assert all((s["spindle_enable"], s["coolant"]) == (0, 0) for s in samples), samples
        assert len({s["spindle_dir"] for s in samples}) == 1, f"spindle direction toggling: {samples}"
    finally:
        tg.reset()                                 # SHUTDOWN cannot be cleared over serial (E045)
