"""N68: no EEPROM write while queued work runs; G10 offsets are saved once the board is idle.

A persisted write masks the medium and low interrupt levels for each value (AVR1008 workaround, xmega_eeprom.c), so
serial bytes that arrive meanwhile are lost and segment preparation stalls. N60 refuses persisted SETs while work is
pending; write_persistent_value() (persistence.c) still refused only when cm.cycle_state != CYCLE_OFF, and a lone G4
dwell does not start a cycle. So a G10 sent during a dwell was written to EEPROM at once (canonical_machine.c
cm_set_coord_offsets, "cycle off and no aline in flight"), with the dwell running.

G10 sets the offset at once and saves it when machining ends (NIST RS274NGC v3 3.5.5, TinyG wiki Coordinate-Systems:
G10 is persisted after the cycle). The test sends G10 during a dwell and resets the board before the dwell ends:
- written at once (the defect): the new offset survives the reset;
- saved when idle (the fix): the reset comes first, so the saved offset is still the old one.
test_g10_saved_after_dwell checks the other half: without a reset, the offset is saved once the dwell is over.
No ICE needed: the reset is the observation.
"""
import time

import pytest

pytestmark = [pytest.mark.persists, pytest.mark.resets_board, pytest.mark.dialect("d449"), pytest.mark.timeout(120)]

OLD, NEW = 1.25, 7.5


def _dwell_then_g10(tg):
    tg.send_gcode("G21 G90 G54", check=True, allow="units")
    tg.wait_idle()
    tg.set("g55x", OLD, allow="known saved offset before the test")
    assert tg.get("g55x") == pytest.approx(OLD)
    tg.send_gcode("G4 P3", check=True, allow="lone dwell: queued work, no cycle")
    started = time.monotonic()
    time.sleep(0.4)                                          # the dwell is running
    line = f"G10 L2 P2 X{NEW}"
    tg.guard.check_gcode(line, allow="set an offset without motion")
    reply = tg.send_line(line, timeout=5)
    assert reply.status == 0, reply.raw
    assert tg.get("g55x") == pytest.approx(NEW), "G10 must take effect at once"
    return started


@pytest.mark.bug("N68", on=["d449"])
def test_g10_during_dwell_not_written_until_idle(tg, keep, record):
    keep("g55x")
    _dwell_then_g10(tg)
    tg.reset()                                               # well inside the 3 s dwell
    saved = tg.get("g55x")
    record("n68_reset_during_dwell", {"old": OLD, "new": NEW, "after_reset": saved})
    assert saved == pytest.approx(OLD), "the G10 offset was written to EEPROM while the dwell ran"


def test_g10_saved_after_dwell(tg, keep, record):
    keep("g55x")
    started = _dwell_then_g10(tg)
    # stat reads idle during a lone dwell (it starts no cycle), so wait_idle() alone returns early: wait the dwell out
    time.sleep(max(0.0, started + 3.0 + 1.0 - time.monotonic()))
    tg.wait_idle(timeout=10)                                 # the deferred write runs from the main loop when idle
    tg.reset()
    saved = tg.get("g55x")
    record("n68_after_dwell", {"old": OLD, "new": NEW, "after_reset": saved})
    assert saved == pytest.approx(NEW), "the G10 offset was not saved once the dwell was over"
