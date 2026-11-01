"""E165: resuming a dwell queued behind an M0 program stop can leave the machine in RUN for good.

canonical_machine.c cm_end_hold(): mp_exit_hold_state() clears the hold and requests the exec, which may
run the queued dwell at once (plan_exec, LO interrupt). _exec_dwell() frees the last buffer and calls
cm_cycle_end(), which does nothing because M0 already ended the cycle (cycle_state CYCLE_OFF). cm_end_hold()
then sees MOTION_RUN and calls cm_cycle_start(): stat 5 (RUN) with an empty queue, and nothing ever ends
that cycle. Whether the exec runs before or after cm_cycle_start() depends on timing, so it is
intermittent (seen in 1 of 4 sessions of test_e020's dwell case on bench 2).

No motion. The sequence is repeated; every resume must end with the machine idle.
"""
import time

import pytest

from tinyg_hil.driver import NoResponse

pytestmark = [pytest.mark.dialect("d449")]

ROUNDS = 25


@pytest.mark.bug("E165", on=[])
def test_resume_after_program_stop_ends_the_cycle(tg, record):
    stuck_at = None
    try:
        for n in range(ROUNDS):
            tg.send_gcode("M0", check=True)
            deadline = time.monotonic() + 3
            while tg.get("hold") != 6:
                assert time.monotonic() < deadline, "M0 never entered HOLD"
            tg.send_gcode("G4 P0.05", check=True)
            tg.send_raw(b"~")
            try:
                tg.wait_idle(timeout=2)
            except NoResponse:
                stuck_at = n
                break
        record("e165", {"rounds": ROUNDS, "stuck_at": stuck_at, "stat": tg.stat(), "qr": tg.get("qr")})
        assert stuck_at is None, f"round {stuck_at}: stat {tg.stat()} with qr {tg.get('qr')} after resume"
    finally:
        if stuck_at is not None:
            tg.reset()                      # the cycle never ends on its own
        else:
            tg.send_raw(b"~")
            tg.wait_idle(timeout=5)
