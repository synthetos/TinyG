"""E020: M0/M1/M60 do not pause; the next queued move starts without a cycle start ('~').

edge canonical_machine.c:1792-1796 cm_program_stop() only queues _exec_program_finalize(MACHINE_PROGRAM_STOP),
the same idle state cm_cycle_end uses, and that clears the hold (:1749-1755). plan_exec.c:62-65 then sets
MOTION_RUN for the next ALINE, so nothing waits for '~'. Master does the same (canonical_machine.c:1396-1400,
plan_exec.c mp_exec_move).

The at-rest position is the model's (canonical_machine.c:145), which already includes the queued move, so the
test watches the planner queue instead: after M0 the second move must still be waiting.
"""
import time

import pytest

MOVE = "two 1 mm +X moves separated by M0, then back"


@pytest.mark.motion
@pytest.mark.bug("E020", on=["d440"])
@pytest.mark.parametrize("stop", ["M0", "M1", "M60"])     # M1 obeys the session optional-stop switch
def test_program_stop_waits_for_cycle_start(tg, keep, stop):
    keep("os")
    tg.set("os", int(stop == "M1"))
    x0 = tg.get("mpox")
    tg.send_gcode("G91 G1 X1 F300", check=True, allow=MOVE)
    tg.send_gcode(stop, check=True)
    tg.send_gcode("G1 X1", check=True, allow=MOVE)
    try:
        time.sleep(2.0)                      # the first move takes ~0.3 s
        assert tg.get("qr") < tg.planner_buffers, "second move ran without a cycle start"
        assert tg.get("hold") == 6, "program stop did not enter HOLD"
        tg.send_raw(b"~")
        tg.wait_idle(timeout=10)
        assert tg.get("mpox") == pytest.approx(x0 + 2, abs=0.03)
    finally:
        tg.send_raw(b"~")
        tg.wait_idle(timeout=10)
        tg.send_gcode("G91 G1 X-2 F600", allow=MOVE)
        tg.wait_idle()


@pytest.mark.bug("E020", on=["d440"])
def test_program_stop_blocks_queued_dwell(tg):
    """A pause gates non-motion blocks too; no motor motion is needed."""
    tg.send_gcode("M0", check=True)
    try:
        deadline = time.monotonic() + 3
        while tg.get("hold") != 6:
            assert time.monotonic() < deadline, "M0 never entered HOLD"
        tg.send_gcode("G4 P0.1", check=True)
        assert tg.get("qr") < tg.planner_buffers, "dwell bypassed program stop"
        assert tg.get("hold") == 6
        tg.send_raw(b"~")
        tg.wait_idle(timeout=10)
        assert tg.get("qr") == tg.planner_buffers
    finally:
        tg.send_raw(b"~")
        tg.wait_idle(timeout=10)


@pytest.mark.dialect("d449")
def test_optional_stop_disabled_does_not_pause(tg, keep):
    keep("os")
    tg.set("os", 0)
    try:
        tg.send_gcode("M1", check=True)
        tg.send_gcode("G4 P0.1", check=True)
        tg.wait_idle(timeout=5)
        assert tg.get("hold") == 0
        assert tg.get("qr") == tg.planner_buffers
    finally:
        tg.send_raw(b"~")
        tg.wait_idle(timeout=5)


@pytest.mark.dialect("d449")
@pytest.mark.resets_board
def test_optional_stop_is_session_only(tg, keep):
    keep("os")
    tg.set("os", 1)
    tg.reset()
    assert tg.get("os") == 0
