"""E046: PANIC (and SHUTDOWN) zero the machine position.

cm_panic (canonical_machine.c:843-846) calls cm_halt_motion -> mp_halt_runtime (planner.c:126-130)
-> planner_reset() = planner_init(), which memsets mr and mm (planner.c:87-99). cm_queue_flush
(canonical_machine.c:1684-1689) then copies the zeroed mr.position into the model, so mpo reads 0,
contradicting the "leaves the system inspectable" comment. The SHUTDOWN path is the same but cannot
be observed until E045 is fixed (no command is answered in SHUTDOWN).
"""
import pytest

pytestmark = pytest.mark.dialect("d449")


@pytest.mark.resets_board
@pytest.mark.bug("E046", on=["d449"])
def test_panic_keeps_machine_position(tg):
    tg.send_gcode("G28.3 X1 Y2", check=True)      # set machine position, no motion
    tg.wait_idle(5)
    assert (tg.get("mpox"), tg.get("mpoy")) == pytest.approx((1, 2))
    try:
        tg.send_json({"panic": None})
        assert tg.stat() == tg.dialect.stat("PANIC")
        assert (tg.get("mpox"), tg.get("mpoy")) == pytest.approx((1, 2))
    finally:
        tg.reset()
