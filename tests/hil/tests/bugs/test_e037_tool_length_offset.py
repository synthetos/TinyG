"""E037: the G43.1 tool length offset is subtracted again on every G91 move, applies under G53, and G92 ignores it.

edge canonical_machine.c:498 `cm.gm.target[axis] -= cm.gmx.tool_offset[axis]; //+++++ TEST THIS` sits
outside the absolute/incremental branch of cm_set_model_target() (:493-497), and
cm_get_active_coord_offset() returns 0 under G53 (:307-308) while the TLO is still subtracted.
G92 (:1045-1046) computes the origin offset without the TLO term, but the work position adds it (:382).
Master has no G43.1, so these run on 449 only. The sign convention itself is not asserted here.
"""
import pytest

pytestmark = pytest.mark.dialect("d449")

MOVE = "tool offset test (Z <= 1 mm, returns to start)"


def _cleanup(tg, mpoz):
    if tg.stat() == tg.dialect.stat("ALARM"):
        tg.clear_alarm()
    tg.send_gcode("G49", check=True)    # M2 does not cancel the tool offset
    tg.send_gcode(f"G90 G53 G0 Z{mpoz:.4f}", allow=MOVE)
    tg.wait_idle()


@pytest.mark.motion
@pytest.mark.bug("E037", on=["d449"])
def test_incremental_move_ignores_tool_offset(tg):
    z0 = tg.get("mpoz")
    try:
        tg.send_gcode("G43.1 Z1", check=True)
        tg.send_gcode("G91 G0 Z0.5", check=True, allow=MOVE)
        tg.wait_idle()
        assert tg.get("mpoz") == pytest.approx(z0 + 0.5, abs=0.01)
    finally:
        _cleanup(tg, z0)


@pytest.mark.motion
@pytest.mark.bug("E037", on=["d449"])
def test_g53_ignores_tool_offset(tg):
    z0 = tg.get("mpoz")
    try:
        tg.send_gcode("G43.1 Z1", check=True)
        tg.send_gcode(f"G90 G53 G0 Z{z0:.4f}", check=True, allow=MOVE)      # already there: no motion
        tg.wait_idle()
        assert tg.get("mpoz") == pytest.approx(z0, abs=0.01)
    finally:
        _cleanup(tg, z0)


@pytest.mark.bug("E037", on=["d449"])
def test_g92_includes_tool_offset(tg):
    try:
        tg.send_gcode("G43.1 Z10", check=True)
        tg.send_gcode("G92 Z0", check=True)
        tg.wait_idle()
        assert tg.get("posz") == pytest.approx(0, abs=0.001)
    finally:
        tg.send_gcode("G92.1", check=True)
        tg.send_gcode("G49", check=True)
