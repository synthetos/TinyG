"""N104: invalid centre words refuse an arc before changing its target, without ALARM.

LinuxCNC G-codes 5.1 Center Format Arcs requires an incremental centre word
or both absolute centre words; a missing centre is an interpreter error:
https://linuxcnc.org/docs/html/gcode/g-code.html#_center_format_arcs
NIST RS274NGC v3 3.5.3.2 likewise requires an in-plane centre word:
https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=823374
Zero means a resolved zero-radius centre, not absolute I0 J0 at a nonzero start.
"""
import pytest

pytestmark = [pytest.mark.motion, pytest.mark.dialect("d449")]
WHY = "N104 refusal check and 0.1 mm recovery move"


@pytest.mark.parametrize("plane,endpoint,offsets", [
    ("G17", "X1 Y1", "I0 J0"),
    ("G18", "X1 Z1", "I0 K0"),
    ("G19", "Y1 Z1", "J0 K0"),
])
@pytest.mark.parametrize("direction", ["G2", "G3"])
@pytest.mark.parametrize("mode,zero", [
    pytest.param("G91.1", False, marks=pytest.mark.bug("N104", on=[])),
    pytest.param("G91.1", True, marks=pytest.mark.bug("N104", on=[])),
    # Missing absolute words already refuse correctly: keep this a passing control.
    pytest.param("G90.1", False, marks=pytest.mark.bug("N104", on=[])),
    pytest.param("G90.1", True, marks=pytest.mark.bug("N104", on=[])),
])
def test_missing_or_zero_centre_refuses_without_alarm(tg, plane, endpoint, offsets, direction, mode, zero):
    before = [tg.get("mpo" + axis) for axis in "xyz"]
    try:
        tg.send_gcode(f"G21 G90 G94 {plane} {mode}", check=True)
        # The current point is the work origin, so absolute zero also resolves
        # to a zero-radius centre. No physical positioning move is needed.
        tg.send_gcode("G92 X0 Y0 Z0", check=True)
        reply = tg.send_gcode(f"{direction} {endpoint} {offsets if zero else ''} F600", allow=WHY)
        assert reply.status == 157, reply.raw
        assert tg.stat() != tg.dialect.stat("ALARM")
        assert [tg.get("mpo" + axis) for axis in "xyz"] == pytest.approx(before, abs=0.001)
        assert tg.send_gcode("G1 X0.1 F600", allow=WHY).status == 0
        tg.wait_idle(timeout=5)
        assert tg.get("mpox") == pytest.approx(before[0] + 0.1, abs=0.002)
    finally:
        if tg.stat() == tg.dialect.stat("ALARM"):
            tg.clear_alarm()
        tg.send_gcode("G91.1", check=True)
        tg.send_gcode("G92.1", check=True)
        tg.send_gcode("G90 G53 G0 " + " ".join(f"{axis}{value:.6f}" for axis, value in zip("XYZ", before)),
                      check=True, allow=WHY)
        tg.wait_idle(timeout=5)
