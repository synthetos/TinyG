"""N22: coordinate offsets and resolved targets must leave finite geometry. TinyG wiki Coordinate-Systems and NIST RS274NGC v3 3.5.1/3.5.2/3.5.5 define offsets and target positions. A 1e18 canonical-unit bound leaves headroom for squaring six differences of two accepted endpoints; reject rather than publish a NaN plan."""
import pytest
pytestmark=[pytest.mark.persists,pytest.mark.dialect("d449"),pytest.mark.bug("N22",on=["d449"])]
def test_huge_coordinate_offset_refused(tg,keep):
    keep("g54x","g54a")
    tg.send_gcode("G21",check=True)
    for token in ("g54x","g54a"):
        before=tg.get(token)
        reply=tg.send_json({token:1e30})
        assert reply.status==tg.dialect.status("INPUT_VALUE_RANGE_ERROR"),reply.raw
        assert tg.get(token)==pytest.approx(before)
