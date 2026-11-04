"""N23: failed group children must not be persisted or reported as accepted. TinyG wiki JSON-Details and TinyG-Configuration-for-Firmware-Version-0.97: grouped settings use the same setting validation and response status as single settings. Earlier successful children remain applied; groups are not transactions."""
import pytest
pytestmark=[pytest.mark.persists,pytest.mark.dialect("d449"),pytest.mark.bug("N23",on=["d449"])]
def test_group_refuses_bad_child(tg,keep):
    keep("xjm")
    tg.send_gcode("G21",check=True)
    tg.send_json({"xjm":20},check=True)
    reply=tg.send_json({"x":{"jm":0}})
    assert reply.status==tg.dialect.status("INPUT_LESS_THAN_MIN_VALUE"),reply.raw
    assert tg.get("xjm")==pytest.approx(20)
