"""E112: setters narrow to uint8_t without a range check; {"gco":7} is accepted and later indexes cm.offset[7].

config_app.c:496 `gco ... get_ui8, set_ui8` has no range check; canonical_machine.c:585 applies the
default coordinate system at every reset/M2, reading past `float offset[COORDS+1][AXES]`
(canonical_machine.h:569). The me/md (stepper.c:1275-1293) and baud (config_app.c:938) cases corrupt
RAM or drop the link to 9600 on stock firmware, so they are not sent here.
"""
import pytest


@pytest.mark.persists
@pytest.mark.bug("E112", on=["d440"])
def test_gco_out_of_range_rejected(tg, keep):
    keep("gco")
    original = tg.get("gco")
    resp = tg.send_json({"gco": 7})
    assert resp.status != 0
    assert tg.get("gco") == original
