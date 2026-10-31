"""E110: in G20, atn is stored x25.4 (set_flu, no F_CONVERT), ara is displayed /25.4 (F_CONVERT, set_flt),
and ajm is stored x25.4 (cm_set_xjm converts every axis), though rotary axes are in degrees.

config_app.c:299 `atn _fip ... set_flu` vs :300 `atm _fip ... set_flt`; :304 `ara _fipc ... set_flt`;
set_flu converts inches to mm (config_app.c:739-743) but only F_CONVERT rows convert back for display
(config_app.c:764). Master has identical rows.
"""
import pytest

pytestmark = pytest.mark.persists


def _round_trip(tg, keep, units, token, value):
    keep(token)
    tg.send_gcode(units, check=True)
    try:
        tg.set(token, value)
        return tg.get(token)
    finally:
        tg.send_gcode("G21", check=True)


@pytest.mark.bug("E110", on=["d440"])
@pytest.mark.parametrize("token, value", [("atn", -1.0), ("ara", 5.305), ("ajm", 324000.0)])
def test_rotary_setting_round_trips_in_inches(tg, keep, token, value):
    assert _round_trip(tg, keep, "G20", token, value) == pytest.approx(value, abs=1e-3)


@pytest.mark.parametrize("token, value", [("atn", -1.0), ("ara", 5.305), ("ajm", 324000.0)])
def test_rotary_setting_round_trips_in_mm(tg, keep, token, value):
    assert _round_trip(tg, keep, "G21", token, value) == pytest.approx(value, abs=1e-3)
