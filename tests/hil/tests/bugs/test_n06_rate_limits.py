"""N06: zero xVM/xFR reaches division in plan_line.c:_calc_move_times.

TinyG wiki TinyG-Configuration-for-Firmware-Version-0.97, xVM/xFR and xAM:
rate caps constrain motion; disabling an axis is xAM=0. Reject invalid rates
before storing/persisting them. No motion is needed to expose the defect.
"""
import pytest

pytestmark = [pytest.mark.persists, pytest.mark.dialect("d449"),
              pytest.mark.bug("N06", on=[])]

@pytest.mark.parametrize("token", [a+s for a in "xyzabc" for s in ("vm", "fr")])
@pytest.mark.parametrize("value", [0, -1])
def test_rate_cap_rejected_without_changing_setting(tg, keep, token, value):
    keep(token)
    before = tg.get(token)
    reply = tg.send_json({token: value})
    assert reply.status == tg.dialect.status("INPUT_LESS_THAN_MIN_VALUE"), reply.raw
    assert tg.get(token) == pytest.approx(before)
