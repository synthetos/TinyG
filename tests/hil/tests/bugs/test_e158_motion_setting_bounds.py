"""E158: zero or negative motor and jerk settings are accepted and saved.

stepper.c st_set_sa()/st_set_tr() store the value and compute steps_per_unit = 360*mi / (tr*sa), so
0 makes it infinite and a negative value flips the direction; st_set_mi() only warns and stores 0
(no steps at all). canonical_machine.c cm_set_xjm()/cm_set_xjh() take 0 or a negative jerk. Each is
now refused with 108 and the stored value is unchanged.
"""
import pytest

pytestmark = [pytest.mark.persists, pytest.mark.dialect("d449")]

BAD = [("1sa", 0), ("1sa", -1.8), ("1tr", 0), ("1tr", -1), ("1mi", 0), ("xjm", 0), ("xjm", -5), ("xjh", 0), ("xjh", -5)]


@pytest.mark.bug("E158", on=["d449"])
def test_non_positive_motion_settings_are_refused(tg, keep):
    keep("1sa", "1tr", "1mi", "xjm", "xjh")
    low = tg.dialect.status("INPUT_LESS_THAN_MIN_VALUE")
    for token, value in BAD:
        before = tg.get(token)
        assert tg.send_json({token: value}).status == low, (token, value)
        assert tg.get(token) == pytest.approx(before), f"{token}={value} changed the setting"
