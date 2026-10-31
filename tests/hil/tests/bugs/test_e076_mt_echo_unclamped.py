"""E076: {"mt":...} clamps the motor timeout but echoes the unclamped input.

stepper.c:1264-1268 st_set_mt() stores min(MAX, max(value, MIN)) in st_cfg.motor_power_timeout
but leaves nv->value_flt alone, so the response (and the persisted value) carry the raw input.
"""
import pytest

pytestmark = pytest.mark.persists


@pytest.mark.bug("E076", on=[])
def test_mt_echo_matches_clamped_value(tg, keep):
    keep("mt")
    echoed = tg.set("mt", 0)
    assert echoed == pytest.approx(tg.get("mt"))


def test_mt_in_range_round_trips(tg, keep):
    keep("mt")
    assert tg.set("mt", 3) == pytest.approx(3)
    assert tg.get("mt") == pytest.approx(3)


def test_group_set_echo_matches_clamped_value(tg, keep):
    """A group SET ({sys:{mt:0}}) sets children through set_grp; its echo must use the row precision too."""
    keep("mt")
    resp = tg.send_json({"sys": {"mt": 0}}, check=True)
    assert resp.body["sys"]["mt"] == pytest.approx(tg.get("mt"))
