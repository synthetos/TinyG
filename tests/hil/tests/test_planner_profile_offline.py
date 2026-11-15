"""Malformed or overflowed timing records must never become small CPU bounds."""
import pytest
from tinyg_hil.planner_profile import decode_sample


def sample(**changes):
    values=dict(calls=100,overflows=0,maximum_ticks=123,site=6,section_at_maximum=31)
    values.update(changes)
    return {'motion_profile.'+key:value for key,value in values.items()}


def test_two_microsecond_sites_and_queue_tag():
    for site,scope in ((6,'lookahead_pass'),(7,'one_trapezoid')):
        row=decode_sample(sample(site=site))
        assert row['maximum_us']==246
        assert row['queued_at_maximum']==31
        assert row['scope']==scope


@pytest.mark.parametrize('change',[dict(overflows=1),dict(maximum_ticks=65535),
    dict(calls=0),dict(maximum_ticks=0),dict(site=5),dict(section_at_maximum=33)])
def test_reject_unusable_sample(change):
    with pytest.raises(ValueError):
        decode_sample(sample(**change))
