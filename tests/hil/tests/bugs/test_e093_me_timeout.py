"""E093: {me:N} selects a motor instead of setting an energize timeout, so motors cannot be held on past $mt.

st_set_me (stepper.c:1284-1296) treats a non-zero value as a motor number and returns
STAT_INPUT_VALUE_RANGE_ERROR when it exceeds MOTORS; the power callback (stepper.c:473-495) then
times out after $mt regardless. g2core and the TinyG wiki (`$me=180`) use seconds. Master is the same.
"""
import pytest


@pytest.mark.bug("E093", on=["d440"])
def test_me_accepts_timeout_seconds(tg):
    try:
        assert tg.send_json({"me": 30}).status == 0
    finally:
        tg.send_json({"md": None})
