"""E084: read-only boot stack watermark; observed margin is not a worst-case bound."""
import pytest

@pytest.mark.bug("E084", on=["d440"])
def test_stack_watermark_is_bounded_and_read_only(tg):
    before = tg.get("fstk")
    assert isinstance(before, (int, float)) and 0 <= before <= 16384
    tg.get("sys")
    after = tg.get("fstk")
    assert 0 <= after <= before
    assert tg.send_json({"fstk": 100}).status != 0
