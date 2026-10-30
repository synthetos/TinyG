"""E083: independent persisted NO/NC probe setting; no physical timing claim."""
import pytest

@pytest.mark.bug("E083", on=["d440", "d449"])
@pytest.mark.persists
@pytest.mark.resets_board
def test_probe_polarity_persists(tg, keep):
    keep("prbp")
    original = tg.get("prbp")
    changed = 1 - original
    tg.set("prbp", changed)
    tg.reset()
    assert tg.get("prbp") == changed

@pytest.mark.bug("E083", on=["d440", "d449"])
def test_probe_polarity_range(tg):
    before = tg.get("prbp")
    assert tg.send_json({"prbp": 2}).status == tg.dialect.status("INPUT_VALUE_RANGE_ERROR")
    assert tg.get("prbp") == before
