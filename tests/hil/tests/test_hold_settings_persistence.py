"""The activated layout-2 hold settings retain user choices across a reset."""
import pytest

@pytest.mark.dialect("d449")
@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.parametrize("token,value", [("spph", 0), ("spde", 0.25), ("coph", 1)])
def test_hold_settings_survive_reset(tg, keep, token, value):
    keep(token)
    tg.set(token, value)
    tg.reset()
    assert tg.get(token) == pytest.approx(value)
