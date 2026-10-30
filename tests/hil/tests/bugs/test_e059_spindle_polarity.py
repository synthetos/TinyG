"""E059: there is no spindle enable/direction polarity setting.

spindle.c:108-116 hard-codes active-high enable and direction; SPINDLE_ENABLE_POLARITY and
SPINDLE_DIR_POLARITY (settings_default.h:52-53) are never referenced and config_app.c has no
spep/spdp rows (g2core names). Master lacks them too.
"""
import pytest


@pytest.mark.bug("E059", on=["d440", "d449"])
def test_polarity_tokens(tg):
    assert tg.get("spep") in (0, 1)
    assert tg.get("spdp") in (0, 1)


# Only the direction polarity is flipped: inverting spep would energize an active-high spindle output.
@pytest.mark.persists
@pytest.mark.resets_board
@pytest.mark.bug("E059", on=["d440", "d449"])
def test_dir_polarity_persists(tg, keep):
    keep("spdp")
    flipped = 1 - tg.get("spdp")
    tg.set("spdp", flipped)
    tg.reset()
    assert tg.get("spdp") == flipped
