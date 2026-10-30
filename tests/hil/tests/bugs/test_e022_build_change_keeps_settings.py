"""E022: every firmware build-number change resets all settings to defaults.

edge config.c:130-132 compares NVM record 0 with cs.fw_build (TINYG_FIRMWARE_BUILD) and loads
defaults on any difference, so a fix released as 449.02 would wipe every customer's motor, limit and
offset settings. (It also uses fp_NE, which is false for an erased EEPROM's NaN, so a blank EEPROM is
loaded as if it were current.)

Bench: flash the same source built with another build number (make FW_BUILD=...), check a
non-default setting survived, then flash the original build back. Needs --allow-destructive.
"""
import pathlib

import pytest

from tinyg_hil import flash as flasher

pytestmark = [pytest.mark.destructive, pytest.mark.persists, pytest.mark.dialect("d449")]

SOURCE = pathlib.Path(__file__).resolve().parents[4]        # the checkout these tests belong to


@pytest.mark.bug("E022", on=[])
def test_settings_survive_a_build_number_change(tg, keep):
    keep("xvm")
    tg.set("xvm", 12345)
    fb = tg.get("fb")
    other_fb = round(fb + 0.98, 2)
    other = flasher.build(SOURCE, [f"FW_BUILD={other_fb:.2f}"])
    original = flasher.build(SOURCE)
    try:
        flasher.flash(tg, other, expect_fb=other_fb)
        assert tg.get("xvm") == 12345, "settings were reset by a firmware build-number change"
    finally:
        flasher.flash(tg, original, expect_fb=fb)
