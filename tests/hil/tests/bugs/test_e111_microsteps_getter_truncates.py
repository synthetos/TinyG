"""E111: {"1mi":256} is accepted (with a warning) but reads back 0, because the getter reads 1 byte of a uint32_t.

config_app.c:189 `1mi ... get_ui8, st_set_mi` over `uint32_t microsteps` (stepper.h:343); st_set_mi
stores all 32 bits (stepper.c:1201) and get_ui8 reads the low byte (config.c:276). Master is the same.
"""
import pytest


@pytest.mark.persists
@pytest.mark.bug("E111", on=["d440", "d449"])
def test_microsteps_256_reads_back_or_is_rejected(tg, keep):
    keep("1mi")
    resp = tg.send_json({"1mi": 256})
    assert resp.status != 0 or tg.get("1mi") == 256
