"""E104: {"hp":null} reads 3 bytes of the float hv written next to it, so hv=7 makes hp 3758096385.

hp is get_u32 over a uint8_t (config_app.c:104, config.c:290); hv is set_flt over a uint8_t
(config_app.c:105, hardware.c:231, config.c:388), which spills 7.0f's bytes into hp's read window.
"""
import pytest


@pytest.mark.persists
def test_hp_after_hv7(tg, keep):
    if tg.get("hv") < 7:
        pytest.skip("hv=7 only binds ports like hv=8 on v7/v8 hardware (hardware.c:61)")
    keep("hv")
    tg.set("hv", 7)
    assert tg.get("hp") == 1
