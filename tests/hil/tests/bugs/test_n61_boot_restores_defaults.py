"""N61: a saved setting that will not load at boot runs on its default, and the board reports it with status 118.

Maintainer requirement 2026-11-04: an EEPROM value that would leave a board unusable must never be kept in effect; the
default is restored and the host is told. Stock edge loads the default too, but reports only the first failing
setting with status 90 "config assertion failure", and a default that nv_set() refuses (as a draft of N60 did for
every setting at boot) left the setting at a stale or zero value: that board came up with JSON verbosity 0, silent.

The test finds the EEPROM slot of $1mi by changing it and diffing EEPROM through the ICE, stores 3 there (microsteps
accept only 1, 2, 4 and 8), resets, and expects: the board answers, $1mi is a supported value, and one exception report
with status 118 "Settings restored to defaults" names 1mi. The EEPROM value is left for the host to replace (the
firmware does not overwrite it); the test restores it. Needs --ice.
"""
import time

import pytest

pytestmark = [pytest.mark.ice, pytest.mark.persists, pytest.mark.resets_board, pytest.mark.dialect("d449"),
              pytest.mark.timeout(240), pytest.mark.bug("N61", on=[])]

RESTORED = 118


def _eeprom_slot(tg, ice, token, a, b):
    """EEPROM address of a setting: the 4-byte word that changes when the setting goes from a to b."""
    tg.set(token, a, allow="find the EEPROM slot")
    before = ice.eeprom_read()
    tg.set(token, b, allow="find the EEPROM slot")
    after = ice.eeprom_read()
    changed = sorted({i - i % 4 for i in range(len(before)) if before[i] != after[i]})
    assert len(changed) == 1, f"expected one changed EEPROM word for {token}, got {changed}"
    return changed[0]


def test_invalid_saved_setting_restores_default_and_reports(tg, keep, ice, record):
    from pyedbglib.protocols.avr8protocol import Avr8Protocol
    keep("1mi")
    original = tg.get("1mi")
    other = 4 if original != 4 else 2
    addr = _eeprom_slot(tg, ice, "1mi", other, original)
    saved = ice.eeprom_read(addr, 4)
    try:
        ice.write(addr, (3).to_bytes(4, "little"), Avr8Protocol.AVR8_MEMTYPE_EEPROM)   # not a supported microstep
        assert ice.eeprom_read(addr, 4) == (3).to_bytes(4, "little")
        mark = len(tg.transport.history)
        tg.reset()
        time.sleep(0.5)
        tg.drain()
        reports = [e for e in tg.transport.history[mark:] if e.kind == "exception"]
        loaded = tg.get("1mi")
        eeprom_after = ice.eeprom_read(addr, 4)
        record("n61", {"addr": addr, "original": original, "loaded": loaded, "eeprom_after": eeprom_after.hex(),
                       "reports": [e.raw for e in reports]})
        assert loaded in (1, 2, 4, 8), f"1mi is {loaded} after boot"
        assert eeprom_after == (3).to_bytes(4, "little"), "the firmware overwrote the saved value"
        mine = [e for e in reports if e.data["er"].get("st") == RESTORED]
        assert len(mine) == 1 and "1mi" in mine[0].data["er"].get("msg", ""), \
            f"expected one status {RESTORED} report naming 1mi: {[e.raw for e in reports]}"
    finally:
        ice.write(addr, saved, Avr8Protocol.AVR8_MEMTYPE_EEPROM)
        tg.reset()
