"""Diagnostic (ICE): the chip's device ID and silicon revision (MCU.DEVID0-2, MCU.REVID), read in one halt.

Some xmega errata (AVR1008: EEPROM writes on revision B) depend on the silicon revision, so fixes that work around
them need to know which revisions the benches carry. Reads only; no reset, no write. The device ID is checked
against the ATxmega192A3 signature so a wrong read path cannot pass as a revision.
"""
import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice, pytest.mark.dialect("d449")]

MCU_BASE = 0x0090          # MCU control registers in the data space (xmega A manual: DEVID0..2 at +0..2, REVID at +7)
XMEGA192A3 = bytes([0x1E, 0x97, 0x44])


def test_mcu_revid(tg, ice, record):
    regs = ice.read(MCU_BASE, 8)
    devid, revid = regs[0:3], regs[7]
    letter = chr(ord("A") + revid) if revid < 26 else "?"
    result = {"devid": devid.hex(), "revid": revid, "revision": letter, "port": tg.transport.port}
    record("mcu_revid", result)
    print("MCU", result)
    assert devid == XMEGA192A3, f"device ID {devid.hex()} is not an ATxmega192A3: read path wrong? {result}"
