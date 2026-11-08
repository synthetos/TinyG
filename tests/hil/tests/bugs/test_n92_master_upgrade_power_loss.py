"""N92: a board upgraded from master keeps its settings even when the power goes during the migration.

The image is the EEPROM of a real master 440.21 board (bench 3) on which xvm=12345, zjm=987, g55x=12.5, jv=2 and
zsn=0 were set first. The test writes it through the ICE, resets into this build and lets the migration run to the
end: that EEPROM is the reference. Then it writes the image again for each cut, resets, waits a little longer each
time, resets again - the power loss - and lets the next boot finish. The delays sweep the whole migration, from
before the journal to after the layout marker. Every interrupted upgrade must end with the reference EEPROM byte
for byte; one cut is repeated during the resumed boot. Before the fix, a boot after a cut re-read master records
that edge values had replaced (edge 4pm sits on master 2ma: motor 2 moved from Y to X).

While the firmware may be writing, the test reads SRAM only: a debugger EEPROM access goes through the NVM
controller the firmware is using, and polling EEPROM held the first migration at its start for a minute. The end
of a boot is config_loaded; where a cut landed is the last EEPROM address the firmware used (nvm.address).

Needs --ice and --allow-destructive: it replaces the whole EEPROM. The session restores the settings afterwards.
"""
import pathlib
import struct
import time

import pytest

pytestmark = [pytest.mark.ice, pytest.mark.persists, pytest.mark.resets_board, pytest.mark.destructive,
              pytest.mark.dialect("d449"), pytest.mark.timeout(3600), pytest.mark.bug("N92", on=[])]

IMAGE = pathlib.Path(__file__).with_name("n92_master_440_21.eeprom")
PAGE = 32                   # ATxmega192A3 EEPROM page
DONE = 10002.0              # the edge layout marker, written last
STEP = 0.08                 # seconds between cut delays, from the firmware's start
NOT_RUN = 0xFFFF            # nvm.address before a reset; the startup code clears it


def _load(ice, image):
    from pyedbglib.protocols.avr8protocol import Avr8Protocol
    with ice.halted():
        for a in range(0, len(image), PAGE):
            ice.write(a, image[a:a + PAGE], Avr8Protocol.AVR8_MEMTYPE_EEPROM_ATOMIC)
    assert ice.eeprom_read() == image, "the EEPROM image did not take"


def _reset(ice):
    """Reset and run, with config_loaded cleared (so the flag left in SRAM is not read as the end of the next boot) and
    nvm.address set to a value the firmware's startup code clears."""
    with ice.halted():
        ice.poke("config_loaded", 0)
        ice.poke("nvm.address", NOT_RUN)
    ice.reset()


def _started(ice, timeout=60.0):
    """Wait, reading SRAM only, until the firmware has started: after a debugger reset bench 3's bootloader held the
    CPU about 6 s while polled, and past 13 s when left alone. Cuts are timed from here."""
    end = time.monotonic() + timeout
    while ice.peek("nvm.address") == NOT_RUN:
        assert time.monotonic() < end, "the firmware never started after the reset"
        time.sleep(0.05)
    return time.monotonic()


def _finish(ice, timeout=60.0):
    """Wait for the boot to end (SRAM only), then read the EEPROM it left."""
    end = time.monotonic() + timeout
    while not ice.peek("config_loaded"):
        assert time.monotonic() < end, f"the boot never finished (nvm.address {ice.peek('nvm.address')})"
        time.sleep(0.05)
    time.sleep(0.5)
    reads = [ice.eeprom_read()]
    for _ in range(4):                  # one debugger read once returned record 0 as master after a finished boot
        reads.append(ice.eeprom_read())
        if reads[-1] == reads[-2]:
            break
    eeprom = reads[-1]
    assert struct.unpack("<f", eeprom[:4])[0] == DONE, \
        f"the boot finished without the layout marker: {len(reads)} reads, record 0 " \
        f"{[r[:4].hex(' ') for r in reads]}, slots {eeprom[2040:2048].hex(' ')}"
    if reads[0] != eeprom:
        print(f"first EEPROM read differed at records {sorted({i // 4 for i in range(2048) if reads[0][i] != eeprom[i]})[:12]}",
              flush=True)
    return eeprom


def _cut(ice, delay):
    """Reset, wait until the firmware starts, let it run `delay` seconds untouched, and reset again: the power loss."""
    _reset(ice)
    _started(ice)
    time.sleep(delay)
    with ice.halted():
        at = {"delay": round(delay, 2), "row": ice.peek("nvm.address") // 4, "loaded": bool(ice.peek("config_loaded"))}
    _reset(ice)
    return at


def _differs(got, reference):
    return sorted({i // 4 for i in range(len(got)) if got[i] != reference[i]})[:12]


def test_power_loss_during_master_upgrade_resumes(tg, ice, record):
    image = IMAGE.read_bytes()
    assert len(image) == 2048 and struct.unpack("<f", image[:4])[0] == pytest.approx(440.21, abs=1e-4)
    _load(ice, image)
    _reset(ice)
    start = _started(ice)
    reference = _finish(ice)
    boot = time.monotonic() - start                     # from the firmware's start to the end of the migration
    cuts = []
    delay, finished = 0.0, 0
    while finished < 2:                                 # until two cuts land after the boot ended
        _load(ice, image)
        at = _cut(ice, delay)
        got = _finish(ice)
        cuts.append(at)
        print(f"cut {len(cuts)}: {at} same={got == reference}", flush=True)
        assert got == reference, f"cut {at}: EEPROM differs from the uninterrupted upgrade at records " \
                                 f"{_differs(got, reference)}"
        finished = finished + 1 if at["loaded"] else 0
        delay += STEP
        assert delay < boot + 2, f"cuts never reached the end of the boot ({boot:.2f} s): {cuts}"
    mid = [c for c in cuts if not c["loaded"]]
    _load(ice, image)                                   # cut, then cut the resumed boot too
    first = _cut(ice, mid[len(mid) // 3]["delay"])
    second = _cut(ice, mid[2 * len(mid) // 3]["delay"])
    got = _finish(ice)
    record("n92_power_loss", {"boot_s": round(boot, 2), "cuts": cuts, "double": [first, second]})
    assert got == reference, f"a second cut {second} after {first} changed the result at records " \
                             f"{_differs(got, reference)}"
    assert len(mid) >= 10, f"too few cuts landed during the migration: {cuts}"
    tg.reset()
    time.sleep(0.5)
    tg.drain()
    assert tg.get("xvm") == pytest.approx(12345) and tg.get("zjm") == pytest.approx(987)
    assert tg.get("g55x") == pytest.approx(12.5) and tg.get("jv") == 3 and tg.get("zsn") == 4
