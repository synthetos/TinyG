"""A host probing during EEPROM initialization must not start the board in ALARM.

Opt-in: temporarily replaces EEPROM, restoring the complete original image in
finally. Only SRAM is read while firmware may be using the NVM controller.
"""
from pathlib import Path
import time

import pytest

pytestmark = [pytest.mark.ice, pytest.mark.destructive, pytest.mark.persists,
              pytest.mark.resets_board, pytest.mark.dialect("d449"), pytest.mark.timeout(180),
              pytest.mark.bug("N93", on=["d449"])]


def _install_and_reset(ice, image):
    from pyedbglib.protocols.avr8protocol import Avr8Protocol
    with ice.halted():
        time.sleep(0.05)  # let any already-issued EEPROM byte operation finish before debugger access
        for address in range(0, len(image), 32):
            ice.write(address, image[address:address + 32], Avr8Protocol.AVR8_MEMTYPE_EEPROM_ATOMIC)
        assert ice.eeprom_read() == image
        ice.poke("config_loaded", 0)
        ice.poke("nvm.address", 0xFFFF)
        ice.protocol.reset()  # outer halted() runs the reset core, never the old partly initialized program


def _application_started(ice):
    end = time.monotonic() + 60
    while ice.peek("nvm.address") == 0xFFFF:
        assert time.monotonic() < end, "application did not start after reset"
        time.sleep(0.05)


def _boot_finished(ice):
    end = time.monotonic() + 30
    while not ice.peek("config_loaded"):
        assert time.monotonic() < end, "configuration did not finish"
        time.sleep(0.05)


@pytest.mark.parametrize("layout", ["defaults", "master"])
def test_boot_probes_do_not_raise_input_lost(tg, ice, record, layout):
    original = ice.eeprom_read()
    assert len(original) == 2048
    image = (b"\xff" * len(original) if layout == "defaults" else
             Path(__file__).with_name("n92_master_440_21.eeprom").read_bytes())
    assert len(image) == len(original)
    mark = len(tg.transport.history)
    sent = 0
    try:
        _install_and_reset(ice, image)
        _application_started(ice)  # never send probe bytes to the bootloader
        end = time.monotonic() + 30
        while not ice.peek("config_loaded"):
            assert time.monotonic() < end, "configuration stalled while the host probed"
            tg.transport.write(b'{"fb":null}\n')
            sent += 1
            time.sleep(0.05)
        assert sent >= 2, "no meaningful pre-ready traffic was injected"
        tg.drain(quiet=0.4, limit=3)
        stat = None
        for _ in range(4):
            response = tg.send_json({"stat": None}, timeout=2)
            stat = response.body.get("stat")
            if stat is not None:
                break
        history = tg.transport.history[mark:]
        lost = [e.raw for e in history if
                (e.kind == "exception" and e.data.get("er", {}).get("st") == 13) or
                (e.kind == "response" and len(e.data.get("f", [])) > 1 and e.data["f"][1] == 13)]
        record("n93_boot", {"layout": layout, "early_queries": sent, "stat": stat, "input_lost": lost})
        assert not lost, f"startup input was reported lost: {lost[:3]}"
        assert stat in tg.dialect.idle_stats, f"startup did not leave an answering idle board: {stat}"
        assert tg.get("fb") is not None, "a clean retry after boot was not answered"
    finally:
        _install_and_reset(ice, original)
        _application_started(ice)
        _boot_finished(ice)
        tg.drain(quiet=0.4, limit=3)
        assert tg.get("fb") is not None, "the original EEPROM image did not recover"
