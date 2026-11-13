"""Reset ordering and new-boot observations without a debugger."""
from contextlib import contextmanager
from pathlib import Path
import runpy
import struct
from types import SimpleNamespace

import pytest

NS = runpy.run_path(str(Path(__file__).parent / 'bugs/test_n92_master_upgrade_power_loss.py'))


class Fake:
    def __init__(self, marker=10003):
        self.events = []
        self.depth = 0
        self.reset_held = False
        self.memory = {'config_loaded': 1, 'nvm.address': 512}
        self.pc_value = 0x1000
        self.image = struct.pack('<f', marker) + bytes(2044)
        self.protocol = SimpleNamespace(reset=self.reset, get_byte=lambda *args: 1)

    @contextmanager
    def halted(self):
        self.depth += 1
        try:
            yield
        finally:
            self.depth -= 1
            if not self.depth:
                self.events.append(('run', self.reset_held))

    def reset(self):
        assert self.depth, 'reset must occur before releasing the old app'
        assert self.memory == {'config_loaded': 0, 'nvm.address': 65535}
        self.events.append(('reset',))
        self.reset_held = True

    def _retry(self, fn, *args):
        return fn(*args)

    def poke(self, key, value):
        self.events.append(('poke', key))
        self.memory[key] = value

    def peek(self, key):
        return self.memory[key]

    def pc(self):
        return self.pc_value

    def eeprom_read(self):
        self.events.append(('eeprom',))
        return self.image


def test_poison_reset_then_first_resume():
    ice = Fake()
    NS['_reset'](ice)
    assert ice.events == [('poke', 'config_loaded'), ('poke', 'nvm.address'),
                          ('reset',), ('run', True)]


def test_start_requires_application_pc_and_cleared_sentinel(monkeypatch):
    ice = Fake()
    ice.pc_value = 192 * 1024
    sleeps = []
    def advance(_):
        sleeps.append(1)
        if len(sleeps) == 1:
            ice.pc_value = 100
            ice.memory['nvm.address'] = 65535
        else:
            ice.memory['nvm.address'] = 0
    monkeypatch.setattr(NS['time'], 'sleep', advance)
    NS['_started'](ice)
    assert len(sleeps) == 2


def test_failure_has_cut_and_pre_eeprom_observation(monkeypatch):
    ice = Fake(marker=440.21)
    monkeypatch.setattr(NS['time'], 'sleep', lambda _: None)
    records = []
    def record(key, value):
        assert not any(e[0] == 'eeprom' for e in ice.events)
        records.append((key, value))
    with pytest.raises(AssertionError, match="'delay': 1.2"):
        NS['_finish'](ice, record=record, cut={'delay': 1.2, 'row': 128})
    assert records[0][1] == {'cut': {'delay': 1.2, 'row': 128},
                            'running_before_halt': 1, 'pc': 4096,
                            'config_loaded': 1, 'nvm_address': 512}
