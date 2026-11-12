"""A command-mode AVR109 fake must never receive application traffic."""
import pytest

from tinyg_hil import bootloader, flash
from tinyg_hil.transport import Transport


class AVR109:
    def __init__(self):
        self.sent = []
        self.erases = 0
        self.lock_writes = 0
        self.lock_argument = False
        self.reply = b''
        self.closed = False
    def reset_input_buffer(self): self.reply = b''
    def close(self): self.closed = True
    def write(self, data):
        self.sent.append(data)
        for byte in data:
            if self.lock_argument:
                self.lock_writes += 1
                self.lock_argument = False
            elif byte == ord('e'): self.erases += 1
            elif byte == ord('l'): self.lock_argument = True
            elif byte == ord('S'): self.reply += b'XBoot++'
    def read(self, count):
        value, self.reply = self.reply[:count], self.reply[count:]
        return value


@pytest.fixture
def boot(monkeypatch, tmp_path):
    monkeypatch.setenv('TINYG_HIL_STATE', str(tmp_path/'state'))
    device = AVR109()
    monkeypatch.setattr(bootloader.serial, 'Serial', lambda *a, **kw: device)
    return device


def test_fake_reproduces_destructive_json():
    device = AVR109()
    device.write(b'{"e":null}\n')
    assert device.erases and device.lock_writes


def test_session_open_and_teardown_never_feed_bootloader_text(boot):
    transport = Transport('fake-guard-port')
    with pytest.raises(bootloader.BootloaderActive): transport.open()
    assert boot.sent == [b'\x1bS']
    for payload in (b'{"e":null}\n', b'G1 X1\n', b'\x18', b'\x11\n'):
        with pytest.raises(bootloader.BootloaderActive): transport.write(payload)
    transport.close()
    with pytest.raises(bootloader.BootloaderActive): Transport('fake-guard-port').open()
    assert boot.sent == [b'\x1bS'] and boot.closed
    assert not boot.erases and not boot.lock_writes


def test_failed_programming_keeps_next_session_quarantined(boot, monkeypatch):
    transport = Transport('fake-guard-port')
    class Board:
        def __init__(self): self.transport = transport
        def close(self): self.transport.close()
        def connect(self, **kwargs): pytest.fail('application reconnect after failed flash')
    monkeypatch.setattr(flash.time, 'sleep', lambda _: None)
    def fail(cmd, timeout): raise RuntimeError('programming interrupted')
    monkeypatch.setattr(flash, '_program', fail)
    with pytest.raises(RuntimeError, match='programming interrupted'):
        flash.flash(Board(), 'unused.hex')
    with pytest.raises(bootloader.BootloaderActive): transport.write(b'{"e":null}\n')
    with pytest.raises(bootloader.BootloaderActive): Transport('fake-guard-port').open()
    assert boot.sent == [b'\x1bS']
    assert not boot.erases and not boot.lock_writes


def test_success_exit_check_still_rejects_bootloader(boot, monkeypatch):
    transport = Transport('fake-guard-port')
    class Board:
        def __init__(self): self.transport = transport
        def close(self): self.transport.close()
        def connect(self, **kwargs): pytest.fail('application reconnect while XBoot++ answers')
    monkeypatch.setattr(flash.time, 'sleep', lambda _: None)
    monkeypatch.setattr(flash, '_program', lambda *a: None)
    with pytest.raises(bootloader.BootloaderActive, match='still active'):
        flash.flash(Board(), 'unused.hex')
    assert boot.sent == [b'\x1bS', b'\x1bS']
    assert bootloader.marker(transport.port).exists()
    assert not boot.erases and not boot.lock_writes


def test_cli_recovery_skips_all_application_bootstrap(monkeypatch, tmp_path):
    from types import SimpleNamespace as NS
    from tinyg_hil import __main__ as cli
    events = []
    class Lock:
        def __init__(self, port): pass
        def acquire(self, *a, **kw): return self
        def release(self): events.append('release')
    class Board:
        def __init__(self, *a, **kw): self.identity = {}; self.dialect = NS(name='d449')
        def connect(self): pytest.fail('recovery CLI must not bootstrap application')
        def close(self): events.append('close')
    monkeypatch.setattr(cli, 'BoardLock', Lock)
    monkeypatch.setattr(cli, 'TinyG', Board)
    def recover(tg, *a, **kw): events.append('AVR109'); return {'fb':450.04}
    monkeypatch.setattr(cli.flasher, 'flash', recover)
    cli.main(['flash','--port','fake','--hex',str(tmp_path/'image.hex'),'--no-snapshot','--no-restore'])
    assert events == ['AVR109', 'close', 'release']


def test_quarantined_silent_board_requires_ice_without_text(boot, monkeypatch):
    transport = Transport('fake-guard-port')
    bootloader.quarantine(transport.port, 'previous failed flash')
    class Board:
        def __init__(self): self.transport = transport
        def close(self): self.transport.close()
    boot.read = lambda count: b''
    monkeypatch.setattr(flash, '_program', lambda *a: pytest.fail('cannot synchronize AVR109'))
    with pytest.raises(bootloader.BootloaderActive): flash.flash(Board(), 'unused.hex')
    assert boot.sent == [b'\x1bS'] and boot.closed
    assert not boot.erases and not boot.lock_writes
