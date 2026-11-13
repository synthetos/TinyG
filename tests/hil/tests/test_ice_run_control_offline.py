"""Bounded ICE run-control recovery; no debugger or board required."""
import threading

import pytest
from pyedbglib.protocols.avr8protocol import Avr8Protocol
from pyedbglib.protocols.jtagice3protocol import Jtagice3ResponseError
from tinyg_hil.ice import Ice


def illegal():
    return Jtagice3ResponseError('AVR8_FAILURE_ILLEGAL_STATE', 0x32)


class Protocol:
    def __init__(self, *, run_errors=0, stop_errors=0, stuck=False, run_applied=False):
        self.running = True
        self.run_errors, self.stop_errors = run_errors, stop_errors
        self.stuck, self.run_applied = stuck, run_applied
        self.events = []

    def stop(self):
        self.events.append('stop')
        self.running = False
        if self.stop_errors:
            self.stop_errors -= 1
            raise illegal()

    def run(self):
        self.events.append('run')
        if self.run_errors:
            self.run_errors -= 1
            if self.run_applied:
                self.running = True
            raise illegal()
        if not self.stuck:
            self.running = True

    def get_byte(self, context, offset):
        assert (context, offset) == (Avr8Protocol.AVR8_CTXT_TEST, Avr8Protocol.AVR8_TEST_TGT_RUNNING)
        self.events.append('state')
        return int(self.running)


@pytest.fixture
def make_ice(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr('tinyg_hil.ice.time.monotonic', lambda: clock[0])
    monkeypatch.setattr('tinyg_hil.ice.time.sleep', lambda delay: clock.__setitem__(0, clock[0] + max(delay, .1)))

    def make(**kw):
        ice = Ice.__new__(Ice)
        ice.protocol = Protocol(**kw)
        ice.wire = threading.Lock()
        ice._mutex, ice._halt_depth = threading.RLock(), 0
        return ice
    return make


def assert_released(ice):
    assert ice._halt_depth == 0
    assert not ice.wire.locked()


def test_run_illegal_once_recovers(make_ice):
    ice = make_ice(run_errors=1)
    with ice.halted():
        assert not ice.protocol.running
    assert ice.protocol.running
    assert ice.protocol.events == ['stop', 'run', 'state', 'run', 'state']
    assert_released(ice)


def test_run_error_after_execution_started_needs_no_second_run(make_ice):
    ice = make_ice(run_errors=1, run_applied=True)
    with ice.halted():
        pass
    assert ice.protocol.events == ['stop', 'run', 'state']
    assert_released(ice)


def test_stop_illegal_once_retries(make_ice):
    ice = make_ice(stop_errors=1)
    with ice.halted():
        assert not ice.protocol.running
    assert ice.protocol.events == ['stop', 'stop', 'run', 'state']
    assert_released(ice)


def test_nested_scope_resumes_only_outermost_even_on_body_error(make_ice):
    ice = make_ice(run_errors=1)
    with pytest.raises(ValueError, match='body failed'):
        with ice.halted():
            with ice.halted():
                assert ice.protocol.events == ['stop']
                raise ValueError('body failed')
    assert ice.protocol.running
    assert ice.protocol.events.count('stop') == 1
    assert_released(ice)


@pytest.mark.parametrize('kw', [{'stuck': True}, {'run_errors': 1000}])
def test_resume_failure_is_bounded_and_explicit(make_ice, kw):
    ice = make_ice(**kw)
    with pytest.raises(RuntimeError, match='CPU left halted'):
        with ice.halted():
            pass
    assert 1 < ice.protocol.events.count('run') < 10
    assert not ice.protocol.running
    assert_released(ice)


def test_failed_stop_still_recovers_cpu_before_propagating(make_ice):
    ice = make_ice(stop_errors=1000)
    with pytest.raises(Jtagice3ResponseError):
        with ice.halted():
            pytest.fail('failed stop must not enter the body')
    assert ice.protocol.running
    assert ice.protocol.events[-2:] == ['run', 'state']
    assert_released(ice)


def test_state_read_illegal_once_recovers(make_ice):
    ice = make_ice()
    original = ice.protocol.get_byte
    calls = [0]
    def read(*args):
        calls[0] += 1
        if calls[0] == 1:
            raise illegal()
        return original(*args)
    ice.protocol.get_byte = read
    with ice.halted():
        pass
    assert calls[0] == 2
    assert ice.protocol.running
    assert_released(ice)
