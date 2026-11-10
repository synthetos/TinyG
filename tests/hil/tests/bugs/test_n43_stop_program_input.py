"""N43/N36: stop a canned file, then accept a fresh USB command without resetting."""
import time

import pytest

pytestmark = [pytest.mark.motion, pytest.mark.ice, pytest.mark.resets_board,
              pytest.mark.persists, pytest.mark.dialect('d449'),
              pytest.mark.bug('N43', on=[])]


def wait_state(ice, predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = ice.peek_many(['xio.primary_src', 'xio.default_src', 'cm.hold_state',
                               'cm.motion_state', 'cm.machine_state'])
        if predicate(state):
            return state
        time.sleep(.05)
    pytest.fail(f'program stop state not reached: {state}')


@pytest.mark.parametrize('mode', [0, 1], ids=['char', 'line'])
@pytest.mark.parametrize('stop', [b'%', b'\x04'], ids=['flush', 'kill'])
def test_stopped_file_returns_to_usb(tg, ice, keep, record, mode, stop):
    keep('rxm')
    tg.set('rxm', mode)
    try:
        tg.send_json({'test': 51}, check=True, allow='canned braid, interrupted by feedhold and stop')
        started = wait_state(ice, lambda s: s['xio.primary_src'] != s['xio.default_src']
                             and s['cm.motion_state'] == 1)
        tg.send_raw(b'!')
        held = wait_state(ice, lambda s: s['cm.hold_state'] == 6)
        assert held['xio.primary_src'] != held['xio.default_src'], 'file already ended before stop'
        mark = len(tg.transport.history)
        tg.send_raw(stop)
        stopped = wait_state(ice, lambda s: s['xio.primary_src'] == s['xio.default_src'])
        record('program_stop', {'mode': mode, 'stop': stop.hex(), 'started': started,
                               'held': held, 'stopped': stopped})
        # A USB query must now be answered without waiting for file EOF or resetting.
        stat = tg.stat()
        if stop == b'\x04':
            assert stat == tg.dialect.stat('ALARM')
            tg.clear_alarm()
        else:
            tg.wait_idle()
        tg.send_gcode('M5', check=True)
        errors = [e.raw for e in tg.transport.history[mark:]
                  if e.kind == 'exception' and e.data.get('er', {}).get('st') == 13]
        assert not errors, errors
    finally:
        tg.reset()
