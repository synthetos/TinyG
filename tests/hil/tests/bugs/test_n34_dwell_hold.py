"""A feedhold during a dwell lets its timer finish but stops the following move."""
import time
import pytest

pytestmark = [pytest.mark.motion, pytest.mark.ice, pytest.mark.persists,
              pytest.mark.resets_board, pytest.mark.dialect('d449')]


def _setup(tg, keep):
    keep('rxm', 'ex', 'xvm')
    tg.set('rxm', 1, allow='dwell hold control lane')
    tg.set('ex', 1, allow='dwell hold control lane')
    tg.set('xvm', 600, allow='bounded dwell hold moves')
    tg.send_gcode('G21 G90 G94 G92.1 G54 G49', check=True)
    tg.send_gcode('G28.3 X0', check=True, allow='declare X for bounded hold checks')


def _counts(ice):
    names = ['en.en[0].encoder_steps', 'en.en[0].steps_run', 'st_run.dda_ticks_downcount',
             'cm.hold_state', 'cm.motion_state', 'cm.cycle_state', 'st_pre.buffer_state',
             'st_pre.block_type', 'mb.buffers_available']
    v = ice.peek_many(names)
    return v, v[names[0]] + v[names[1]]


def _queue(tg, lines):
    for line in lines:
        tg.guard.check_gcode(line, allow='dwell and at most five millimetres of X travel')
    tg.transport.write(('\n'.join(lines) + '\n').encode())


def _hold(tg):
    tg.drain(quiet=0.05, limit=0.2)
    tg.send_raw(b'!')
    end=time.monotonic()+1.0
    while time.monotonic()<end:
        if tg.stat()==tg.dialect.stat('HOLD') and tg.get('hold')==tg.dialect.hold('HOLD'):
            return
        time.sleep(0.03)
    assert False, 'feedhold did not report stat 6 before the dwell ended'


def _cleanup(tg):
    tg.send_raw(b'\x04')
    time.sleep(0.2)
    tg.drain(quiet=0.05, limit=0.2)
    tg.clear_alarm()
    tg.send_gcode('G90 G0 X0', allow='return after bounded dwell test')
    tg.wait_idle(timeout=10)
    tg.reset()
    tg.send_json({'fb': None})  # reset can restore persisted text mode


@pytest.mark.bug('N34', on=[])
@pytest.mark.parametrize('prefix,dwell', [([],4), (['G0 X2'],3)], ids=['opening','between-moves'])
def test_dwell_hold_blocks_following_move(tg, keep, ice, record, prefix, dwell):
    _setup(tg, keep)
    try:
        _queue(tg, prefix+[f'G4 P{dwell}', 'G0 X5'])
        time.sleep(1.0)
        before, steps = _counts(ice)
        _hold(tg)
        time.sleep(dwell+0.4)  # timer must finish, with no following X pulses
        after, held_steps = _counts(ice)
        record('dwell_hold', {'before':before, 'after':after, 'step_delta':held_steps-steps})
        assert held_steps==steps, 'queued move stepped after dwell while held'
        assert tg.stat()==tg.dialect.stat('HOLD')
        tg.send_raw(b'~')
        tg.wait_idle(timeout=10)
        assert abs(tg.get('mpox')-5)<0.002
    finally:
        _cleanup(tg)


@pytest.mark.bug('N34', on=[])
@pytest.mark.parametrize('control', [b'~',b'%'], ids=['resume','flush'])
def test_lone_dwell_hold_ends_cleanly(tg, keep, ice, record, control):
    _setup(tg, keep)
    try:
        _queue(tg,['G4 P3'])
        time.sleep(1.0)
        _hold(tg)
        if control==b'%':
            tg.send_raw(control)  # flush waits for the active dwell timer
        time.sleep(2.6)
        if control==b'~':
            assert tg.stat()==tg.dialect.stat('HOLD')
            tg.send_raw(control)
        tg.wait_idle(timeout=5)
        state,_=_counts(ice)
        record('lone_dwell',state)
        assert state['cm.hold_state']==0
        assert state['st_run.dda_ticks_downcount']==0
        assert state['mb.buffers_available']==tg.planner_buffers
        assert state['st_pre.buffer_state']==1  # PREP_BUFFER_OWNED_BY_EXEC
        tg.send_gcode('G0 X1',check=True,allow='verify no stale dwell/hold blocks the next job')
        tg.wait_idle(timeout=5)
        assert abs(tg.get('mpox')-1)<0.002
    finally:
        _cleanup(tg)


def test_idle_feedhold_does_not_block_next_job(tg, keep):
    _setup(tg, keep)
    tg.wait_idle(timeout=5)
    try:
        tg.send_raw(b'!')
        time.sleep(0.1)
        assert tg.get('hold')==0
        tg.send_gcode('G0 X1',check=True,allow='idle ! must leave the next job runnable')
        tg.wait_idle(timeout=5)
        assert abs(tg.get('mpox')-1)<0.002
    finally:
        _cleanup(tg)
