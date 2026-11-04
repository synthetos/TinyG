"""Discarding a long held dwell must release the runtime and host input promptly."""
import time
import pytest

pytestmark = [pytest.mark.motion, pytest.mark.ice, pytest.mark.resets_board,
              pytest.mark.dialect('d449')]


def _state(ice):
    keys=['en.en[0].encoder_steps','en.en[0].steps_run','st_run.dda_ticks_downcount',
          'cm.hold_state','cm.cycle_state','cm.queue_flush_state','mb.buffers_available',
          'st_pre.buffer_state','st_pre.block_type']
    return ice.peek_many(keys)


@pytest.mark.bug('N51', on=[])
@pytest.mark.parametrize('control',[b'%', b'\x04', b'%\x04'], ids=['flush','kill','flush-then-kill'])
def test_long_dwell_abort_stays_responsive(tg, ice, record, control):
    tg.send_gcode('G21 G90 G94 G92.1 G54 G49',check=True)
    before=_state(ice)
    try:
        tg.send_json({'gc':'G4 P30'},check=True,allow='stationary bounded dwell abort regression')
        time.sleep(1.0)
        tg.send_raw(b'!')
        assert tg.get('stat',timeout=1)==tg.dialect.stat('HOLD')
        time.sleep(0.8)
        start=time.monotonic()
        tg.send_raw(control[:1])
        if len(control)>1:
            time.sleep(0.1)
            tg.send_raw(control[1:])
        response=tg.send_json({'stat':None},timeout=max(0.01,1-(time.monotonic()-start)))
        assert 'stat' in response.body, response
        answered=time.monotonic()-start
        expected=tg.dialect.stats('ALARM') if b'\x04' in control else tg.dialect.idle_stats
        deadline=start+(1 if b'\x04' in control else 2)
        while True:
            state=_state(ice)
            stat=tg.get('stat',timeout=max(0.01,deadline-time.monotonic()))
            if stat in expected and state['mb.buffers_available']==tg.planner_buffers and not state['st_run.dda_ticks_downcount']:
                break
            assert time.monotonic()<deadline,(stat,state)
            time.sleep(0.02)
        record('dwell_abort',{'reply_seconds':answered,'settled_seconds':time.monotonic()-start,
                              'stat':stat,'state':state})
        assert state['cm.hold_state']==0
        assert state['cm.queue_flush_state']==0
        if b'\x04' not in control:
            assert state['cm.cycle_state']==0
        assert state['st_pre.buffer_state']==1
        assert state['st_pre.block_type']==0
        assert state['en.en[0].encoder_steps']+state['en.en[0].steps_run']==before['en.en[0].encoder_steps']+before['en.en[0].steps_run']
    finally:
        tg.reset()  # bounded even on the deaf stock firmware; no remaining 30-second wait
