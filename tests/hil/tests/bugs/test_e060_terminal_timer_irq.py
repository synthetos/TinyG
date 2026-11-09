"""Force a second DDA overflow at the final hold tick; '%' must still finish and answer."""
import time
import pytest

pytestmark = [pytest.mark.ice, pytest.mark.motion, pytest.mark.persists,
              pytest.mark.resets_board, pytest.mark.dialect('d449'), pytest.mark.timeout(45)]


@pytest.mark.bug('E060', on=['d449'])
def test_alarm_flush_after_a_late_terminal_irq(tg, ice, keep, record):
    try:
        ice.symbols.resolve('test_timer.dda_finish_delay')
    except KeyError:
        pytest.skip('requires the terminal-overflow test hook')
    for token in ('xsn', 'xsx', 'ysn', 'ysx', 'zsx', 'asn', 'asx'):
        if tg.get(token) == 4:
            keep(token)
            tg.set(token, 0)
    keep('zsn')
    tg.set('zsn', 4)
    tg.send_gcode('G21 G90 G94 G92 X0 Y0 Z0', check=True)
    tg.wait_idle()
    with ice.halted():
        ice.poke('test_timer.dda_finish_delays', 0)
        ice.poke('test_timer.dda_finish_delay', 1)
    try:
        tg.send_gcode('G1 X5 F100', check=True, allow='bounded move for terminal hold regression')
        tg.send_gcode('G38.2 Z-1 F60', check=True, allow='queued probe must be discarded by alarm')
        time.sleep(0.5)
        tg.send_json({'alarm': None}, check=True)
        time.sleep(1.0)
        tg.send_raw(b'%\n')
        time.sleep(0.2)
        state = ice.peek_many(['test_timer.dda_finish_delay', 'test_timer.dda_finish_delays',
                               'test_timer.dda_finish_pre_type', 'test_timer.dda_finish_flags',
                               'test_timer.dda_finish_ctrl', 'test_timer.dda_finish_cnt',
                               'test_timer.dda_finish_per', 'test_timer.dda_finish_flags_after_stop',
                               'st_run.dda_ticks_downcount', 'cm.hold_state',
                               'cm.queue_flush_state', 'cm.queue_flush_input', 'mb.buffers_available'])
        record('terminal_overflow', state)
        qualified = (state['test_timer.dda_finish_delays'] == 1 and
                     state['test_timer.dda_finish_pre_type'] == 0 and
                     state['test_timer.dda_finish_flags'] & 1 and
                     state['test_timer.dda_finish_ctrl'] != 0)
        if not qualified:
            pytest.skip(f'terminal overflow stimulus not qualified; not bug evidence: {state}')
        assert state['st_run.dda_ticks_downcount'] != 0xFFFFFFFF, state
        assert state['st_run.dda_ticks_downcount'] == 0, state
        tg.send_json({'clear': None}, check=True, timeout=1)
        tg.wait_idle(timeout=3)
        assert tg.get('qr') == tg.planner_buffers
    finally:
        ice.poke('test_timer.dda_finish_delay', 0)
        tg.reset()  # bounded even on stock: clear is intentionally blocked behind the stuck flush
