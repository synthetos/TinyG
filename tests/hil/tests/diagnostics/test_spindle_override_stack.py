"""Observe combined-image stack depth during live spindle updates and queued permissions."""
import time
import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice, pytest.mark.motion,
              pytest.mark.persists, pytest.mark.resets_board,
              pytest.mark.dialect('d449'), pytest.mark.timeout(180)]


def test_spindle_override_stack(tg, ice, keep, record):
    try:
        tg.reset()
        keep('sso', 'ssoe', 'm48e', 'p1csl', 'p1csh', 'p1wsl', 'p1wsh')
        for key, value in dict(ssoe=1, m48e=1, sso=1, p1csl=0, p1csh=10000,
                               p1wsl=0, p1wsh=10000).items():
            tg.set(key, value, allow='known spindle range for stack measurement')
        tg.send_gcode('G21 G91 G94 G64 M48', check=True)
        start, end = ice.stack_paint()
        record('layout', {'bss_end': hex(start), 'painted_bytes': end-start})
        for direction, distance in (('M3', 20), ('M4', -20)):
            tg.send_gcode(f'{direction} S6000', check=True, allow='spindle stack workload')
            tg.send_gcode(f'G1 Y{distance} F600', check=True, allow='concurrent spindle update workload')
            for factor in (.05, .5, 1, 2):
                tg.set('sso', factor)
                time.sleep(.1)
                for token in ('sso', 'ssc', 'sps', 'sse', 'scp'):
                    tg.get(token)
            tg.send_gcode('M49', check=True)
            tg.send_gcode('S3000', check=True, allow='queued spindle speed change')
            tg.send_gcode('M51 P1', check=True)
            tg.wait_idle(timeout=30)
            tg.send_gcode('M48', check=True)
        tg.send_gcode('M5', check=True, allow='spindle off after stack workload')
        tg.wait_idle(timeout=30)
        tg.set('sso', .5)
        time.sleep(.1)
        assert tg.get('sps') == 0
        low, free = ice.stack_highwater(start, end)
        floor = 1024 if 'test_hook' in ice.symbols.sym else 1536
        record('spindle_override_highwater', {'lowest_stack_address': hex(low),
               'min_free_bytes': free, 'max_stack_bytes': 0x5fff-low+1, 'required_floor': floor,
               'scope': 'live CW/CCW scaling and clamps during motion, queued permissions, reports and M5'})
        assert free >= floor, f'spindle workload leaves {free} bytes; requires {floor}'
    finally:
        tg.reset()
