"""Paint the stack across live queued/BODY feed replanning on an unloaded bench."""
import time
import pytest

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice, pytest.mark.motion,
              pytest.mark.persists, pytest.mark.resets_board,
              pytest.mark.dialect('d449'), pytest.mark.timeout(180)]


def test_override_stack_highwater(tg, ice, keep, record):
    try:
        tg.reset()
        keep('mfo', 'mfoe', 'm48e')
        tg.set('mfoe', 1)
        tg.set('m48e', 1)
        tg.set('mfo', 1)
        tg.send_gcode('G21 G91 G94 G64 M48', check=True)
        start, end = ice.stack_paint()
        record('layout', {'bss_end': hex(start), 'painted_bytes': end-start})
        for distance, factor in ((20, .5), (-20, 1.5)):
            for _ in range(4):
                tg.send_gcode(f'G1 Y{distance} F600', check=True,
                              allow='queued live override stack workload')
            time.sleep(1)
            tg.set('mfo', factor)
            tg.wait_idle(timeout=60)
        # Exercise the bounded partial-factor attempts on an already-filled short queue.
        tg.set('mfo', 1)
        for _ in range(28):
            tg.send_gcode('G1 Y0.4 F600', check=True, allow='short queued override stack workload')
        tg.set('mfo', .5)
        tg.wait_idle(timeout=30)
        tg.send_gcode('G0 Y-11.2', check=True, allow='return after short queue workload')
        tg.wait_idle(timeout=30)
        low, free = ice.stack_highwater(start, end)
        floor = 1024 if 'test_hook' in ice.symbols.sym else 1536
        record('override_highwater', {'lowest_stack_address': hex(low), 'min_free_bytes': free,
                                     'max_stack_bytes': 0x5fff-low+1, 'required_floor': floor,
                                     'scope': 'observed live decrease/increase and partial queue planning'})
        assert free >= floor, f'override workload leaves {free} bytes; requires {floor}'
    finally:
        tg.reset()
