"""Measure the queue publication bracket with MOTION_PROFILE=8 and its ELF.

Timer setup and accounting are outside the interrupt mask. Counter clear and
stop are inside it, excluding pending DDA interrupts at guard exit. Add 1 us
(32 CPU cycles) for the unmeasured edge instructions and timer quantization.
Site 5 remains the separate running BODY claim measurement.
"""
import time
import pytest
from tinyg_hil.planner_profile import FIELDS

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice, pytest.mark.motion,
              pytest.mark.persists, pytest.mark.resets_board,
              pytest.mark.dialect('d449'), pytest.mark.timeout(180)]


def test_queue_publication(tg, ice, keep, record):
    assert 'motion_profile' in ice.symbols.sym, 'use MOTION_PROFILE=8 and matching ELF'
    assert 'test_hook' not in ice.symbols.sym
    try:
        tg.reset()
        keep('mfo', 'mfoe', 'm48e')
        tg.set('mfoe', 1)
        tg.set('m48e', 1)
        tg.set('mfo', 1)
        tg.send_gcode('G21 G91 G94 G64 M48', check=True)
        for _ in range(4):
            tg.send_gcode('G1 Y20 F600', check=True, allow='queued override publication profile')
        time.sleep(1)
        tg.set('mfo', .5)
        tg.wait_idle(timeout=90)
        values = ice.peek_many(['motion_profile.'+name for name in FIELDS])
        row = {name: int(values['motion_profile.'+name]) for name in FIELDS}
        row.update(tick_us=.25, measured_us=.25*row['maximum_ticks'],
                   edge_allowance_us=1., maximum_us=.25*row['maximum_ticks']+1.,
                   includes='guard body plus conservative 32-cycle edge/quantization allowance',
                   excludes='timer setup/accounting and unmasked ISR preemption; observed workload only')
        record('queue_publication_timer', row)
        assert row['site'] == 8 and row['calls'] > 0, row
        assert not row['overflows'] and row['maximum_ticks'] < 65535, row
        assert row['section_at_maximum'] == 1, 'paired publication must be exercised'
        assert row['maximum_us'] < 20, row
    finally:
        tg.reset()
