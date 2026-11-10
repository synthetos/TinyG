"""N12 residual: a short block must retain its configured motor pulse-rate limit.

Found while investigating N78; this reduced valid chain needs no feedhold to
panic on stock. It is not evidence that the original N78 report is reproduced.
ICE counts commanded pulses; external capture remains the physical check.
"""
from pathlib import Path

import pytest
from tinyg_hil.sender import stream

pytestmark = [pytest.mark.motion, pytest.mark.ice, pytest.mark.persists,
              pytest.mark.resets_board, pytest.mark.dialect('d449'),
              pytest.mark.bug('N12', on=[])]


def test_short_block_cap_prevents_dda_panic(tg, ice, keep, record):
    keep('rxm', 'ex', *[a + s for a in 'xy' for s in ('vm', 'fr', 'jm')],
         *[f'{m}{s}' for m in range(1, 5) for s in ('ma', 'sa', 'mi', 'tr')])
    try:
        for m in range(1, 5):
            for suffix, value in (('ma', m - 1), ('sa', 1.8), ('mi', 8),
                                  ('tr', 1.25 if m <= 3 else 360)):
                tg.set(f'{m}{suffix}', value, allow='N12 short-block pulse-cap profile')
        for axis in 'xy':
            for suffix, value in (('vm', 20000), ('fr', 20000), ('jm', 5000)):
                tg.set(axis + suffix, value, allow='N12 planner must enforce its 50 kHz DDA capacity')
        tg.set('rxm', 1)
        tg.set('ex', 1)
        tg.send_gcode('G21 G90 G17 G94 G92.1 G54', check=True)
        tg.send_gcode('G28.3 X0 Y0 Z0 A0', check=True, allow='N12 seed unwired bench origin')
        tg.wait_idle()
        names = [f'en.en[{m}].{field}' for m in range(2) for field in ('encoder_steps', 'steps_run')]
        before = ice.peek_many(names)
        program = Path(__file__).resolve().parents[2] / 'corpus/n12/short-block-pulse-cap.nc'
        lines = program.read_text().splitlines()
        for line in lines:
            tg.guard.check_gcode(line, allow='N12 reduced CAM chain inside a 2 mm box')
        result = stream(tg, lines, 'g2', timeout=20)
        assert result.ok, result
        after = ice.peek_many(names + ['mr.position[0]', 'mr.position[1]'])
        record('short_block_cap', {'before': before, 'after': after, 'stream': str(result)})
        assert tg.stat() not in tg.dialect.stats('ALARM', 'SHUTDOWN', 'PANIC')
        for motor, end in enumerate((1.9951, 1.8282)):
            actual = sum(after[f'en.en[{motor}].{field}'] - before[f'en.en[{motor}].{field}']
                         for field in ('encoder_steps', 'steps_run'))
            assert after[f'mr.position[{motor}]'] == pytest.approx(end, abs=0.001)
            assert abs(actual - end * 1280) <= 1.5, (motor, actual, end * 1280)
    finally:
        tg.reset()  # A stock PANIC must be reset before the saved settings can be restored.
