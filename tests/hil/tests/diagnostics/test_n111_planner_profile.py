"""Operator-run AVR planning-cost baseline; no claim of suffix-producer timing.

Flash MOTION_PROFILE=6 or 7 and pass that image's ELF. Site 6 times a lookahead
pass; site 7 times one trapezoid. Full fixture 21 is opt-in because it is long.
ICE reads occur only after motion has stopped. No MSO is required.
"""
import os
from pathlib import Path
import time
import pytest
from tinyg_hil import sender
from tinyg_hil.planner_profile import FIELDS, decode_sample

pytestmark = [pytest.mark.diagnostic, pytest.mark.ice, pytest.mark.motion,
              pytest.mark.persists, pytest.mark.resets_board,
              pytest.mark.dialect('d449'), pytest.mark.timeout(4000)]
FIXTURES = Path(__file__).resolve().parents[2] / 'fixtures' / 'planner_timing'


@pytest.mark.parametrize('fixture', ['05_tiny_collinear.nc', '21_dense_cam_excerpt.nc'])
def test_planning_service(tg, ice, keep, record, fixture):
    if fixture.startswith('21') and os.environ.get('N111_PROFILE_DENSE') != '1':
        pytest.skip('full dense fixture: enable N111_PROFILE_DENSE=1')
    assert 'motion_profile' in ice.symbols.sym, 'use a profiling image and matching ELF'
    assert 'test_hook' not in ice.symbols.sym, 'profiling and hooks are separate images'
    lines = (FIXTURES / fixture).read_text().splitlines()
    for line in lines:
        tg.guard.check_gcode(line, allow='planner service profile on unloaded bench')
    try:
        tg.reset()
        keep('ej', 'rxm', 'ex', 'qv', 'sv', 'si')
        for key,value in dict(ej=1,rxm=1,ex=1,qv=1,sv=1,si=100).items():
            tg.set(key,value,allow='planner profiling line stream')
        tg.set('mfo',1)
        start=time.monotonic()
        result=sender.stream(tg,lines,'line',timeout=3600)
        record('stream',dict(fixture=fixture,ok=result.ok,elapsed_s=time.monotonic()-start,
                            content_lines=len(lines)))
        assert result.ok, result
        tg.wait_idle(timeout=60)
        values=ice.peek_many(['motion_profile.'+name for name in FIELDS])
        record('raw_planning_timer',values)
        row=decode_sample(values)
        record('planning_timer',row)
        record('limits',dict(
            includes='higher-priority interrupt preemption; queue-count argument evaluation',
            excludes='timer setup/accounting and wrapper entry/exit',
            coverage='observed maximum for this fixture/settings, not all legal inputs',
            producer='baseline existing planner, not the future suffix producer'))
    finally:
        tg.reset()
