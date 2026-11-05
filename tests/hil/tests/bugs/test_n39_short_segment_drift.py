"""N39: compare virtual counted steps with runtime after a dense, closed excerpt.

ICE needs the exact ELF. Virtual counters verify commanded pulses, not electrical
step edges; Saleae remains the independent physical check. No ICE halt occurs
while the excerpt streams. Stock failure must be confirmed by the bench gate.
"""
from pathlib import Path
import pytest
from tinyg_hil.sender import stream
pytestmark=[pytest.mark.motion,pytest.mark.ice,pytest.mark.persists,
            pytest.mark.dialect('d449'),pytest.mark.bug('N39',on=[])]

def test_dense_excerpt_step_endpoint(tg,ice,keep,record):
    tokens=[axis+suffix for axis in 'xyz' for suffix in ['vm','fr','jm']]
    keep('rxm','ex',*tokens)
    for axis in 'xyz':
        for suffix,value in [('vm',1200),('fr',1200),('jm',500)]:
            tg.set(axis+suffix,value,allow='N39 bounded dense-file motion profile')
    tg.set('rxm',1,allow='N39 g2 streaming')
    tg.set('ex',1,allow='N39 g2 streaming')
    tg.send_gcode('G21 G90 G17 G94 G92.1 G54',check=True)
    tg.send_gcode('G28.3 X0 Y0 Z0 A0',check=True,allow='N39 seed unwired bench origin')
    tg.wait_idle()
    motors=[m for m in range(4) if tg.get(f'{m+1}ma')==0]
    assert motors,'requires a motor mapped to X'
    spu={m:360/tg.get(f'{m+1}sa')*tg.get(f'{m+1}mi')/tg.get(f'{m+1}tr') for m in motors}
    def snapshot():
        names=['mr.position[0]']
        for m in motors:names += [f'en.en[{m}].encoder_steps',f'en.en[{m}].steps_run']
        return ice.peek_many(names)
    before=snapshot()
    path=Path(__file__).resolve().parents[2]/'corpus/n39/dense-isolation-excerpt.nc'
    lines=path.read_text().splitlines()
    for line in lines:tg.guard.check_gcode(line,allow='N39 bounded isolation excerpt on unwired bench')
    result=stream(tg,lines,'g2',timeout=120)
    assert result.ok,result
    after=snapshot();record('step_endpoint',{'before':before,'after':after,'steps_per_unit':spu})
    assert abs(after['mr.position[0]'])<.00001,'runtime missed the commanded endpoint'
    for m in motors:
        actual=sum(after[f'en.en[{m}].{field}']-before[f'en.en[{m}].{field}']
                   for field in ['encoder_steps','steps_run'])
        expected=(after['mr.position[0]']-before['mr.position[0]'])*spu[m]
        assert abs(actual-expected)<=1.1,(m,actual,expected)
