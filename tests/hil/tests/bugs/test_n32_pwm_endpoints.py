"""N32: inspect PWM endpoints and a full-width timer period through the ICE.

Use an unwired spindle bench. Register observations establish configured output
state; a scope is needed to verify the electrical waveform and transition glitches.
"""
import time
import pytest
pytestmark=[pytest.mark.ice,pytest.mark.motion,pytest.mark.persists,
            pytest.mark.dialect('d449'),pytest.mark.bug('N32',on=[])]
TCD1=0x0940
CTRLB,PER,CCB=TCD1+1,TCD1+0x26,TCD1+0x2a
PORTD_OUT=0x0664

def test_zero_and_full_duty(tg,ice,keep,record):
    keep('p1pof','p1csl','p1csh','p1cpl','p1cph')
    for token,value in [('p1pof',0),('p1csl',0),('p1csh',10000),('p1cpl',0),('p1cph',1)]:
        tg.set(token,value,allow='N32 temporary spindle PWM configuration')
    original=None
    def sample():
        with ice.halted():
            return {'ctrlb':ice.read(CTRLB,1)[0],
                    'per':int.from_bytes(ice.read(PER,2),'little'),
                    'ccb':int.from_bytes(ice.read(CCB,2),'little'),
                    'out':ice.read(PORTD_OUT,1)[0]}
    def command(line):
        tg.send_gcode(line,check=True,allow='N32 unwired spindle output test')
        time.sleep(.2)
    try:
        command('M5')
        off=sample();record('zero',off)
        assert off['ccb']==0 and not(off['ctrlb']&0x20) and not(off['out']&0x20)
        with ice.halted():original=ice.read(PER,2)
        with ice.halted():ice.write(PER,b'\xff\xff')
        command('M3 S10000')
        full=sample();record('full',full)
        assert full['per']==65535 and full['ccb']==65535,'full-duty compare wrapped'
        assert not(full['ctrlb']&0x20) and full['out']&0x20
        command('S5000')
        middle=sample();record('middle',middle)
        assert middle['ctrlb']&0x20 and middle['ccb']==32768
    finally:
        command('M5')
        if original is not None:
            with ice.halted():ice.write(PER,original)
