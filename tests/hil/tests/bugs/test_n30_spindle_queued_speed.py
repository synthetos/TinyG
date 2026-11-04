"""N30: an S before an already-parsed M30 must keep the driven spindle mode.

Two dwells make queue ordering observable without relying on a motor trajectory.
spc is the public getter of spindle_out.mode; sps reports the clamped output speed.
"""
import time
import pytest
pytestmark=[pytest.mark.dialect('d449'),pytest.mark.motion,pytest.mark.persists,
            pytest.mark.bug('N30',on=[])]

def test_queued_speed_before_m30_uses_driven_mode(tg,keep):
    keep('p1csl','p1csh')
    tg.set('p1csl',0,allow='N30 restore spindle speed range afterward')
    tg.set('p1csh',10000,allow='N30 restore spindle speed range afterward')
    try:
        tg.send_gcode('M3 S1000',check=True,allow='N30 bench spindle output test')
        assert tg.get('spc')==1 and tg.get('sps')==pytest.approx(1000)
        for line in ('G4 P1','S5000','G4 P3','M30'):
            tg.guard.check_gcode(line,allow='N30 bounded dwells and queued speed change')
            assert tg.send_line(line,timeout=5).status==0
        deadline=time.monotonic()+2.5
        speed=1000
        while time.monotonic()<deadline and speed==1000:
            speed=tg.get('sps')
            if speed==1000:time.sleep(.02)
        assert tg.get('spc')==1,'M30 must still be behind the second dwell'
        assert speed==pytest.approx(5000),'S used the reset model instead of the driven spindle mode'
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and tg.get('spc')!=0:time.sleep(.05)
        assert tg.get('spc')==0 and tg.get('sps')==0
    finally:
        tg.send_gcode('M5',check=True,allow='N30 turn spindle off')
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and tg.get('spc')!=0:time.sleep(.05)
