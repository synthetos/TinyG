"""N66/N40: timed holds on a long rapid retain its exact endpoint with accounting enabled.

Original Marlin file intentionally retains 2599 bare-I parse refusals: they determine sender/hold timing.
The regression checks runtime and emitted-step counters separately; it does not treat model position as motion.
"""
from pathlib import Path
import random
import threading
import pytest
from tinyg_hil import sender

pytestmark=[pytest.mark.motion,pytest.mark.ice,pytest.mark.persists,pytest.mark.resets_board,
            pytest.mark.dialect('d449'),pytest.mark.bug('N66'),pytest.mark.timeout(240)]


class Holds(threading.Thread):
    def __init__(self,tg):
        super().__init__(daemon=True);self.tg=tg;self.stop=threading.Event();self.sent=[]
    def run(self):
        r=random.Random(9359860)
        for _ in range(3):
            if self.stop.wait(r.uniform(2,8)):return
            self.tg.send_raw(b'!');self.sent.append('!')
            self.stop.wait(r.uniform(.4,1.5));self.tg.send_raw(b'~');self.sent.append('~')


def test_accounting_holds_return_runtime_and_steps_to_origin(tg,ice,keep,record):
    desired={'rxm':1,'ex':1,'jv':3,'qv':1,'sv':1,'si':250,'sl':0,'ja':100000,'ct':.01,
             '1ma':0,'2ma':0,'3ma':2,'1po':1,'2po':1,'3po':0}
    for m in (1,2,3):desired.update({f'{m}mi':8,f'{m}sa':1.8,f'{m}tr':1.25})
    for a in 'xyz':desired.update({a+'vm':800,a+'fr':800,a+'jm':20,a+'jd':.05,'g54'+a:0})
    keep(*desired)
    tg.reset()
    tg.wait_idle()
    for token,value in desired.items():tg.set(token,value,allow='unwired bench N66 replay geometry')
    tg.send_gcode('G21 G90 G17 G94 G92.1 G54 G49',check=True)
    tg.send_gcode('G28.3 X0 Y0 Z0 A0',check=True,allow='declare unwired bench origin')
    tg.wait_idle()
    names=[f'mr.position[{a}]' for a in range(3)]
    for m in (0,1):names += [f'en.en[{m}].encoder_steps',f'en.en[{m}].steps_run',f'mr.target_steps[{m}]']
    before=ice.peek_many(names)
    assert all(before[f'en.en[{m}].encoder_steps']+before[f'en.en[{m}].steps_run']==0 for m in (0,1)),before
    path=Path(__file__).resolve().parents[2]/'corpus/n66/lightburn-lasse.gcode.nc'
    lines=[line for line in path.read_text().splitlines() if line.strip()]
    for line in lines:tg.guard.check_gcode(line,allow='unwired bench original N66 rapid to X203.12 Y164.35')
    holder=Holds(tg)
    try:
        tg.send_line('{"rt":{"fi":0}}',check=True)
        tg.send_line('{"rt":{"fi":1}}',check=True)
        tg.send_line('{"rt":{"hb":1}}',check=True)
        holder.start()
        result=sender.stream(tg,lines,'line',timeout=150)
        holder.stop.set();holder.join(timeout=3);assert not holder.is_alive()
        tg.send_raw(b'~');tg.wait_idle(timeout=30)
        tg.send_gcode('M5 M9 G92.1 G90 G21 G94 G64',check=True)
        tg.send_gcode('G53 G0 X0 Y0 Z0 A0',check=True,allow='return unwired bench to replay origin')
        tg.wait_idle(timeout=30)
        after=ice.peek_many(names)
        record('n66_endpoint',{'before':before,'after':after,'controls':holder.sent,'acked':result.acked,
                               'error_count':len(result.errors),'seed':9359860})
        assert len(holder.sent)==6,holder.sent
        assert result.acked==len(lines) and len(result.errors)==2599
        assert all(error[1]==102 for error in result.errors)
        assert all(abs(after[f'mr.position[{a}]'])<1e-7 for a in range(3)),after
        for m in (0,1):
            delta=sum(after[f'en.en[{m}].{f}']-before[f'en.en[{m}].{f}'] for f in ('encoder_steps','steps_run'))
            assert abs(delta)<=1,(m,delta,after)
    finally:
        holder.stop.set()
        if holder.ident is not None:holder.join(timeout=3)
        tg.reset() # abort queued work before keep restores settings; no flush before measured endpoint
