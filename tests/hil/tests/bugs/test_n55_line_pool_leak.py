"""Priority CTRL reads must return their RX credits after older DATA drains."""
import time
import pytest
from tinyg_hil import NoResponse, sender

pytestmark=[pytest.mark.dialect('d449'),pytest.mark.resets_board,pytest.mark.persists,
            pytest.mark.motion]


def _setup(tg,keep):
    keep('rxm','ex','qv','sv','jv')
    tg.reset()
    for token,value in {'rxm':1,'ex':1,'qv':0,'sv':0,'jv':3}.items():
        if tg.get(token)!=value:
            tg.set(token,value,allow='line-pool ownership regression')
    tg.send_gcode('M2',check=True)
    tg.wait_idle(timeout=5)
    assert tg.get('rx')==10


def _overtake(tg,control):
    # One running dwell and a full planner keep DATA queued while CTRL is read.
    started=time.monotonic()
    tg.send_gcode('G4 P1.5',check=True)
    for _ in range(40):
        try:
            tg.send_gcode('G4 P0.001',timeout=0.12,check=True)
        except NoResponse:
            break
    else:
        pytest.fail('planner did not reach its headroom gate')
    # An older DATA row and a newer filling/data row bracket this priority CTRL.
    tg.transport.write((control+'\nG4 P0.001\n').encode())
    end=time.monotonic()+1
    while True:
        ev=tg.transport.next_event(end-time.monotonic())
        assert ev is not None,'priority CTRL did not answer while DATA was blocked'
        if control.startswith('$'):
            if ev.kind=='prompt':break
        elif ev.kind=='response' and any(k in ev.body for k in ('stat','sr','rt','qr')):
            assert ev.data['f'][1]==0,ev.raw
            break
    # Avoid generating extra priority queries until the queued DATA has drained.
    time.sleep(max(0,started+2.5-time.monotonic()))
    tg.wait_idle(timeout=5)
    return tg.get('rx')


@pytest.mark.bug('N55',on=[])
@pytest.mark.parametrize('control',[
    '{"stat":null}', '{"sr":null}',
    '{"rt":{"up":null,"hb":null,"fi":null}}',
    '{"rt":{"hb":0}}', '$HZ',
],ids=['plain-json-get','status-get','runtime-get','runtime-set','text-error'])
def test_priority_line_returns_its_credit(tg,keep,record,control):
    _setup(tg,keep)
    try:
        after=_overtake(tg,control)
        record('line_pool',{'control':control,'before':10,'after':after})
        assert after==10,'reclaimed priority line did not return its RX credit'
    finally:
        tg.reset()


@pytest.mark.bug('N55',on=[])
def test_repeated_overtakes_leave_credit_for_next_stream(tg,keep,record):
    _setup(tg,keep)
    try:
        counts=[10]
        for _ in range(10):
            counts.append(_overtake(tg,'{"stat":null}'))
        record('line_pool_exhaustion',counts)
        assert counts==[10]*11,'idle RX credits leaked toward zero across jobs'
        result=sender.stream(tg,['G4 P0.001'],'line',timeout=5)
        record('next_stream',{'acked':result.acked})
        assert result.acked==1
        tg.wait_idle(timeout=5)
    finally:
        tg.reset()
