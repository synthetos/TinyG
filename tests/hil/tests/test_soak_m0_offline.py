"""Program-stop automation never turns an ordinary stall or feedhold into success."""
import json
import time
from types import SimpleNamespace

import pytest
from tinyg_hil import soak,sender
from tinyg_hil.dialect import load
from tinyg_hil.protocol import parse_line
from test_sender_offline import FakeTG,FakeTransport,board,ok,no_real_power_assertion


@pytest.mark.parametrize('name',['d440','d449'])
def test_stop_settles_records_and_ignores_old_or_nonstop_reports(monkeypatch,name):
    now=[10.0];writes=[];history=[]
    def sleep(seconds):now[0]+=seconds
    monkeypatch.setattr(soak,'time',SimpleNamespace(monotonic=lambda:now[0],sleep=sleep))
    tg=SimpleNamespace(dialect=load(name),transport=SimpleNamespace(history=history,write=lambda b:writes.append((now[0],b))))
    control=soak.ProgramStopResumer(tg)
    assert not control(9,{'stat':3})
    for invalid in (None,True,3.0,'3'):
        assert not control(11,{'stat':invalid})
    for state in tg.dialect.stat_names:
        if state!='PROGRAM_STOP':assert not control(11,{'stat':tg.dialect.stat(state)})
    now[0]=11
    assert control(11,{'stat':3})
    assert writes==[(11.5,b'~')] and len(control.events)==1
    assert not control(11.4,{'stat':3})
    now[0]=12
    assert control(12,{'stat':3})  # a fresh adjacent M0 may have no intervening RUN frame
    assert len(control.events)==2


@pytest.mark.parametrize('name',['d440','d449'])
def test_feedhold_during_settle_cancels_cycle_start(monkeypatch,name):
    now=[10.0];writes=[];history=[]
    def sleep(seconds):
        now[0]+=seconds
        history.append(parse_line('{"sr":{"stat":6}}',now[0]))
    monkeypatch.setattr(soak,'time',SimpleNamespace(monotonic=lambda:now[0],sleep=sleep))
    tg=SimpleNamespace(dialect=load(name),transport=SimpleNamespace(history=history,write=writes.append))
    control=soak.ProgramStopResumer(tg);now[0]=11
    assert not control(11,{'stat':3})
    assert not writes and not control.events


def test_modes_and_option_validation():
    assert soak.mode_for(load('d440'))=='count'
    assert soak.mode_for(load('d449'))=='line'
    for mode in ('count','count-crlf','rts','xon','xon-app'):
        assert soak.mode_for(load('d440'),mode)==mode
    with pytest.raises(ValueError,match='no line mode'):soak.mode_for(load('d440'),'line')
    assert not soak.resume_policy('') and soak.resume_policy('resume')
    with pytest.raises(ValueError):soak.resume_policy('yes')


class HistoryTransport(FakeTransport):
    def __init__(self,board):super().__init__(board);self.history=[]
    def emit(self,raw):
        event=parse_line(raw,time.monotonic());self.history.append(event);self.events.put(event)


@pytest.mark.parametrize('name,mode',[('d440','count'),('d449','line')])
def test_real_sender_resumes_program_stop_and_preserves_all_acks(name,mode):
    pending=[];paused=[False]
    rev=1 if name=='d440' else 3
    def answer(tr,data):
        if data==b'~':
            paused[0]=False
            tr.emit('{"sr":{"stat":5}}')
            queued=list(pending);pending.clear()
            for line in queued:answer(tr,line)
        elif data.startswith(b'{"stat"'):
            tr.emit(json.dumps({'r':{'stat':3 if paused[0] else 4},'f':[rev,0,10]}))
        elif paused[0]:pending.append(data)
        else:
            tr.emit(json.dumps({'r':{},'f':[rev,0,10 if mode=='line' else len(data)]}))
            if data.strip() in (b'M0',b'M1'):
                paused[0]=True;tr.emit('{"sr":{"stat":3}}')
    tg=FakeTG(answer);tg.transport=HistoryTransport(answer);tg.dialect=load(name)
    control=soak.ProgramStopResumer(tg,settle=.001)
    result=sender.stream(tg,['G1 X1','M0','G1 X2','M1','G1 X3'],mode,timeout=2,on_status=control)
    assert result.ok and result.acked==5 and len(control.events)==2


@pytest.mark.parametrize('stat',[5,6])
def test_non_program_stop_still_times_out_without_resume(stat):
    tg=FakeTG(board(stat=stat));tg.transport=HistoryTransport(board(stat=stat));tg.dialect=load('d449')
    control=soak.ProgramStopResumer(tg,settle=.001)
    result=sender.stream(tg,['G1 X1'],'count',timeout=.04,on_status=control)
    assert result.completion=='timeout' and not result.ok
    assert not control.events and all(data!=b'~' for data,_ in tg.transport.writes)


def test_final_drain_handles_stops_after_all_line_replies(monkeypatch):
    import importlib.util
    from pathlib import Path
    spec=importlib.util.spec_from_file_location('soak_diagnostic',Path(__file__).parent/'diagnostics/test_soak_file.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    writes=[];states=iter([3,3,4]);queues=iter([10,10,32])
    tg=SimpleNamespace(dialect=load('d440'),stat=lambda:next(states),get=lambda _:next(queues),
        transport=SimpleNamespace(history=[],write=writes.append))
    control=soak.ProgramStopResumer(tg,settle=.001)
    assert module._settle(tg,2,control)==4
    assert writes==[b'~',b'~'] and len(control.events)==2


def test_ack_stall_is_not_reset_by_other_status_reports(monkeypatch):
    def answer(tr,data):
        if data.startswith(b'{"stat"'):
            tr.emit('{"r":{"stat":5},"tid":'+str(sender.PROBE_TID)+' ,"f":[3,0,10]}')
        else:tr.emit('{"sr":{"stat":5}}')
    tg=FakeTG(answer);tg.transport=HistoryTransport(answer);tg.dialect=load('d449')
    control=soak.ProgramStopResumer(tg,settle=.001)
    monkeypatch.setattr(sender,'STALL_HARD',.01)
    with pytest.raises(sender.StreamStalled,match='no line acknowledged'):
        sender.stream(tg,['G1 X1'],'line',timeout=.5,on_status=control)
    assert not control.events and all(data!=b'~' for data,_ in tg.transport.writes)
