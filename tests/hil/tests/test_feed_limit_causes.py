"""Missing advisor causes, tied to timing fixtures 13,05,15,19.

The pending-input case uses bounded line-mode control/data bursts. It must
observe pending time rather than pass on a no-input gap alone. Bench qualification
is required: a board too fast to retain a backlog will fail with the raw evidence.
"""
import time
from pathlib import Path
import pytest
from tinyg_hil import sender
from tinyg_hil.feed_causes import check_cause,check_command,check_gaps,check_accounted_cause,exact_stop_lines
from test_feed_advisor_step3 import rt,since,settle

pytestmark=[pytest.mark.motion,pytest.mark.persists,pytest.mark.dialect('d449'),pytest.mark.timeout(120)]
FIXTURES=Path(__file__).resolve().parents[1]/'fixtures/planner_timing'


@pytest.fixture
def cause_window(tg,keep):
    keep('ej','rxm','ex','qv','sv','si','xfr','xvm','xjm','yfr','yvm','yjm')
    for k,v in dict(ej=1,rxm=1,ex=1,qv=1,sv=1,si=100,xfr=6000,xvm=6000,xjm=1000,
                    yfr=6000,yvm=6000,yjm=1000).items():tg.set(k,v)
    tg.send_gcode('G21 G90 G17 G94 G64 G92.1',check=True)
    tg.send_gcode('G28.3 X0 Y0 Z0',check=True,allow='declare unloaded cause-test origin')
    tg.wait_idle()
    assert rt(tg,{'fi':0,'job':0,'fg':0}).status==0
    assert rt(tg,{'fi':1,'fg':1}).status==0
    yield
    tg.send_raw(b'!%');time.sleep(.3)
    tg.send_raw(b'~');tg.wait_idle(timeout=10)
    rt(tg,{'fi':0,'fg':0,'job':0})
    tg.send_gcode('M5 M9 G21 G90 G94 G64 G92.1',check=True)


def finish(tg,mark,record,label):
    tg.wait_idle(timeout=60);rt(tg,{'fs':None});settle(tg,.4)
    fl,fs,fg=(since(tg,mark,k) for k in ('feedlimit','feedsummary','feedgaps'))
    record(label,dict(fl=fl,fs=fs,fg=fg))
    return fl,fs,fg


def test_exact_stop_tag_and_time(tg,cause_window,record):
    mark=len(tg.transport.history)
    lines=exact_stop_lines((FIXTURES/'13_exact_stop.nc').read_text().splitlines())
    for line in lines:tg.guard.check_gcode(line,allow='fixture13 exact-stop cause')
    res=sender.stream(tg,lines,'line',timeout=60)
    assert res.ok
    fl,fs,_=finish(tg,mark,record,'fixture13_exact')
    check_cause(fl,fs,4,2)


def test_single_segment_accounted_time(tg,cause_window,record):
    # Short blocks can end between periodic fl reports; fs retains their completed time.
    # No reversal, G61, above-cap F, or command boundary inside the chain.
    lines=['G21 G90 G17 G94 G64 G92 X0']+[f'N{i+10} G1 X{i*.05:.4f} F1200' for i in range(1,2001)]
    mark=len(tg.transport.history)
    for line in lines:tg.guard.check_gcode(line,allow='fixture05 single-segment chain')
    res=sender.stream(tg,lines,'line',timeout=60);assert res.ok
    fl,fs,_=finish(tg,mark,record,'fixture05_single')
    check_accounted_cause(fs,4)
    # This does not qualify the separate single-segment-with-no-other-tag case.
    # Retain sampled masks so bench evidence can establish that coverage later.
    record('single_segment_sampling',dict(masks=[e.get('m') for e in fl],
           alone_observed=any(e.get('k')==1 and e.get('m')==16 for e in fl)))


def test_command_time_in_fs_v2(tg,cause_window,record):
    mark=len(tg.transport.history)
    lines=(FIXTURES/'15_spindle_coolant.nc').read_text().splitlines()
    for line in lines:tg.guard.check_gcode(line,allow='fixture15 command boundary')
    res=sender.stream(tg,lines,'line',timeout=60);assert res.ok
    _,fs,_=finish(tg,mark,record,'fixture15_command')
    check_command(fs)


def test_pending_and_empty_gaps_then_clear(tg,cause_window,record):
    assert rt(tg,{'job':1}).status==0
    mark=len(tg.transport.history)
    tg.send_gcode('N10 G1 X5 F600',check=True,allow='fixture19 first move')
    tg.wait_idle();settle(tg,.8) # no host input, class2
    # One data line and control lines in a bounded burst. Control priority keeps data pending;
    # never exceed the returned line credit or192 bytes. Drain all replies before another burst.
    for attempt in range(30):
        free=tg.get('rx');count=min(6,max(0,int(free)-3))
        assert count>=1,'not enough line-mode credit for pending-input stimulus'
        move=f'N{100+attempt} G1 X{5+(attempt+1)*.01:.3f} F600\n'
        query='{"stat":null}\n';payload=(move+query*count).encode()
        assert len(payload)<=192
        tg.guard.check_gcode(move,allow='fixture19 pending-input stimulus')
        tg.transport.write(payload)
        replies=0;deadline=time.monotonic()+3
        while replies<count+1 and time.monotonic()<deadline:
            ev=tg.transport.next_event(.02)
            if ev and ev.kind=='response':
                assert sender._footer(ev) and sender._footer(ev)[0]==0,ev.raw
                replies+=1
            if ev and ev.kind=='exception':pytest.fail(ev.raw)
        assert replies==count+1,'pending stimulus responses missing'
    tg.wait_idle();settle(tg,.5)
    resumed_mark=len(tg.transport.history)
    tg.send_gcode('N500 G1 X20 F300',check=True,allow='fixture19 resumed feed')
    settle(tg,.8)
    resumed=since(tg,resumed_mark,'feedlimit')
    rt(tg,{'job':0})
    fl,_,fg=finish(tg,mark,record,'fixture19_gaps')
    record('fixture19_resumed',resumed)
    check_gaps(fl,fg,resumed)
