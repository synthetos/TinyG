"""MSO physical timing plus observed status through E060 hold/resume/flush.

Command-to-edge intervals include MSO start alignment uncertainty (100 ms).
Edge-to-edge spindle/step intervals share the capture clock. No board use offline.
"""
import time
import pytest
from tinyg_hil import benches,mso,perf
from tinyg_hil.hold_timing import measure

pytestmark=[pytest.mark.diagnostic,pytest.mark.motion,pytest.mark.persists,
            pytest.mark.dialect('d449'),pytest.mark.timeout(120)]


@pytest.mark.parametrize('flush',[False,True],ids=['hold-resume','hold-flush'])
def test_e060_timing(tg,keep,record,request,flush):
    cfg=(benches.bench(tg.transport.port) or {}).get('mso')
    if not cfg:pytest.skip('requires bench3 MSO and spindle PWM')
    pwm_name=(cfg.get('analog') or {}).get('0')
    assert pwm_name,'spindle PWM channel missing'
    recorder=request.config._tinyg_recorder
    assert recorder is not None
    stem=recorder.dir/f'motion-hold-timing-{int(flush)}'
    settings=dict(p1csl=0,p1csh=1000,p1cpl=.1,p1cph=.8,p1pof=.1,sv=1,si=100)
    keep(*settings)
    info=dict(kind='e060-timing',flush=flush,statuses=[],commands={},valid=False)
    def status_until(predicate,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            before=time.time();s=tg.get('stat');info['statuses'].append(dict(sent=before,received=time.time(),stat=s))
            if predicate(s):return s
            time.sleep(.05)
        raise AssertionError('status deadline: '+str(info['statuses'][-5:]))
    def command(data):
        info['commands'][data.decode()]=time.time();tg.send_raw(data)
    try:
        for k,v in settings.items():tg.set(k,v)
        tg.send_gcode('G21 G90 G94 G64 G92.1 M5',check=True)
        tg.send_gcode('G28.3 X0',check=True,allow='declare unloaded bench origin')
        tg.send_gcode('S1000 M3',check=True,allow='unloaded spindle output timing')
        tg.wait_idle();time.sleep(.2)
        digital=[f'M{m} {k}' for m in range(1,5) for k in ('step','dir')]
        mark=len(tg.transport.history)
        with mso.Capture(cfg,digital=digital,analog=[pwm_name],seconds=25,rate=1_000_000,
                         pwm_window=.02,save_as=stem) as cap:
            time.sleep(.35)
            tg.send_gcode('G1 X60 F600',check=True,allow='feedhold timing move')
            status_until(lambda s:s==tg.dialect.stat('RUN'),3)
            time.sleep(.7);command(b'!')
            status_until(lambda s:s==tg.dialect.stat('HOLD'),5)
            # Observe a whole second held before requesting resume/flush.
            until=time.monotonic()+1
            while time.monotonic()<until:
                info['statuses'].append(dict(received=time.time(),stat=tg.get('stat')));time.sleep(.05)
            command(b'%' if flush else b'~')
            if not flush:status_until(lambda s:s==tg.dialect.stat('RUN'),4)
            status_until(lambda s:s in tg.dialect.idle_stats,12)
            info['settled_wall']=time.time()
        assert info['settled_wall']-cap.started_wall<24.5,'capture did not contain completion'
        info['metrics']=measure(cap.edges(),cap.pwm(pwm_name),info['commands'],cap.started_wall,
                                flush=flush)
        info['capture_file']=str(cap.out)
        info['automatic_status']=[dict(t=e.t,body=e.body) for e in tg.transport.history[mark:] if e.kind=='status']
        info['valid']=True
    except Exception as e:
        info['error']=str(e);raise
    finally:
        perf.write_record(str(stem)+'.json',info);record('e060_timing',info)
        if not info['valid']:tg.reset()
        else:tg.send_gcode('M5 M9',check=True,allow='stop timing outputs')
