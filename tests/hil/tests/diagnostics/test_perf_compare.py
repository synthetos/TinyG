"""Full-file A/B capture. FIRMWARE_TAG=A|B SLOT=1 RT=off|on.

PERF_DENSE=1 enables fixture 21. PERF_CAPTURE_S overrides the fixed MSO capture
budget; PERF_TIMEOUT_S bounds the stream. PERF_SOURCE=mso|saleae selects the
instrument. Bench 2 explicitly lacks M4; never treated as four-motor coverage.
"""
import json
import os
import time
from pathlib import Path
import pytest
from tinyg_hil import benches, gcodeplay, mso, saleae, state
from tinyg_hil import perf, perf_snapshot
from tinyg_hil.flash import read_provenance
from tinyg_hil.hostpower import AwakeSession

pytestmark=[pytest.mark.diagnostic,pytest.mark.motion,pytest.mark.persists,
            pytest.mark.dialect('d449'),pytest.mark.timeout(7200)]
FIXTURES=Path(__file__).resolve().parents[2]/'fixtures/planner_timing'
FILES=sorted(p.name for p in FIXTURES.glob('*.nc'))+['perfbench']
BUDGET=json.loads((FIXTURES/'capture-budgets.json').read_text())


def capture_config(bench, source):
    if source=='mso':
        cfg=bench.get('mso') or {}; names=set((cfg.get('digital') or {}).values())
    else:
        cfg=bench;names=set((bench.get('saleae_channels') or {}).values())
        bad=set(bench.get('saleae_faulty_channels') or [])
        names-= {v for k,v in (bench.get('saleae_channels') or {}).items() if int(k) in bad}
    motors=[f'M{m}' for m in range(1,5) if all(f'M{m} {k}' in names for k in ('step','dir'))]
    if not motors or (source=='mso' and len(motors)!=4):
        raise ValueError('MSO needs all four step/dir pairs; Saleae needs mapped healthy pairs')
    return cfg,motors


@pytest.mark.parametrize('fixture',FILES)
def test_perf_compare(tg,keep,record,request,fixture):
    if fixture.startswith('21_') and os.environ.get('PERF_DENSE')!='1':
        pytest.skip('42-minute dense fixture; enable PERF_DENSE=1')
    tag,slot,rt=perf.options(os.environ)
    probe_interval=perf.probe_options(os.environ)
    probe_kind,probe_ms=perf.probe_parameters(os.environ)
    probe_mode="off" if probe_interval is None else "on"
    recorder=request.config._tinyg_recorder
    assert recorder is not None,'capture evidence requires recording'
    lines=gcodeplay.perfbench() if fixture=='perfbench' else (FIXTURES/fixture).read_text().splitlines()
    bench=benches.bench(tg.transport.port) or {}
    source=os.environ.get('PERF_SOURCE','mso' if bench.get('mso') else 'saleae')
    assert source in ('mso','saleae')
    cfg,motors=capture_config(bench,source)
    requested_budget=float(os.environ.get('PERF_CAPTURE_S',BUDGET.get(fixture,600)))
    budget,timeout=perf.capture_budget(source,requested_budget,os.environ.get('PERF_TIMEOUT_S'))
    assert 2<timeout<budget,'capture must outlast the bounded stream'
    stem=recorder.dir/f'perf-{Path(fixture).stem}-{tag}-{slot}-{rt}-probes-{perf.probe_suffix(os.environ)}'
    info=dict(schema=1,kind='bench-perf',fixture=fixture,sha256=perf.digest(lines),tag=tag,slot=slot,rt=rt,
              perf_probes=probe_mode,perf_probe_kind=probe_kind,perf_probe_ms=probe_ms,bench=bench.get('name'),identity=dict(tg.identity),source=source,motors_captured=motors,
              missing_motors=sorted(set(['M1','M2','M3','M4'])-set(motors)),capture_s=budget,requested_capture_s=requested_budget,sample_rate_hz=2_000_000 if source=='mso' else 12_000_000,
              parse_bound=fixture.startswith(('05_','06_','21_')),valid=False)
    if source=='mso':info['capture_channel_map']=cfg['digital']
    prepared=False
    power=AwakeSession()
    try:
        # Reject a mislabeled image before any settings write, G-code or capture.
        info['identity']['fb']=tg.get('fb')
        info['flash']=perf.flash_metadata(read_provenance(tg.transport.port))
        info.update(perf.validate_firmware(info['identity'],info['flash'],tag,os.environ))
        fixed,meta=perf_snapshot.load(os.environ.get('PERF_SNAPSHOT'))
        info['fixed_snapshot']=meta
        # The pytest session has already applied these protocol settings, before this test.
        baseline=dict(js=1,jv=4,ee=0,ec=0,tv=1,sv=1,qv=0)
        info['fixed_preflight']=perf_snapshot.verify(tg,fixed,overrides=baseline)
        prepared=True
        power.start()
        keep('ej','rxm','ex','qv','sv','si')
        for token,value in dict(ej=1,rxm=1,ex=1,qv=1,sv=1,si=250).items(): tg.set(token,value)
        if tag=='B':
            response=tg.send_json({'rt':{'fi':1 if rt=='on' else 0,'fg':1 if rt=='on' else 0}})
            assert response.status==0,'B requires rt support'
        # A deliberately never sends rt, including cleanup.
        settings=perf.settings_snapshot(tg,include_new=tag=='B')
        info['settings']=settings
        assert 'stat' in settings['sr'],'automatic stat reports required for probe-free completion'
        if tag=='A':assert 'vel' in settings['sr'],'449 requires automatic velocity reports for hold/completion'
        effective=dict(baseline,ej=1,rxm=1,ex=1,qv=1,sv=1,si=250)
        info['fixed_verified']=perf_snapshot.verify(tg,fixed,overrides=effective)
        for line in lines:
            if not line.startswith(';'): tg.guard.check_gcode(line,allow='unloaded bench performance file')
        tg.send_gcode('G21 G90 G17 G94 G64 G54 G92.1 M5 M9',check=True)
        tg.send_gcode('G28.3 X0 Y0 Z0 A0',check=True,allow='declare unloaded bench origin')
        tg.wait_idle()
        info['start']={a:tg.get('mpo'+a) for a in 'xyza'}
        plus={m:1 if tg.get(m[1:]+'po') else 0 for m in motors}
        digital=[f'{m} {k}' for m in motors for k in ('step','dir')]
        if source=='mso':
            cap=mso.Capture(cfg,digital=digital,seconds=budget,rate=2_000_000,save_as=stem)
        else:
            mapping={v:int(k) for k,v in cfg['saleae_channels'].items()}
            # Capture recovery records are persistent for this collector too.
            state.state_dir().mkdir(parents=True,exist_ok=True)
            saleae._ACTIVE=state.state_dir()/'perf-saleae-active'
            saleae._KEPT=state.state_dir()/'perf-saleae-kept'
            cap=saleae.Capture(cfg['saleae'],channels=[mapping[n] for n in digital],seconds=None,
                               rate=12_000_000,save_as=stem)
        with cap:
            time.sleep(.35)
            info['serial']=perf.stream(tg,lines,timeout,probe_interval=probe_interval,probe_kind=probe_kind)
            power.check()
            assert info['serial']['completion']=='idle' and not info['serial']['errors'],info['serial']['errors']
            info['end']={a:tg.get('mpo'+a) for a in 'xyza'}
            time.sleep(.35)
        if source=='mso':
            edges,initial=cap.edges(),cap.initial()
            info['capture_file']=str(cap.out)
        else:
            raw=cap.edges();edges={n:raw[mapping[n]] for n in digital}
            initial={n:cap.initial[mapping[n]] for n in digital}
            info['capture_file']=str(stem)+'-edges.json'
        info['metrics']=perf.steps(edges,initial,plus)
        if source=='mso':
            last=info['metrics']['last_step_s']
            assert last is not None and last<budget-.25,'capture ended before a quiet tail'
            assert info['serial']['finished']-info['serial']['started']<budget-.7,'stream outlasted capture'
        assert info['serial']['completion']=='idle' and not info['serial']['errors'],info['serial']['errors']
        assert info['metrics']['motion_s'] is not None,'no motion captured'
        info['valid']=True
    except Exception as e:
        info['error']=str(e)
        raise
    finally:
        power.close()
        perf.write_record(str(stem)+'.json',info);record('perf_compare',info)
        # Reset on failure, so a timed-out hold or an old-firmware panic cannot leak into the next case.
        if prepared and not info['valid']: tg.reset()
        elif prepared:
            tg.send_gcode('M5 M9 G21 G90 G94 G64 G92.1',check=True)
            if tag=='B': tg.send_json({'rt':{'fi':0,'fg':0,'job':0}})
