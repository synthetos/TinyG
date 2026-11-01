"""E015 baseline/fix motion-envelope measurements from physical step edges.

No ICE halts. Each case captures one +X move and returns by the same distance
outside the capture. Read live geometry; never assume the nominal 1280 steps/mm.
See tinyg_hil.motion_profile for smoothing and error assumptions. This diagnostic
records estimates, not a jerk-limit verdict. Quantization and smoothing bias must
be evaluated before using these captures as a merge gate.
"""
import json
import os
from pathlib import Path
import time

import pytest
from tinyg_hil import benches, saleae
from tinyg_hil.motion_profile import analyze

pytestmark=[pytest.mark.diagnostic,pytest.mark.motion,pytest.mark.dialect('d449')]
LENGTHS=[.01,.02,.05,.1,.25,.5,1.,2.,5.,10.]

@pytest.mark.parametrize('length',LENGTHS)
def test_x_motion_profile(tg,record,request,length):
    holder=json.loads(Path(f'/tmp/tinyg-hil-{Path(tg.transport.port).name}.lock').read_text())
    assert holder['pid']==os.getpid() and holder['owner']==os.environ.get('TINYG_OWNER','unknown')   # as lock.py records it
    config=benches.bench(tg.transport.port)
    if not config.get('saleae'):
        pytest.skip(f'no Saleae recorded for {tg.transport.port}')
    assert config['saleae_channels']['6']=='M1 step' and config['saleae_channels']['7']=='M1 dir'
    assert '1' not in config['saleae_channels']
    settings={t:tg.get(t) for t in ('1ma','1sa','1mi','1tr','1po','xjm','xvm','xfr','xam')}
    assert settings['1ma']==0 and settings['xam']==1
    assert all(tg.get(f'{m}ma')!=0 for m in (2,3,4)), 'X must map to motor 1 only'
    spu=360/settings['1sa']*settings['1mi']/settings['1tr']
    assert spu>0
    feed=min(600.,settings['xvm'],settings['xfr'])
    jerk=settings['xjm']*1_000_000/60**3
    assert feed>0 and jerk>0
    recorder=request.config._tinyg_recorder
    assert recorder, 'run records required'
    dest=recorder.dir.resolve()/f'profile-X-{length:g}mm'
    tg.send_gcode('G21 G91 G94',check=True);tg.wait_idle()
    cap=saleae.Capture(config['saleae'],channels=[6,7],seconds=None,rate=12_000_000,keep_open=True)
    moved=False
    try:
        cap.__enter__()
        time.sleep(.15)
        tg.send_gcode(f'G1 X{length:g} F{feed:g}',check=True,allow='single-axis motion profile sweep')
        tg.wait_idle(timeout=60)
        moved=True
        time.sleep(.15)
        edges=cap.finish()
        saleae.call('save_capture',captureId=cap.capture_id,filepath=str(dest)+'.sal',timeout=30)
        rises=saleae.rising(edges[6])
        raw={'sample_rate':12_000_000,'initial':cap.initial,'transitions':edges,'step_rises':rises}
        Path(str(dest)+'-edges.json').write_text(json.dumps(raw)+'\n')
        result=analyze(rises,spu)
        result.update({'length_mm':length,'feed_mm_min':feed,'commanded_jerk_mm_s3':jerk,
                       'settings':settings,'steps_per_mm':spu,'capture':str(dest)+'.sal',
                       'edge_file':str(dest)+'-edges.json','smoothing':'degree-5 position fits, 10/20/40/80ms centered windows',
                       'direction_initial':cap.initial[7],'direction_transitions':edges[7],
                       'gate_verdict':'measurement only; bias/quantization review required'})
        for window in result['windows']:
            window['peak_jerk_ratio']=window['peak_j_mm_s3']/jerk if window['resolved'] else None
        Path(str(dest)+'-analysis.json').write_text(json.dumps(result,indent=2)+'\n')
        # Full derivative traces are in the artifact, not duplicated into run metadata.
        summary={k:v for k,v in result.items() if k not in ('raw','windows')}
        summary['windows']=[{k:v for k,v in w.items() if k!='estimates'} for w in result['windows']]
        record('motion_profile',summary)
        assert abs(len(rises)-round(length*spu))<=2, 'unexpected delivered step count'
    finally:
        if cap.capture_id is not None:
            try:
                if cap._edges is None:
                    saleae.call('stop_capture',captureId=cap.capture_id,timeout=15)
            finally:
                saleae.call('close_capture',captureId=cap.capture_id,timeout=15)
                if saleae._KEPT.exists() and saleae._KEPT.read_text().strip()==str(cap.capture_id):
                    saleae._KEPT.unlink()
        if moved:
            tg.send_gcode(f'G1 X{-length:g} F{feed:g}',check=True,allow='return after profile capture')
            tg.wait_idle(timeout=60)
        tg.send_gcode('G90',check=True)
