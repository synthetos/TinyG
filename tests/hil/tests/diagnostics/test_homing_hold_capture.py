"""E114: physical switch-edge/step captures, with no ICE halt during motion.

Requires confirmed ch0 -> X-min signal, M1 step/dir ch6/ch7, and TEST_HOOKS=1
for a firmware-mounted switch. Export TINYG_XMIN_CAPTURE_CHANNEL=0 only after
wiring is confirmed. The hooks build exercises real electrical edges and the
switch ISR, but is not a normal-build latency certificate. A baseline pass does
not disprove the rare E114 race; the deterministic host interleaving is primary.
"""
import json
import os
from pathlib import Path
import time
import pytest
from tinyg_hil import saleae
from tinyg_hil.benches import bench as bench_config
from tinyg_hil.lock import BoardLock
from tinyg_hil.homing_trace import summarize

pytestmark=[pytest.mark.diagnostic,pytest.mark.timing,pytest.mark.ice,
            pytest.mark.motion,pytest.mark.persists,pytest.mark.resets_board,
            pytest.mark.dialect('d449'),pytest.mark.profile('switches-no')]

@pytest.mark.parametrize('speed',[120,300])
def test_homing_hold_capture(tg,ice,switches,keep,record,request,speed):
    if os.environ.get('TINYG_XMIN_CAPTURE_CHANNEL')!='0':
        pytest.skip('requires confirmed Saleae ch0 on X-min')
    holder=json.loads(BoardLock(tg.transport.port).holder())
    assert holder['pid']==os.getpid(), 'capture requires the session board lock'
    try:ice.symbols.resolve('test_hook.mount[0]')
    except KeyError:pytest.skip('firmware-mounted switch required; no halted polling during capture')
    bench=bench_config(tg.transport.port)
    if not bench.get('saleae'):
        pytest.skip('configure a Saleae device in the local bench configuration')
    assert bench['saleae_channels']['6']=='M1 step' and bench['saleae_channels']['7']=='M1 dir'
    assert tg.get('1ma')==0
    assert all(tg.get(f'{m}ma')!=0 for m in (2,3,4)), 'only M1 may drive X'
    spu=360/tg.get('1sa')*tg.get('1mi')/tg.get('1tr')
    settings={'xsn':1,'xsx':2,'xtn':-2,'xtm':2,'xsv':speed,'xlv':60,'xlb':1,'xzb':.2,'xjh':20}
    keep(*settings)
    for k,v in settings.items():tg.set(k,v,allow='bounded homing capture profile')
    rounds=[]
    for iteration in range(3):
        switches.unmount_all();switches.open('x','min');switches.open('x','max')
        tg.send_gcode('G21 G90',check=True)
        tg.send_gcode('G28.3 X0',check=True,allow='set local bare-board origin')
        switches.mount('x','min',at=-.5)
        assert ('x','min') in switches.frames and switches._thread is None
        dest=request.config._tinyg_recorder.dir.resolve()/f'homing-{speed}-{iteration}'
        cap=saleae.Capture(bench['saleae'],channels=[0,6,7],seconds=None,rate=12000000,save_as=dest)
        completed=False
        try:
            cap.__enter__();time.sleep(.1)
            tg.send_gcode('G28.2 X0',check=True,allow='home X against mounted input at -0.5 mm')
            deadline=time.monotonic()+25
            saw_unhomed=False
            while time.monotonic()<deadline:
                # Serial only: no debugger halts while the switch and step edges run.
                homed=tg.get('homx')
                if homed==0:saw_unhomed=True
                elif saw_unhomed:
                    tg.wait_idle(timeout=10);completed=True;break
                time.sleep(.05)
            time.sleep(.15)
        finally:
            try:
                if cap.capture_id is not None:
                    edges=cap.finish()  # saves the capture/edges and closes the tab, even on failure
            finally:
                if not completed:
                    tg.reset()
                    switches.unmount_all()
        assert completed,'homing did not finish within 25 seconds; capture retained'
        # The capture has stopped. Reading the final logical state is now safe.
        state=ice.peek_many(['cm.cycle_state','cm.hold_state','cm.homed[0]','mr.position[0]'])
        mount=switches.mount_status('x','min');physical=switches.physical_position('x')
        trace=summarize(edges,cap.initial,spu)
        result={'speed_mm_min':speed,'iteration':iteration,'jerk_homing':20,
                'trace':trace,'final':state,'mount':mount,'physical_position_mm':physical,
                'capture':dest.name+'.sal','sample_rate':12000000,'steps_per_mm':spu,
                'verdict':'captured integration evidence; race reproduction and normal-build timing not assumed'}
        Path(str(dest)+'-analysis.json').write_text(json.dumps(result,indent=2)+'\n')
        rounds.append(result);record('homing_captures',rounds)
        switches.unmount_all()
        assert state['cm.cycle_state']==0 and state['cm.hold_state']==0 and state['cm.homed[0]']==1
        assert state['mr.position[0]']==pytest.approx(0,abs=.005)
        assert trace['switch_initial']==1 and len(trace['switch_closures_s'])==1
        assert len(trace['switch_openings_s'])==1 and len(trace['direction_changes_at_steps_s'])==1
        assert trace['input_to_last_search_step_s']>=0
        assert -.5 < trace['final_position_mm'] < .2
        assert -2 < trace['min_position_mm'] and trace['max_position_mm'] < .75
        assert trace['final_position_mm']==pytest.approx(physical,abs=3/spu)
        assert mount['trips']==1 and not mount['closed']
