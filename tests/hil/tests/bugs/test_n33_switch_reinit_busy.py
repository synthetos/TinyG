"""N33: a setting written during motion must not swallow a deferred limit edge.

The ICE widens and qualifies the lockout window. This verifies logic on a hooks
build, not production ISR latency or electrical switch timing.
"""
import time
import pytest
pytestmark=[pytest.mark.motion,pytest.mark.ice,pytest.mark.timing,pytest.mark.persists,
            pytest.mark.resets_board,pytest.mark.profile('switches-no'),
            pytest.mark.dialect('d449'),pytest.mark.bug('N33',on=[])]

@pytest.mark.parametrize('token',['st','ysx'])
def test_setting_preserves_deferred_limit(tg,ice,switches,keep,record,token):
    if 'test_hook' not in ice.symbols.sym:pytest.skip('requires TEST_HOOKS=1')
    keep('xsn','st','ysx','rxm')
    tg.set('rxm',1,allow='N33 control traffic during motion')
    tg.set('st',0,allow='N33 normally open test inputs')
    tg.set('xsn',0,allow='N33 prepare active initial state without a limit edge')
    switches.close('x','min')
    tg.set('xsn',2,allow='N33 enable X-min limit')
    try:
        assert ice.peek('sw.s[0].state')==1
        tg.send_gcode('G21 G90 G28.3 X0',check=True,allow='N33 seed bounded move')
        tg.send_gcode('G1 X4 F60',check=True,allow='N33 4 mm move')
        time.sleep(.1)
        ice.poke('sw.s[0].lockout_ms',1000)
        switches.open('x','min')                 # real trailing edge starts lockout
        switches.close('x','min')               # closure must be deferred
        with ice.halted():
            observed=ice.peek_many(['rtc.sys_ticks','sw.s[0].timeout._start',
                                   'sw.s[0].timeout._delay','sw.s[0].state','cm.motion_state'])
            observed['raw_pin']=ice.switch_pin('x','min')
        elapsed=(observed['rtc.sys_ticks']-observed['sw.s[0].timeout._start'])&0xffffffff
        record('qualified_closure',observed)
        assert observed['sw.s[0].state']==0 and observed['raw_pin']==0
        assert observed['sw.s[0].timeout._delay']==1000 and elapsed<800
        assert observed['cm.motion_state']==1,'move did not begin'
        reply=tg.send_json({token:0},allow='N33 same-value setting while edge deferred')
        record('setter_reply',reply.raw)
        deadline=time.monotonic()+3
        state=tg.stat()
        while state!=tg.dialect.stat('ALARM') and time.monotonic()<deadline:
            time.sleep(.05);state=tg.stat()
        assert state==tg.dialect.stat('ALARM'),'deferred limit was swallowed'
        assert reply.status==106,'busy setting was accepted'
    finally:
        switches.open('x','min')
        tg.reset()
