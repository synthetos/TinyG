import pytest
from tinyg_hil.hold_timing import measure


def trace(flush=False):
    return {'M1 step':[(t,1) for t in ([1,1.5,2,2.1] if flush else [1,1.5,2,2.1,5.2,5.3])]}


def pwm():return [{'t':i/10,'duty':.1 if 2.2<=i/10<4.1 else .8} for i in range(10,60)]


def test_physical_intervals_and_command_uncertainty():
    r=measure(trace(),pwm(),{'!':102,'~':104},100)
    assert r['hold_to_stop_s']==pytest.approx(.1)
    assert r['stop_to_spindle_off_s']==pytest.approx(.1)
    assert r['resume_to_spindle_on_s']==pytest.approx(.1)
    assert r['spindle_on_to_motion_s']==pytest.approx(1.1)
    assert r['command_clock_uncertainty_s']==.1


def test_pause_disabled_and_flush():
    high=[dict(t=w['t'],duty=.8) for w in pwm()]
    assert measure(trace(),high,{'!':102,'~':104},100,pause=False)['resume_to_spindle_on_s'] is None
    low=[dict(t=w['t'],duty=.1 if w['t']>=2.2 else .8) for w in pwm()]
    measure(trace(True),low,{'!':102,'%':104},100,flush=True)
    with pytest.raises(ValueError):measure(trace(),low,{'!':102,'%':104},100,flush=True)
    with pytest.raises(ValueError):measure(trace(),pwm(),{'!':102,'~':104},100,pause=False)
