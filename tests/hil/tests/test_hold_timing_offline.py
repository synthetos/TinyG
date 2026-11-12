import pytest
from tinyg_hil.hold_timing import measure


def trace(flush=False):
    return {'M1 step':[(t,1) for t in ([1,1.5,2,2.1] if flush else [1,1.5,2,2.1,5.2,5.3])]}


def pwm():return [{'t':i/10,'duty':.8} for i in range(10,60)]


def test_motion_only_intervals():
    r=measure(trace(),pwm(),{'!':102,'~':104},100)
    assert r['hold_to_stop_s']==pytest.approx(.1)
    assert r['resume_to_motion_s']==pytest.approx(1.2)
    assert r['stop_to_spindle_off_s'] is None
    assert r['spindle_unchanged']
    measure(trace(True),pwm(),{'!':102,'%':104},100,flush=True)


def test_reject_output_drop_missing_evidence_and_motion_after_flush():
    low=[dict(t=w['t'],duty=.1 if w['t']>=2.2 else .8) for w in pwm()]
    with pytest.raises(ValueError):measure(trace(True),low,{'!':102,'%':104},100,flush=True)
    with pytest.raises(ValueError):measure(trace(),low,{'!':102,'~':104},100)
    with pytest.raises(ValueError):measure(trace(),[],{'!':102,'~':104},100)
    with pytest.raises(ValueError):measure(trace(),pwm(),{'!':102,'%':104},100,flush=True)
