from tinyg_hil.switch_window import qualify


def row(ticks, start, state, pin, delay=50):
    return dict(ticks=ticks, start=start, state=state, raw_pin=pin, delay=delay)


def test_release_rearmed_timer_is_not_a_valid_window():
    result=qualify(row(1650,0,0,1,0),row(1670,1660,1,0),row(1730,1720,0,1),50)
    assert result['outcome']=='invalid'
    assert result['elapsed_ms']==70 and result['original_start']==1660


def test_valid_ignored_release():
    assert qualify(row(1650,0,0,1,0),row(1670,1660,1,0),row(1680,1660,1,1),50)['outcome']=='valid'


def test_missed_edge_is_not_hidden_by_retries():
    result=qualify(row(1650,0,0,1,0),row(1670,0,0,0,0),row(1680,0,0,1,0),50)
    assert result['outcome']=='failure'


def test_bad_initial_state_and_late_observation_are_invalid():
    assert qualify(row(1650,1640,0,1),row(1670,1660,1,0),row(1680,1660,1,1),50)['outcome']=='invalid'
    assert qualify(row(1650,0,0,1,0),row(1670,1660,1,0),row(1720,1660,0,1),50)['outcome']=='invalid'


def test_proven_in_window_state_change_is_a_failure():
    assert qualify(row(1650,0,0,1,0),row(1670,1660,1,0),row(1680,1680,0,1),50)['outcome']=='failure'


def test_tick_wrap_and_strict_expiry_boundary():
    before=row(0xffffffe0,0,0,1,0)
    closed=row(0xfffffff0,0xfffffff0,1,0)
    assert qualify(before,closed,row(34,0xfffffff0,1,1),50)['outcome']=='valid'
    assert qualify(before,closed,row(35,0xfffffff0,0,1),50)['outcome']=='invalid'


def test_retry_invalid_then_valid_never_counts_invalid_as_pass():
    from tinyg_hil.switch_window import trials
    samples=iter([row(1650,0,0,1,0),row(1670,1660,1,0),row(1730,1720,0,1),
                  row(1800,0,0,1,0),row(1820,1810,1,0),row(1830,1810,1,1)])
    actions=[]; recorded=[]
    result=trials(lambda:next(samples),lambda:actions.append('close'),lambda:actions.append('release'),
                  lambda:actions.append('prepare'),lambda k,v:recorded.append(v),50)
    assert result['attempt']==2 and result['outcome']=='valid'
    assert actions==['prepare','close','release']*2
    assert [x['outcome'] for x in recorded[-1]]==['invalid','valid']


def test_exhaustion_returns_no_valid_trial_and_failure_is_not_retried():
    import pytest
    from tinyg_hil.switch_window import trials
    invalid=[row(1650,0,0,1,0),row(1670,1660,1,0),row(1730,1720,0,1)]
    samples=iter(invalid*5); recorded=[]
    assert trials(lambda:next(samples),lambda:None,lambda:None,lambda:None,
                  lambda k,v:recorded.append(v),50) is None
    assert len(recorded[-1])==5 and all(r['outcome']=='invalid' for r in recorded[-1])
    missed=iter([row(1650,0,0,1,0),row(1670,0,0,0,0),row(1680,0,0,1,0)])
    with pytest.raises(AssertionError,match='active input was not cached'):
        trials(lambda:next(missed),lambda:None,lambda:None,lambda:None,lambda *a:None,50)
