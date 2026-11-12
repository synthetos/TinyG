import pytest
from tinyg_hil.feed_causes import check_cause,check_gaps,check_command


def summary(i):
    c=[0]*7;c[i]=10
    return [{'v':2,'c':c}]


def test_exact_and_single_are_distinct():
    check_cause([{'k':1,'m':4}],summary(2),4,2)
    check_cause([{'k':1,'m':16}],summary(4),16,4,alone=True)
    with pytest.raises(AssertionError):check_cause([{'k':1,'m':24}],summary(4),16,4,alone=True)
    with pytest.raises(AssertionError):check_cause([{'k':1,'m':4}],summary(4),4,2)


def test_gap_classes_and_clear_are_required():
    fl=[{'k':0,'g':2}];fg=[{'g':[10,500,1]}];resume=[{'k':1,'m':0}]
    check_gaps(fl,fg,resume)
    with pytest.raises(AssertionError):check_gaps(fl,[{'g':[0,500,1]}],resume)
    with pytest.raises(AssertionError):check_gaps(fl,fg,[{'k':1,'g':2}])


def test_command_requires_v2_counter_six():
    check_command(summary(6))
    with pytest.raises(AssertionError):check_command(summary(5))


def test_single_accounting_survives_periodic_report_missing_the_block():
    from tinyg_hil.feed_causes import check_accounted_cause
    check_accounted_cause(summary(4), 4)
    with pytest.raises(AssertionError): check_accounted_cause(summary(3), 4)


def test_exact_stop_stimulus_uses_tinyg_exact_stop_mode():
    from tinyg_hil.feed_causes import exact_stop_lines
    lines = exact_stop_lines(['G61', 'G1 X20 F2000', 'G64'])
    assert lines == ['G61.1', 'G1 X20 F2000', 'G64']
