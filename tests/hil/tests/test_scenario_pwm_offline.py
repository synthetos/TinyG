"""PWM window boundaries must not merge distinct sustained spindle speeds."""
import importlib.util
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location('scenario_pwm', Path(__file__).parent / 'diagnostics/test_bench3_scenario.py')
scenario = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scenario)


def rows(values, window=.02):
    return [dict(t=i*window,duty=v) for i,v in enumerate(values)]


def test_transition_window_does_not_hide_final_programmed_speed():
    # Seed770884841: 100ms capture windows across0.1 ->0.1323 ->0.125.
    values=[.1]*72+[.1291]+[.1324]*5+[.1257]+[.125]*81
    assert scenario._plateaus(rows(values,.1)) == pytest.approx([.1,.1324,.125])


def test_plateau_requires_three_windows():
    assert scenario._plateaus(rows([.1]*5+[.16]*2+[.1]*5)) == [.1]
    assert scenario._plateaus(rows([.1]*5+[.16]*3+[.1]*5)) == [.1,.16,.1]


def test_sustained_wrong_level_and_missing_level_remain_visible():
    assert scenario._plateaus(rows([.1]*5+[.16]*5+[.13]*5)) == [.1,.16,.13]
    assert scenario._plateaus(rows([.1]*5+[.1324]*10)) != [.1,.1324,.125]
