"""Limit-stop position checks include report quantization, not rounded pulse distances."""
import importlib.util
from pathlib import Path
import pytest
spec=importlib.util.spec_from_file_location('stop_combos',Path(__file__).parent/'diagnostics/test_bench3_stop_combos.py')
stop=importlib.util.module_from_spec(spec);spec.loader.exec_module(stop)
class Board:
    dialect=None
    def __init__(self):self.dialect=self
    def stat(self,name):return 2

def record():
    return dict(edge_s=5.119877,stat=2,steps_m2=62387,steps_m3=62387,
                reported_mm=48.741,step_mm=48.7398,spm=1280.)

def test_saved_stop_fits_report_rounding_and_one_step():
    stop._check_kept(Board(),record())

def test_position_outside_quantization_budget_still_fails():
    s=record();s['reported_mm']=48.742
    with pytest.raises(AssertionError,match='position not kept'):stop._check_kept(Board(),s)

def test_motor_disagreement_still_fails():
    s=record();s['steps_m3']-=1
    with pytest.raises(AssertionError,match='two Y motors'):stop._check_kept(Board(),s)
