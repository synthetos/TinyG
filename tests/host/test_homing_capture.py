"""Independent checks for captured-edge homing measurements."""
import sys
import importlib
import types
from pathlib import Path
import unittest
package=types.ModuleType('homing_capture_helpers')
package.__path__=[str(Path(__file__).resolve().parents[1]/'hil/tinyg_hil')]
sys.modules[package.__name__]=package
summarize=importlib.import_module('homing_capture_helpers.homing_trace').summarize

class HomingCapture(unittest.TestCase):
    def test_electrical_trigger_and_signed_steps(self):
        edges={0:[(.25,0),(.65,1)],6:[],7:[(.5,1)]}
        for t in (.1,.2,.3,.4,.6,.7,.8):edges[6]+=[(t,1),(t+.01,0)]
        out=summarize(edges,{0:1,6:0,7:0},10)
        self.assertEqual(out['step_count'],7)
        self.assertAlmostEqual(out['input_to_last_search_step_s'],.15)
        self.assertAlmostEqual(out['input_to_first_reverse_step_s'],.35)
        self.assertAlmostEqual(out['min_position_mm'],-.4)
        self.assertAlmostEqual(out['final_position_mm'],-.1)
        self.assertEqual(out['direction_changes_at_steps_s'],[.6])
    def test_open_probe_is_not_assumed_to_be_a_trigger(self):
        out=summarize({0:[],6:[(.1,1),(.11,0)],7:[]},{0:1,6:0,7:1},100)
        self.assertIsNone(out['input_to_last_search_step_s'])
        self.assertEqual(out['switch_closures_s'],[])
