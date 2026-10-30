"""Offline calibration for edge-derived envelope estimates; no hardware needed."""
from pathlib import Path
import importlib.util
import unittest
ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('motion_profile',ROOT/'tests/hil/tinyg_hil/motion_profile.py')
profile=importlib.util.module_from_spec(spec);spec.loader.exec_module(profile)

class MotionProfileAnalysis(unittest.TestCase):
 def test_constant_speed_and_units(self):
  r=profile.analyze([i/12800 for i in range(1,2561)],1280)
  self.assertAlmostEqual(r['raw_peak_v_mm_s'],10,places=8)
  for w in r['windows']:
   self.assertTrue(w['resolved']);self.assertLess(w['peak_j_mm_s3'],.1)
 def test_constant_jerk_and_quantized_samples(self):
  jerk=1000.
  for phase in (0,1/100000):
   edges=profile.synthetic_edges(lambda t:2*t+jerk*t**3/6,.25,1280,phase=phase)
   r=profile.analyze(edges,1280,windows=(.04,.08))
   for w in r['windows']:
    self.assertTrue(w['resolved'])
    for e in w['estimates']:
     self.assertLessEqual(abs(e['j']-jerk),e['j_quantization_bound'])
   self.assertAlmostEqual(r['windows'][-1]['peak_j_mm_s3'],jerk,delta=100)
 def test_short_capture_is_underresolved(self):
  r=profile.analyze([i*.0001 for i in range(5)],1280)
  self.assertFalse(any(w['resolved'] for w in r['windows']))
 def test_bad_edges_rejected(self):
  with self.assertRaises(ValueError):profile.analyze([0,1,1,2],1280)
