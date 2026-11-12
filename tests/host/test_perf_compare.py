import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
import subprocess
import planner_timing

spec = importlib.util.spec_from_file_location('perf_compare', Path(__file__).resolve().parents[2]/'tools/perf/compare.py')
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)

def record(t=10, **kw):
    return dict(profile={'axis': 1}, files=[dict(name='a.nc', sha256='same', host={'duration_s':t}, **kw)])

class Compare(unittest.TestCase):
    def test_threshold(self):
        self.assertEqual(m.compare(record(),record(10.1))[0]['result'],'OK')
        self.assertEqual(m.compare(record(),record(10.11))[0]['result'],'SLOWER')
    def test_speedup_needs_review(self):
        self.assertEqual(m.compare(record(), record(9.9))[0]['result'], 'OK')
        for notes in ({}, {'a.nc': 'same'}, {'a.nc': 'differs:E180'}):
            with self.subTest(notes=notes):
                self.assertEqual(m.compare(record(), record(9.89), notes)[0]['result'], 'FASTER-REVIEW')
                self.assertEqual(m.compare(record(), record(2.633), notes)[0]['result'], 'FASTER-REVIEW')

    def test_error_never_becomes_fast_time(self):
        r=m.compare(record(error='EXEC 20'),record(),{'a.nc':'differs:N105'})[0]
        self.assertEqual(r['result'],'FINDING'); self.assertIsNone(r['delta_pct'])
    def test_profile(self):
        b=record(); b['profile']={}
        with self.assertRaises(ValueError):m.compare(record(),b)
    def test_identity_and_missing(self):
        b=record();b['files'][0]['sha256']='changed'
        self.assertEqual(m.compare(record(),b)[0]['result'],'FINDING')
        b['files'][0]['name']='other.nc'
        self.assertEqual(len(m.compare(record(),b)),2)
    def test_invalid(self):
        for t in (0,-1,float('nan'),float('inf')):
            self.assertEqual(m.compare(record(),record(t))[0]['result'],'FINDING')
    def test_replay_rejects_errors_even_with_zero_process_exit(self):
        for output, err in [('EXEC 20\n',''), ('STATE 3 2\n',''), ('LINE 2 154\n',''),
                            ('','EXCEPTION 154 arc error'), ('SEG nan 0 0 0 0 0 0 1 1\n','')]:
            with self.subTest(output=output, err=err):
                result=subprocess.CompletedProcess([],0,output.encode(),err.encode())
                with patch.object(planner_timing.subprocess,'run',return_value=result):
                    with self.assertRaises(RuntimeError):planner_timing.replay('unused','')
    def test_duplicate(self):
        b=record();b['files']*=2
        with self.assertRaises(ValueError):m.compare(record(),b)

if __name__=='__main__':unittest.main()
