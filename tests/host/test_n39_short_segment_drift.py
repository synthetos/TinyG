"""N39: accepted dense travel must reach its endpoint in runtime and counted pulses."""
from pathlib import Path
import subprocess,tempfile,unittest
import step_replay
ROOT=Path(__file__).resolve().parents[2]
class ShortSegmentDrift(unittest.TestCase):
 def test_dense_excerpt_conserves_endpoint(self):
  data=(ROOT/'tests/hil/corpus/n39/dense-isolation-excerpt.nc').read_bytes()
  with tempfile.TemporaryDirectory() as d:
   exe=step_replay.build(ROOT,Path(d))
   for batch in [4,16,28]:
    p=subprocess.run([str(exe),str(batch)],input=data,capture_output=True,timeout=30)
    self.assertEqual(p.returncode,0,p.stderr.decode())
    rows=[s.split() for s in p.stdout.decode().splitlines() if s.startswith('STEPS ')]
    for row in rows[:2]:
     self.assertLessEqual(abs(float(row[2])),.01,('runtime endpoint',batch,row))
     self.assertLessEqual(abs(int(row[3])),1,('pulse endpoint',batch,row))
if __name__=='__main__':unittest.main()
