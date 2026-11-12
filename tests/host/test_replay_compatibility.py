"""Optional real-tree matrix; paths are explicit, no serial hardware involved."""
import os
from pathlib import Path
import tempfile
import unittest
import planner_timing as timing
import trajectory_replay as trajectory

class Compatibility(unittest.TestCase):
    def test_real_trees(self):
        trees=[os.environ.get('TINYG_REPLAY_BASE'),os.environ.get('TINYG_REPLAY_CAND')]
        if not all(trees): self.skipTest('set TINYG_REPLAY_BASE and TINYG_REPLAY_CAND')
        parent=Path(os.environ.get('TINYG_HIL_STATE','/Volumes/External/projects/TinyG-work/state'))
        parent.mkdir(parents=True,exist_ok=True)
        import json
        profile=json.loads((Path(__file__).resolve().parents[1]/'hil/fixtures/planner_timing/profiles/bench1.json').read_text())
        with tempfile.TemporaryDirectory(dir=parent,prefix='replay-matrix-') as tmp:
            # Reuse output to catch stale compatibility headers in both directions.
            for tree in (trees[0],trees[1],trees[0]):
                with self.subTest(tree=tree):
                    exe=timing.build(Path(tree),Path(tmp)/'build',profile)
                    r=timing.replay(exe,'G21 G90 G94\nG1 X10 F600\nG4 P0.2\nM2\n')
                    self.assertGreater(r['motion_s'],0)
                    self.assertAlmostEqual(r['dwell_s'],.2,places=5)
                    with self.assertRaises(RuntimeError):
                        timing.replay(exe,'G21 G90 G94\nG1 X1 F0\n')
                    with self.assertRaises(RuntimeError):
                        trajectory.run(exe,'G21 G90 G94\nG1 X1 F0\n')

if __name__=='__main__':unittest.main()
