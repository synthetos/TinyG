"""Trajectory baseline for the live feed override battery: today's planner and exec, segment by segment.

The live override engine must keep these properties with factor changes injected; this file pins them on the
unmodified code so the checker itself is proven first (feedoverride brief, 2026-11-09).
- no segment above the axis feed limit; step totals within the established 1-step DDA residual;
- tangential jerk, estimated from the exec's average segment velocities within one block, stays within 2.5x the
  block jerk on long moves, corners and zigzags (the estimate overshoots the S-curve's true jerk by up to ~2.2x);
- no speed step between consecutive blocks on those programs.
Dense chains of sub-millimetre blocks exceed the block jerk today (E015, parked): they are measured, not bounded.
"""
from pathlib import Path
import tempfile
import unittest

import trajectory_replay as tr

ROOT = Path(__file__).resolve().parents[2]

PROGRAMS = {
    "long_straight": "G21 G90 G94\nN10 G1 X100 F1200\n",
    "square": "G21 G90 G94\nN10 G1 X20 F1000\nN20 G1 Y20\nN30 G1 X0\nN40 G1 Y0\n",
    "zigzag": "G21 G90 G94\n" + "".join(f"N{i} G1 X{(i % 2) * 5} Y{i * 0.5} F1200\n" for i in range(1, 40)),
}
DENSE = "G21 G90 G94\n" + "".join(f"N{i} G1 X{i * 0.08:.3f} Y{(i % 7) * 0.01:.3f} F1200\n" for i in range(1, 400))


class TrajectoryBaseline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.exe = tr.build(ROOT, Path(cls._tmp.name))

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_bounded_programs(self):
        for name, gcode in PROGRAMS.items():
            with self.subTest(program=name):
                segs, steps = tr.run(self.exe, gcode)
                self.assertTrue(segs and tr.finite(segs))
                vmax, _ = tr.worst(segs)
                self.assertLessEqual(vmax, 1.0005, name)
                jerk, step = tr.path_jerk(segs)
                self.assertLessEqual(jerk, 2.5, name)
                self.assertLessEqual(step, 0.01, name)
                for row in steps[:2]:                            # motors 1 and 2 (X): pulses vs runtime target
                    self.assertLessEqual(abs(int(row[3]) - round(float(row[2]))), 1, (name, row))

    def test_dense_chain_is_measured(self):
        segs, steps = tr.run(self.exe, DENSE)
        self.assertTrue(segs and tr.finite(segs))
        vmax, _ = tr.worst(segs)
        self.assertLessEqual(vmax, 1.0005)
        jerk, step = tr.path_jerk(segs)
        print(f"dense chain: path jerk {jerk:.1f}x block jerk, junction speed step {step:.1f} mm/min (E015)")


if __name__ == "__main__":
    unittest.main()
