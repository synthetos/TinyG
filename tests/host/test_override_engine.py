"""Test-first long-move override handoffs through the real planner and exec.

The response checks require the new speed during the running block, while
preflight checks retain the endpoint and step-count requirements.
"""
from pathlib import Path
import math
import tempfile
import unittest

import override_replay as replay

ROOT = Path(__file__).resolve().parents[2]
PROGRAM = 'G21 G90 G94\nN10 G1 X100 F600\n'


class OverrideEngine(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.work = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.work.cleanup)
        cls.exe = replay.build(ROOT, Path(cls.work.name))
        cls.plain = replay.run(cls.exe, PROGRAM)
        cls.runs = {section: replay.run(cls.exe, PROGRAM, at=20, factor=500, section=section)
                    for section in (0, 1, 2)}
        cls.increase = replay.run(cls.exe, PROGRAM, at=20, factor=1500, section=1)

    def test_injections_reach_all_sections_and_keep_endpoints(self):
        # Infrastructure failures must fail this ordinary test, not disappear
        # behind an expected-failure response assertion.
        for section, result in self.runs.items():
            with self.subTest(section=section):
                self.assertEqual(len(result['requests']), 1)
                tick, actual_section, factor, generation = result['requests'][0]
                self.assertGreaterEqual(tick, 20)
                self.assertEqual(actual_section, section)
                self.assertEqual(factor, 500)
                self.assertGreater(generation, 0)
                self.assertEqual([r[:2] for r in result['steps']],
                                 [r[:2] for r in self.plain['steps']])
                for motor, target, pulses in result['steps']:
                    self.assertLessEqual(abs(target - pulses), 1.5, motor)
                self.assertTrue(all(math.isfinite(x) for row in result['segments'] for x in row))
        self.assertEqual(len(self.increase['requests']), 1)
        self.assertEqual(self.increase['requests'][0][1:3], (1, 1500))
        self.assertEqual([r[:2] for r in self.increase['steps']],
                         [r[:2] for r in self.plain['steps']])

    def _assert_body_response(self, result, target):
        tick = result['requests'][0][0]
        samples = [velocity for step, section, velocity, dt, x in result['segments']
                   if step > tick + 10 and section == 1]
        self.assertTrue(samples, 'long move must still have an unstaged body after the request')
        self.assertTrue(any(abs(velocity - target) <= 0.5 for velocity in samples),
                        f'no in-move body response to {target} mm/min; '
                        f'observed {min(samples)}..{max(samples)}')

    def test_body_reduction_takes_effect_before_the_move_ends(self):
        self._assert_body_response(self.runs[1], 300)

    def test_body_increase_takes_effect_before_the_move_ends(self):
        self._assert_body_response(self.increase, 900)


if __name__ == '__main__':
    unittest.main()
