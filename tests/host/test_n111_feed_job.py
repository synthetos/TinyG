"""Whole-job feed scaling through the real parser, planner, executor and DDA."""
from pathlib import Path
import tempfile
import unittest
import override_replay as replay

ROOT = Path(__file__).resolve().parents[2]


class FeedJob(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.work = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.work.cleanup)
        cls.exe = replay.build(ROOT, Path(cls.work.name))

    def check_endpoints(self, result):
        previous = {}
        for move, motor, target, pulses, encoder, position in result['ends']:
            self.assertLessEqual(abs(target - pulses), 1, (move, motor))
            old_target, old_pulses = previous.get(motor, (0, 0))
            self.assertLessEqual(abs((target-old_target)-(pulses-old_pulses)), 1)
            previous[motor] = target, pulses
            self.assertEqual(encoder, pulses)
            self.assertAlmostEqual(position, target, places=4)

    def test_factor_from_first_block_covers_lines_and_arc_chords(self):
        for moves in ('G1 X20 F600\n'*4, 'G1 X0.4 F600\n'*200,
                      'G17 G2 X20 Y0 I10 J0 F600\n'):
            with self.subTest(moves=moves[:24]):
                program = 'G21 G91 G94 G64\n' + moves
                plain = replay.run(self.exe, program)
                half = replay.run(self.exe, program, initial_factor=500)
                self.check_endpoints(plain)
                self.check_endpoints(half)
                duration = lambda r: sum(row[3] for row in r['segments'])
                self.assertAlmostEqual(duration(half)/duration(plain), 2, delta=.08)
                self.assertLessEqual(max(row[2] for row in half['segments']), 300.1)

    @unittest.expectedFailure
    def test_live_change_covers_already_queued_long_blocks(self):
        program = 'G21 G91 G94 G64\n' + 'G1 X20 F600\n'*4
        plain = replay.run(self.exe, program)
        half = replay.run(self.exe, program, at=200, factor=500)
        self.check_endpoints(half)
        duration = lambda r: sum(row[3] for row in r['segments'])
        self.assertAlmostEqual(duration(half)/duration(plain), 1.875, delta=.06)

    def test_ordered_permissions_at_admission(self):
        program = 'G21 G90 G94\nM49\nG1 X20 F600\nM48\nG1 X40\n'
        result = replay.run(self.exe, program, initial_factor=500)
        self.check_endpoints(result)
        first = [r[2] for r in result['segments'] if r[4] <= 20 and r[1] == 1]
        second = [r[2] for r in result['segments'] if r[4] > 20 and r[1] == 1]
        self.assertAlmostEqual(max(first), 600, delta=.1)
        self.assertAlmostEqual(max(second), 300, delta=.1)


if __name__ == '__main__':
    unittest.main()
