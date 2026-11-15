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
                plain = replay.run(self.exe, program.replace("F600", "F300"))
                half = replay.run(self.exe, program, initial_factor=500)
                self.check_endpoints(plain)
                self.check_endpoints(half)
                duration = lambda r: sum(row[3] for row in r['segments'])
                self.assertAlmostEqual(duration(half)/duration(plain), 1, delta=.03)
                self.assertLessEqual(max(row[2] for row in half['segments']), 300.1)

    def test_live_change_covers_already_queued_long_blocks(self):
        program = 'G21 G91 G94 G64\n' + 'G1 X20 F600\n'*4
        plain = replay.run(self.exe, program)
        half = replay.run(self.exe, program, at=200, factor=500)
        self.check_endpoints(half)
        duration = lambda r: sum(row[3] for row in r['segments'])
        self.assertAlmostEqual(duration(half)/duration(plain), 1.875, delta=1.875*.05)
        elapsed = 0
        request = half['requests'][0][0]
        settled = []
        for tick, section, speed, dt, position in half['segments']:
            if tick >= request:
                elapsed += dt*60
                if elapsed > 1.5:
                    settled.append(speed)
        self.assertTrue(settled)
        self.assertLessEqual(max(settled), 309)

    def test_live_change_at_low_jerk_responds_and_settles_without_a_surge(self):
        program = 'G21 G91 G94 G64\n' + 'G1 X20 F600\n'*4
        for jerk in (20, 50):
            with self.subTest(jerk=jerk):
                result = replay.run(self.exe, program, at=200, factor=500, jerk=jerk)
                self.check_endpoints(result)
                request = result['requests'][0][0]
                elapsed, response, settled = 0, None, []
                for tick, section, speed, dt, position in result['segments']:
                    if tick < request:
                        continue
                    elapsed += dt*60
                    if speed < 590 and response is None:
                        response = elapsed
                    if elapsed > 1.5:
                        settled.append(speed)
                self.assertIsNotNone(response)
                self.assertLess(response, 1)
                self.assertTrue(settled)
                self.assertLessEqual(max(settled), 309)
                for first, second in zip(result['blocks'], result['blocks'][1:]):
                    self.assertAlmostEqual(first[4], second[2], places=4)
                self.assertTrue(all(b[3] <= 300.1 for b in result['blocks'][1:]))

    def test_dense_refill_adopts_latest_request_and_keeps_boundaries(self):
        program = 'G21 G91 G94 G64\n' + 'G1 X0.4 F600\n'*200
        result = replay.run(self.exe, program, at=20, factor=500)
        self.check_endpoints(result)
        blocks = result['blocks']
        self.assertEqual(len(blocks), 200)
        for first, second in zip(blocks, blocks[1:]):
            self.assertAlmostEqual(first[4], second[2], places=4)
            self.assertGreater(first[4], 0)
        self.assertTrue(all(block[5] == 500 for block in blocks[32:]))
        self.assertTrue(all(block[3] <= 300.1 for block in blocks[40:]))

    def test_queued_transition_jerk_and_boundary_speeds(self):
        program = 'G21 G91 G94 G64\n' + 'G1 X20 F600\n'*4
        result = replay.run(self.exe, program, at=200, factor=500)
        blocks = result['blocks']
        for first, second in zip(blocks, blocks[1:]):
            self.assertAlmostEqual(first[4], second[2], places=4)
            self.assertGreater(first[4], 0)
        self.assertTrue(all(b[4] <= 300.1 for b in blocks[1:]))
        for block in blocks[1:]:
            _, _, entry, cruise, exit, factor, length, head, body, tail, jerk = block
            self.assertEqual(factor, 500)
            self.assertAlmostEqual(head+body+tail, length, places=4)
            for start, end, distance in ((entry, cruise, head), (cruise, exit, tail)):
                if distance:
                    time = 2*distance/(start+end)
                    peak = (10/(3**.5))*abs(end-start)/(time*time)
                    self.assertLessEqual(peak, jerk*1.001)

    def test_stale_and_consumed_queue_candidates_are_rejected(self):
        program = 'G21 G91 G94 G64\n' + 'G1 X20 F600\n'*4
        plain = replay.run(self.exe, program)
        for mode in (1, 2, 3):
            with self.subTest(mode=mode):
                result = replay.run(self.exe, program, at=20, factor=500, queue_interleave=mode)
                self.assertEqual(result['queue_interleave'], [mode])
                self.check_endpoints(result)
                self.assertEqual([r[:2] for r in result['steps']], [r[:2] for r in plain['steps']])
                if mode != 3:
                    self.assertTrue(all(b[5] == 1000 for b in result['blocks']))
                for first, second in zip(result['blocks'], result['blocks'][1:]):
                    self.assertAlmostEqual(first[4], second[2], places=4)

    def test_live_increase_joins_queued_blocks_at_new_speed(self):
        program = 'G21 G91 G94 G64\n' + 'G1 X20 F600\n'*4
        result = replay.run(self.exe, program, at=20, factor=1500)
        self.check_endpoints(result)
        blocks = result['blocks']
        self.assertEqual(len(blocks), 4)
        self.assertTrue(all(b[5] == 1500 for b in blocks[1:]))
        self.assertAlmostEqual(blocks[1][4], 900, delta=.1)
        self.assertAlmostEqual(blocks[2][4], 900, delta=.1)
        for first, second in zip(blocks, blocks[1:]):
            self.assertAlmostEqual(first[4], second[2], places=4)

    def test_short_queued_blocks_start_ramping_within_one_second(self):
        for length in (.3, .4, .5, .8, 1.0):
            with self.subTest(length=length):
                program = 'G21 G91 G94 G64\n' + f'G1 X{length} F600\n'*100
                result = replay.run(self.exe, program, at=50, factor=500, batch=28)
                self.check_endpoints(result)
                request = result['requests'][0][0]
                ticks = [b[0] for b in result['blocks'] if b[5] != 1000 and b[0] >= request]
                ticks += [t[0] for t in result['taken'] if t[0] >= request]
                self.assertTrue(ticks)
                first = min(ticks)
                delay = sum(r[3]*60 for r in result['segments'] if request <= r[0] < first)
                self.assertLess(delay, 1.0)
                for previous, following in zip(result['blocks'], result['blocks'][1:]):
                    self.assertAlmostEqual(previous[4], following[2], places=4)
                    self.assertGreater(previous[4], 0)

    def test_programmed_feed_is_scaled_before_axis_caps(self):
        result = replay.run(self.exe, 'G21 G90 G94\nG1 X20 F2000\n', initial_factor=500)
        self.check_endpoints(result)
        self.assertAlmostEqual(max(b[3] for b in result['blocks']), 1000, delta=.1)
        self.assertTrue(all(b[5] == 500 for b in result['blocks']))

    def test_rapid_inverse_time_and_rotary_remain_unscaled(self):
        for program in ('G21 G90 G94\nG0 X20\n', 'G21 G90 G93\nG1 X20 F6\n',
                        'G21 G90 G94\nG1 A90 F600\n'):
            with self.subTest(program=program):
                plain = replay.run(self.exe, program)
                half = replay.run(self.exe, program, initial_factor=500)
                self.assertEqual(plain['segments'], half['segments'])
                self.assertEqual(plain['steps'], half['steps'])
                self.assertTrue(all(b[5] == 1000 for b in half['blocks']))
                self.check_endpoints(half)

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
