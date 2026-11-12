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
                    self.assertLessEqual(abs(target - pulses), 1, motor)
                self.assertTrue(all(math.isfinite(x) for row in result['segments'] for x in row))
        self.assertEqual(len(self.increase['requests']), 1)
        self.assertEqual(self.increase['requests'][0][1:3], (1, 1500))
        self.assertEqual([r[:2] for r in self.increase['steps']],
                         [r[:2] for r in self.plain['steps']])

    def assert_consistent_endpoints(self,result):
        self.assertTrue(result['ends'])
        previous={}
        for move,motor,target,pulses,encoder,position in result['ends']:
            with self.subTest(move=move,motor=motor):
                self.assertLessEqual(abs(target-pulses),1)
                old_target,old_pulses=previous.get(motor,(0,0))
                self.assertLessEqual(abs((pulses-old_pulses)-(target-old_target)),1)
                previous[motor]=(target,pulses)
                self.assertEqual(encoder,pulses)
                self.assertLessEqual(abs(position-pulses),1)
                self.assertAlmostEqual(position,target,places=4)

    def test_body_pulses_are_within_one_step_of_target(self):
        fixed=[replay.run(self.exe,PROGRAM,factor=f,schedule=2) for f in (500,1500)]
        for result in (self.plain,self.runs[1],self.increase,*fixed):
            self.assert_consistent_endpoints(result)

    def test_many_changes_do_not_accumulate_position_error(self):
        program='G21 G90 G94\n'+''.join(
            f'N{i+10} G1 X{100 if i%2==0 else 0} F600\n' for i in range(50))
        for scheduled in (False,True):
            result=replay.run(self.exe,program,schedule=scheduled)
            self.assertEqual(len(result['ends']),50*4)
            self.assert_consistent_endpoints(result)
            for move,motor,target,*_ in result['ends']:
                self.assertEqual(target,128000 if motor<2 and move%2 else 0)
            for _,target,pulses in result['steps']:
                self.assertEqual(target,0)
                self.assertLessEqual(abs(pulses),1)
            if scheduled:
                self.assertGreaterEqual(len(result['changes']),150)
                self.assertEqual(sum(tick==0 for _,tick,_ in result['changes']),50)
                self.assertGreaterEqual(len(result['taken']),100)

    def test_committed_factor_and_cap_follow_the_accepted_plan(self):
        self.assertTrue(self.runs[1]['committed'])
        self.assertTrue(all(row[1:]==(500,0) for row in self.runs[1]['committed']))
        self.assertTrue(all(row[1:]==(1500,0) for row in self.increase['committed']))
        capped=replay.run(self.exe,PROGRAM,at=20,factor=1500,section=1,max_feed=800)
        self.assertTrue(capped['committed'])
        self.assertTrue(all(row[1:]==(1500,1) for row in capped['committed']))
        self._assert_body_response(capped,800)
        self.assert_consistent_endpoints(capped)

    def test_jerk_through_body_transition(self):
        for result in (self.runs[1],self.increase):
            rows=[x for x in result['segments'] if x[0]>result['requests'][0][0]+30 and x[2]>0]
            accel=[((b[2]-a[2])/((a[3]+b[3])/2),(a[3]+b[3])/2)
                   for a,b in zip(rows,rows[1:])]
            jerk=[abs(2*(b[0]-a[0])/(a[1]+b[1])) for a,b in zip(accel,accel[1:])]
            self.assertLessEqual(max(jerk),500e6*1.005)

    def test_new_request_and_cancel_during_planning_reject_old_plan(self):
        for mode in (1,3):
            result=replay.run(self.exe,PROGRAM,at=20,factor=500,section=1,interleave=mode)
            self.assertEqual(len(result['interleave']),1)
            self.assertEqual(result['taken'],[])
            self.assertEqual(result['steps'],self.plain['steps'])

    def test_stop_requests_cancel_private_plan(self):
        for mode in (4,5,6,7):
            with self.subTest(mode=mode):
                result=replay.run(self.exe,PROGRAM,at=20,factor=500,section=1,interleave=mode)
                self.assertEqual(len(result['interleave']),1)
                self.assertEqual(result['taken'],[])
                self.assertEqual(len(result['stops']),1)
                if mode==4:
                    self.assertEqual([r[:2] for r in result['steps']], [r[:2] for r in self.plain['steps']])
                    for _,target,pulses in result['steps']:
                        self.assertLessEqual(abs(target-pulses),1)
                else:
                    self.assertLess(result['steps'][0][2], self.plain['steps'][0][2])

    def test_hold_after_handoff_keeps_endpoint_and_requested_speed_on_resume(self):
        result=replay.run(self.exe,PROGRAM,at=20,factor=500,section=1,interleave=9)
        self.assertEqual(len(result['stops']),1)
        self.assert_consistent_endpoints(result)
        stopped=result['stops'][0][0]
        samples=[x[2] for x in result['segments'] if x[0]>stopped+100 and x[1]==1]
        self.assertTrue(samples)
        self.assertTrue(any(abs(v-300)<.5 for v in samples), (min(samples),max(samples)))
        self.assertEqual([r[:2] for r in result['steps']], [r[:2] for r in self.plain['steps']])
        for _,target,pulses in result['steps']:
            self.assertLessEqual(abs(target-pulses),1)

    def test_completed_block_rejects_private_plan(self):
        program=PROGRAM+'N20 G1 X200 F600\n'
        result=replay.run(self.exe,program,at=20,factor=500,section=1,interleave=8)
        plain=replay.run(self.exe,program)
        self.assertEqual(len(result['interleave']),1)
        self.assertTrue(result['taken'])
        self.assertGreater(result['taken'][0][0], result['interleave'][0][0]+30)
        self.assertEqual([r[:2] for r in result['steps']], [r[:2] for r in plain['steps']])
        for _,target,pulses in result['steps']:
            self.assertLessEqual(abs(target-pulses),1)

    def test_expired_future_boundary_is_not_applied_late(self):
        result=replay.run(self.exe,PROGRAM,at=20,factor=500,section=1,interleave=2)
        self.assertEqual(len(result['interleave']),1)
        self.assertTrue(result['taken'])
        self.assertGreater(result['taken'][0][0],result['interleave'][0][0]+30)
        self._assert_body_response(result,300)

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
