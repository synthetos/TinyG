"""Long snapshot and plan copies must permit the DDA interrupt."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
import override_replay as replay

ROOT = Path(__file__).resolve().parents[2]


class InterruptibleOverride(unittest.TestCase):
    def test_long_operations_are_outside_the_guard(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            exe = replay.build(ROOT, out)
            p = out / 'plan_exec.c'
            code = p.read_text()
            code = '#include <assert.h>\nstatic unsigned masked;\n' + code
            code = code.replace('#define BODY_LOCK uint8_t', '#define BODY_LOCK masked++; uint8_t')
            code = code.replace('#define BODY_UNLOCK SREG = body_sreg', '#define BODY_UNLOCK masked--; SREG = body_sreg')
            code = code.replace('#define BODY_LOCK ((void)0)', '#define BODY_LOCK masked++')
            code = code.replace('#define BODY_UNLOCK ((void)0)', '#define BODY_UNLOCK masked--')
            for anchor in ('    mr.head_length = ov_body.plan.head;',
                           '        ovBoundary_t *b = &ov_body.ticket.boundary;',
                           '    float segment_length = ov_body.ticket.boundary.velocity * ov_body.segment_time;'):
                self.assertEqual(code.count(anchor), 1)
                code = code.replace(anchor, '    assert(!masked);\n' + anchor)
            p.write_text(code)
            args = json.loads((out / 'command.json').read_text())
            args[args.index('-lm'):args.index('-lm')] = [str(out / (n + '.c'))
                                                      for n in ('plan_exec', 'kinematics', 'step_engine')]
            args.insert(1, '-DF_CPU=32000000UL')
            subprocess.run(args, check=True, capture_output=True)
            for factor in (500, 1500):
                row = replay.run(exe, 'G21 G90 G94\nG1 X100 F600\n', at=20, factor=factor, section=1)
                self.assertTrue(row['taken'])
                self.assertTrue(row['ends'])

    def test_exec_during_snapshot_discards_the_copy(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            exe = replay.build(ROOT, out)
            p = out / 'plan_exec.c'
            code = p.read_text()
            anchor = '        ov_ticket_stamp(&ov_body.ticket, ov_segment_ordinal);'
            self.assertEqual(code.count(anchor), 1)
            code = code.replace(anchor, '        extern void host_override_interleave(void);host_override_interleave();\n' + anchor)
            check = 'if (!ov_runtime_snapshot_valid || ov_body.ticket.request != request.request) return STAT_NOOP;'
            self.assertEqual(code.count(check), 1)
            code = code.replace(check, 'if (!ov_runtime_snapshot_valid || ov_body.ticket.request != request.request) { puts("SNAPSHOT_DISCARDED");return STAT_NOOP; }')
            p.write_text(code)
            self.rebuild(out)
            proc = subprocess.run([str(exe), '28', '20', '500', '1', '2', '0', '1200'],
                                  input='G21 G90 G94\nG1 X100 F600\n', text=True,
                                  capture_output=True, check=True, timeout=60)
            self.assertIn('SNAPSHOT_DISCARDED', proc.stdout)
            self.assertIn('OVEND', proc.stdout)

    def test_stop_before_and_after_claim_keeps_stop_priority(self):
        for after_claim in (False, True):
            with self.subTest(after_claim=after_claim), tempfile.TemporaryDirectory() as tmp:
                out = Path(tmp)
                exe = replay.build(ROOT, out)
                p = out / 'plan_exec.c'
                code = p.read_text()
                hook = '    extern void host_override_interleave(void); host_override_interleave();\n'
                self.assertEqual(code.count(hook), 1)
                code = code.replace(hook, '')
                anchor = '    mr.head_length = ov_body.plan.head;' if after_claim else '    bool claimed;'
                self.assertEqual(code.count(anchor), 1)
                code = code.replace(anchor, hook + anchor)
                p.write_text(code)
                self.rebuild(out)
                for stop in (5, 6, 7):  # hold/flush, kill, alarm
                    with self.subTest(stop=stop):
                        row = replay.run(exe, 'G21 G90 G94\nG1 X100 F600\n', at=20, factor=500,
                                         section=1, interleave=stop)
                        self.assertTrue(row['stops'])
                        self.assertEqual(bool(row['taken']), after_claim)
                        if after_claim and stop == 6:
                            self.assertEqual(row['committed'][-1][1], 1000)

    @staticmethod
    def rebuild(out):
        args = json.loads((out / 'command.json').read_text())
        args[args.index('-lm'):args.index('-lm')] = [str(out / (n + '.c'))
                                                  for n in ('plan_exec', 'kinematics', 'step_engine')]
        args.insert(1, '-DF_CPU=32000000UL')
        subprocess.run(args, check=True, capture_output=True)
