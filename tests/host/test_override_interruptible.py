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
