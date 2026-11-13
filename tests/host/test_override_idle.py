"""Unity motion must not enter the override snapshot/publication/consume guards."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
import override_replay as replay

ROOT = Path(__file__).resolve().parents[2]


class OverrideIdle(unittest.TestCase):
    def test_unity_has_no_guarded_override_work(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            exe = replay.build(ROOT, out)
            p = out / 'plan_exec.c'
            code = p.read_text()
            code = code.replace('#define BODY_LOCK ((void)0)',
                                '#define BODY_LOCK (++guard_calls)')
            code = code.replace('    ov_snapshot(&request);',
                                '    guard_calls++;ov_snapshot(&request);')
            code = code.replace('#define BODY_LOCK uint8_t', '#define BODY_LOCK ++guard_calls; uint8_t')
            code += '\n__attribute__((destructor)) static void check_idle(void){assert(guard_calls==0);}\n'
            code = '#include <assert.h>\nstatic unsigned guard_calls;\n' + code
            p.write_text(code)
            args = json.loads((out / 'command.json').read_text())
            args[args.index('-lm'):args.index('-lm')] = [str(out / (n + '.c'))
                                                      for n in ('plan_exec', 'kinematics', 'step_engine')]
            args.insert(1, '-DF_CPU=32000000UL')
            subprocess.run(args, check=True)
            result = replay.run(exe, 'G21 G90 G94\nG1 X100 F600\nG1 X0\n')
            self.assertTrue(result['ends'])
