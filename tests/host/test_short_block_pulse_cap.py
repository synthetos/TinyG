"""N12 residual found during N78 replication: short profiles discard their rate cap.

The minimized valid CAM chain panics without a hold. It is a separate confirmed
counterexample; it does not reproduce the original N78 hold-only report.
"""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import step_replay
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / 'firmware/tinyg'
FIX = ROOT / 'tests/host/planner_startup'


class ShortBlockPulseCap(unittest.TestCase):
    def test_short_trapezoids_respect_their_own_cruise_cap(self):
        source = (FW / 'plan_zoid.c').read_text(encoding='latin1')
        code = r'''
#include <assert.h>
#include "tinyg.h"
#include "config.h"
#include "planner.h"
#include "util.h"
#define GET_VELOCITY_ITERATIONS 0
#define MIN_HEAD_LENGTH (MIN_SEGMENT_TIME_PLUS_MARGIN * (bf->cruise_velocity + bf->entry_velocity))
#define MIN_TAIL_LENGTH (MIN_SEGMENT_TIME_PLUS_MARGIN * (bf->cruise_velocity + bf->exit_velocity))
#define MIN_BODY_LENGTH (MIN_SEGMENT_TIME_PLUS_MARGIN * bf->cruise_velocity)
'''
        for name in ('_target_length', '_target_velocity', 'mp_calculate_trapezoid'):
            code += function(source, r'(?:static )?(?:float|void) ' + name + r'\([^;{}]*\)(?=\s*\{)') + '\n'
        code += r'''
static void check(float length,float entry,float exit,float cap,float delta) {
    mpTrapezoid_t b={.length=length,.entry_velocity=entry,.exit_velocity=exit,
        .cruise_velocity=cap,.cruise_vmax=cap,.delta_vmax=delta,
        .jerk=5e9f,.recip_jerk=2e-10f,.cbrt_jerk=1709.97595f};
    mp_calculate_trapezoid(&b);
    assert(b.length==length);
    assert(isfinite(b.cruise_velocity) && b.cruise_velocity>0 && b.cruise_velocity<=cap);
    assert(b.exit_velocity>=0 && b.exit_velocity<=cap);
    assert(b.head_length==0 && b.tail_length==0 && b.body_length==length);
}
int main(void) {
    check(.1f,2500,2000,2100,500);       /* short body inherits too-fast entry */
    check(.2f,5000,5000,2100,500);       /* force-fit must retain the hard cap too */
    check(.0008f,0,20,30,1000);         /* from rest, delta/2 is not a cruise cap */
    check(.1f,1500,1500,2100,500);       /* ordinary short body remains unchanged */
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            src, exe = Path(tmp) / 'profile.c', Path(tmp) / 'profile'
            src.write_text(code)
            subprocess.run(['cc', '-std=gnu99', '-O1', '-Wall', '-Werror',
                            '-D_POSIX_C_SOURCE=200809L',
                            '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                            '-fstrict-flex-arrays=3', '-I' + str(FIX), '-I' + str(FW),
                            str(src), '-lm', '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_real_exec_completes_the_reduced_chain_without_dda_panic(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            exe = step_replay.build(ROOT, out)
            driver = (out / 'driver.c').read_text()
            for old, new in (
                    ('a<3?1200:3600', 'a<3?20000:3600'),
                    ('cm_set_axis_jerk(a,500);', 'cm_set_axis_jerk(a,5000);'),
                    ('st_cfg.mot[m].motor_map=(m==1?0:m);', 'st_cfg.mot[m].motor_map=m;')):
                self.assertEqual(driver.count(old), 1, old)
                driver = driver.replace(old, new)
            marker = '            stat_t status=mp_exec_move();'
            self.assertEqual(driver.count(marker), 1)
            driver = driver.replace(marker, marker + '\n'
                                    '            if(cm_is_alarmed()!=STAT_OK)exit(127);')
            (out / 'driver.c').write_text(driver)
            args = json.loads((out / 'command.json').read_text())
            args[args.index('-lm'):args.index('-lm')] = [str(out / (n + '.c'))
                                                         for n in ('plan_exec', 'kinematics', 'step_engine')]
            args.insert(1, '-DF_CPU=32000000UL')
            subprocess.run(args, check=True, capture_output=True, text=True)
            program = (ROOT / 'tests/hil/corpus/n12/short-block-pulse-cap.nc').read_bytes()
            for batch in (4, 16, 28):
                with self.subTest(batch=batch):
                    result = subprocess.run([str(exe), str(batch)], input=program,
                                            capture_output=True, timeout=20)
                    self.assertEqual(result.returncode, 0, result.stderr.decode())
                    rows = [line.split() for line in result.stdout.decode().splitlines()]
                    rejected = [r for r in rows if r and r[0] == 'LINE' and r[2] not in ('0', '3')]
                    self.assertFalse(rejected)
                    steps = [r for r in rows if r and r[0] == 'STEPS']
                    self.assertEqual(len(steps), 4)
                    for row in steps:
                        self.assertLessEqual(abs(float(row[2]) - int(row[3])), 1.5, row)
                    self.assertAlmostEqual(float(steps[0][2]) / 1280, 1.9951, places=4)
                    self.assertAlmostEqual(float(steps[1][2]) / 1280, 1.8282, places=4)


if __name__ == '__main__':
    unittest.main()
