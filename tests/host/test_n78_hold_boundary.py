"""N78: a queued block must not reuse the previous block's forward differences on hold."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import replay_hold
import trajectory_replay

ROOT = Path(__file__).resolve().parents[2]
CORPUS = ROOT / 'tests/hil/corpus/n78'


def build(root, out):
    def patch(driver):
        driver = replay_hold._hold_patch(driver)
        for old, new in (
                ('a<3?1200:3600', 'a<3?20000:3600'),
                ('cm_set_axis_jerk(a,500);', 'cm_set_axis_jerk(a,5000);'),
                ('st_cfg.mot[m].motor_map=(m==1?0:m);', 'st_cfg.mot[m].motor_map=m;')):
            assert driver.count(old) == 1, old
            driver = driver.replace(old, new)
        marker = '            stat_t status=mp_exec_move();'
        driver = driver.replace(marker, '''
            if (argc_boundary && !boundary_fired && mr.block_state==BLOCK_IDLE &&
                mb.r->gm.linenum==781) {
                cm_request_feedhold(); cm_feedhold_sequencing_callback(); boundary_fired=1;
                host_acted=1; host_nholds=1; host_hold_act[0]='r'; host_hi=0;
                printf("BOUNDARY %.9g %.9g %.9g\\n", (double)mb.r->entry_velocity,
                       (double)mr.segment_velocity, (double)mr.forward_diff_5);
            }
''' + marker + '''
            if(cm_is_alarmed()!=STAT_OK)exit(127);
''')
        driver = driver.replace('static void take_one(void)',
                                'static int argc_boundary, boundary_fired;\nstatic void take_one(void)')
        driver = driver.replace('    setup();',
                                '    argc_boundary=(argc>3);\n    setup();')
        return driver
    return trajectory_replay.build(root, out, extra_patch=patch)


class HoldBoundary(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix='tinyg-n78-')
        cls.exe = build(ROOT, Path(cls.tmp.name))

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_program(self, name, schedule='', boundary=False):
        args = [str(self.exe), '28', schedule] + (['boundary'] if boundary else [])
        result = subprocess.run(args, input=(CORPUS/name).read_bytes(), capture_output=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr.decode()[-2000:])
        rows = [line.split() for line in result.stdout.decode().splitlines()]
        steps = [r for r in rows if r and r[0]=='STEPS']
        self.assertEqual(len(steps), 4)
        for row in steps:
            self.assertLessEqual(abs(float(row[2])-int(row[3])), 1.5, row)
        final = next(r for r in rows if r and r[0]=='FINAL')
        for model, runtime in zip(map(float, final[2:5]), map(float, final[6:9])):
            self.assertAlmostEqual(model, runtime, places=3)
        return rows

    def test_recorded_tiny_cat_schedule(self):
        rows = self.run_program('tiny-cat-rapid.nc', '560r,4217r,7004r')
        done = next(r for r in rows if r and r[0]=='HOLDS')
        self.assertEqual(done[1:4], ['3', 'of', '3'])

    def test_numbered_cam_block_boundary(self):
        rows = self.run_program('hold-boundary.nc', boundary=True)
        self.assertEqual(sum(r[0]=='BOUNDARY' for r in rows if r), 1)
        done = next(r for r in rows if r and r[0]=='HOLDS')
        self.assertEqual(done[1:4], ['1', 'of', '1'])


if __name__=='__main__':
    unittest.main()
