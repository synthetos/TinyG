"""Dense legal blocks must fit the controller reserve without mid-block refusal."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import corpus_replay

ROOT = Path(__file__).resolve().parents[2]


class OverrideHeadroom(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.work = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.work.cleanup)
        out = Path(cls.work.name)
        cls.exe = corpus_replay.build(ROOT, out)
        driver = out/'driver.c'
        source = driver.read_text()
        probe = r'''
    if (argc > 1) {
        cm.gmx.g28_position[0]=2;cm.gmx.g30_position[0]=2;
        unsigned reserve=PLANNER_BUFFER_HEADROOM;
        if (!strcmp(argv[1],"short")) reserve=1;
        if (!strcmp(argv[1],"two")) reserve=2;
        while(mp_get_planner_buffers_available()>reserve) {
            if(cm_dwell(2)!=STAT_OK)abort();
        }
        unsigned before=mp_get_planner_buffers_available();
        char block[128];
        if (reserve<=2) strcpy(block,"G28 X1");
        else snprintf(block,sizeof block,"S100 T1 M6 M3 M7 M8 %s G49 G54 %s X1 M30",
                      argv[1],argc>2?argv[2]:"G28");
        float target[AXES]={1,0,0,0,0,0};
        bool flags[AXES]={true,false,false,false,false,false};
        // Direct helper admission is separate from the parser's full reserve.
        stat_t result=reserve<=2 ? cm_goto_g28_position(target,flags) : gc_gcode_parser(block);
        printf("RESERVE %u STATUS %u USED %u FINAL %g\n",before,result,
               before-mp_get_planner_buffers_available(),(double)cm.gmx.position[0]);
        return 0;
    }
'''
        source = source.replace('    setup();', '    setup();\n'+probe)
        driver.write_text(source)
        command = json.loads((out/'command.json').read_text())
        built = subprocess.run(command, capture_output=True, text=True)
        if built.returncode:
            raise AssertionError(built.stderr)

    def result(self, *args):
        run = subprocess.run([str(self.exe), *args], capture_output=True, text=True, check=True)
        line = next(line for line in run.stdout.splitlines() if line.startswith('RESERVE '))
        fields = line.split()
        return int(fields[1]), int(fields[3]), int(fields[5]), float(fields[7])

    def test_dense_permission_and_two_moves_fit_twelve_slots(self):
        for permission in ('M48', 'M49', 'M50 P0', 'M50 P1'):
            for stored in ('G28', 'G30'):
                with self.subTest(permission=permission, stored=stored):
                    self.assertEqual(self.result(permission, stored), (12, 0, 10, 2))

    def test_same_dense_block_without_permission_also_fits(self):
        self.assertEqual(self.result(''), (12, 0, 9, 2))

    def test_direct_stored_move_needs_two_slots_and_refuses_before_either_leg(self):
        self.assertEqual(self.result('short'), (1, 13, 0, 0))
        self.assertEqual(self.result('two'), (2, 0, 2, 2))
