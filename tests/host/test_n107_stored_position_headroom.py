"""Dense legal blocks must fit the controller reserve without mid-block refusal."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import corpus_replay

ROOT = Path(__file__).resolve().parents[2]


class StoredPositionHeadroom(unittest.TestCase):
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
        if (!strcmp(argv[1],"low")) reserve--;
        if (!strcmp(argv[1],"direct-one")) reserve=1;
        if (!strcmp(argv[1],"direct-two")) reserve=2;
        while(mp_get_planner_buffers_available()>reserve) {
            if(cm_dwell(2)!=STAT_OK)abort();
        }
        unsigned before=mp_get_planner_buffers_available();
        __typeof__(cm.gm) model=cm.gm;
        __typeof__(cm.gmx) extended=cm.gmx;
        stat_t result;
        if(reserve<=2) {
            float target[AXES]={1};bool flags[AXES]={true};
            result=cm_goto_g28_position(target,flags);
        } else {
            char block[128];
            snprintf(block,sizeof block,"S100 T1 M6 M3 M7 M8 G49 G54 %s X1 M30",
                     argc>2?argv[2]:"G28");
            result=gc_gcode_parser(block);
        }
        printf("RESERVE %u STATUS %u USED %u FINAL %g UNCHANGED %u\n",before,result,
               before-mp_get_planner_buffers_available(),(double)cm.gmx.position[0],
               !memcmp(&model,&cm.gm,sizeof model) && !memcmp(&extended,&cm.gmx,sizeof extended));
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
        return int(fields[1]), int(fields[3]), int(fields[5]), float(fields[7]), int(fields[9])

    def test_dense_stored_moves_fit_the_dispatch_reserve(self):
        for stored in ('G28','G30'):
            with self.subTest(stored=stored):
                self.assertEqual(self.result('dense',stored)[:4], (12,0,9,2))

    def test_ungated_line_is_refused_before_any_command_or_model_change(self):
        for stored in ('G28','G30'):
            with self.subTest(stored=stored):
                self.assertEqual(self.result('low',stored), (11,13,0,0,1))

    def test_direct_canonical_call_needs_only_its_two_legs(self):
        self.assertEqual(self.result('direct-one'), (1,13,0,0,1))
        self.assertEqual(self.result('direct-two')[:4], (2,0,2,2))
