"""Real JSON dispatch and group handlers keep reads usable during lockout."""
from pathlib import Path
from unittest.mock import patch
import tempfile
import unittest

import input_harness as harness
from source_extract import function


class NestedGetAlarm(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.work = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.work.cleanup)
        out=Path(cls.work.name)
        fixture=(harness.FIXTURES/'json_prelude.h').read_text()
        fixture=fixture.replace('"sr", "js"};','"sr", "js", "x", "rt", "rtmfo", "rtmfe", "rtmfc", "rtmcp", "rtovb", "rtov"};')
        fixture=fixture.replace('TYPE_TXTCON, TYPE_INTEGER, TYPE_NULL, TYPE_INTEGER};',
                                'TYPE_TXTCON, TYPE_INTEGER, TYPE_NULL, TYPE_INTEGER, TYPE_NULL, TYPE_NULL, TYPE_FLOAT, TYPE_INTEGER, TYPE_FLOAT, TYPE_BOOLEAN, TYPE_INTEGER, TYPE_INTEGER};')
        fixture=fixture.replace('return !strcmp(t,"1");','return !strcmp(t,"1") || !strcmp(t,"x") || !strcmp(t,"rt");')
        fixture=fixture.replace('static fptrCmd table_set(index_t i) { return i ? capture_set : set_nul; }',
                                'stat_t rt_set_grp(nvObj_t *);\nstatic fptrCmd table_set(index_t i) { return i==3 || i==10 ? set_grp : i==11 ? rt_set_grp : i ? capture_set : set_nul; }')
        fixture=fixture.replace('bool allowed, fl_taken;', 'bool allowed, fl_taken, check_only;')
        fixture=fixture.replace('static stat_t rt_set_grp(nvObj_t *n) { (void)n; return STAT_OK; }',
            function((harness.FW/'runtime.c').read_text(), r'stat_t rt_set_grp\('))
        fixture=fixture.replace('stat_t set_grp(nvObj_t *n) { (void)n; return STAT_OK; }',
            'static bool _refused_in_cycle(nvObj_t *nv);\ncontroller_t cs;\n'+
            function((harness.FW/'config.c').read_text(),r'stat_t set_grp\('))
        fixture=fixture.replace('static void reset(void) {','static void reset(void) {\n    cs.comm_mode=JSON_MODE;')
        (out/'json_prelude.h').write_text(fixture)
        with patch.object(harness,'FIXTURES',out):
            cls.exe=harness.build('json',out)

    def rows(self, lines, state):
        result=harness.run(self.exe, [line.encode() for line in lines], state=state)
        self.assertEqual(result.returncode,0,result.stderr)
        return [list(map(int,row.split()[:4])) for row in result.stdout.splitlines()]

    def test_group_reads_in_alarm_shutdown_and_panic(self):
        for state in ('alarm','shutdown','panic'):
            with self.subTest(state=state):
                self.assertEqual(self.rows(['{"x":{"vm":null}}','{"1":{"mi":null}}',
                    '{"rt":{"mfo":null,"mfe":null,"mfc":null,"mcp":null}}',
                    '{"rt":{"ovb":null}}','{"rt":{"ov":null}}'],state),
                    [[0,1,1,0],[0,1,1,0],[0,4,4,0],[0,1,1,0],[0,1,1,0]])

    def test_writes_and_mixed_groups_refuse_before_any_child_executes(self):
        for state, code in [('alarm',204),('shutdown',205),('panic',206)]:
            lines=['{"rt":{"mfo":0.5}}','{"rt":{"mfc":null,"mfo":0.5}}',
                   '{"rt":{"mfo":0.5,"mfc":null}}','{"x":{"vm":600}}',
                   '{"sr":{"xvm":null}}']
            with self.subTest(state=state):
                self.assertEqual(self.rows(lines,state),[[code,0,0,0]]*len(lines))

    def test_empty_parent_is_not_a_read_and_idle_groups_still_work(self):
        self.assertEqual(self.rows(['{"rt":{}}'],'alarm')[0][0],111)
        self.assertEqual(self.rows(['{"x":{"vm":null}}','{"rt":{"mfo":null}}'],'ready'),
                         [[0,1,1,0],[0,1,1,0]])
