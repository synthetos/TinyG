"""Link actual planner and dispatch logic; fake only physical execution and the clock."""
from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT=Path(__file__).resolve().parents[2]
FIRMWARE=Path(os.environ.get('TINYG_FIRMWARE',ROOT/'firmware/tinyg'))
FIXTURE=Path(__file__).parent/'planner_startup'


class PlannerStartup(unittest.TestCase):
    def test_lookahead_and_release(self):
        source=(FIRMWARE/'plan_exec.c').read_text()
        dispatch=function(source,r'(?m)^stat_t mp_exec_move\([^;{}\n]*\)\s*')
        cm=(FIRMWARE/'canonical_machine.c').read_text()
        hold=function(cm,r'(?m)^void cm_request_feedhold\([^;{}\n]*\)\s*')
        with tempfile.TemporaryDirectory() as d:
            c=Path(d)/'check.c'
            c.write_text((FIXTURE/'check.c').read_text()+'\nvoid cm_cycle_start(void) { cm.machine_state=MACHINE_CYCLE; cm.cycle_state=CYCLE_MACHINING; }\nstatic struct {bool resume_cancelled;} hold_outputs;\n'+dispatch+'\n'+hold)
            out=Path(d)/'check'
            flags=['-DHAS_STARTUP'] if 'mp_startup_callback' in (FIRMWARE/'planner.c').read_text() else []
            subprocess.run(['cc','-std=gnu99','-O1','-g','-funsigned-char','-fsanitize=address,undefined','-fstrict-flex-arrays=3',
                            '-D_POSIX_C_SOURCE=200809L','-include','tgmath.h','-I'+str(FIXTURE),'-I'+str(FIRMWARE),*flags,str(c),
                            *[str(FIRMWARE/f) for f in ['planner.c','plan_line.c','plan_zoid.c','util.c']],
                            '-lm','-o',str(out)],check=True)
            subprocess.run([str(out)],check=True)


if __name__=='__main__': unittest.main()
