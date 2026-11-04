"""N19: chordal tolerance is a positive geometric error bound. TinyG wiki TinyG-Configuration-for-Firmware-Version-0.97, CT; NIST RS274NGC v3 section 3.5.3 permits full circles. Negative zero must not discard a circle (plan_arc.c segment calculation)."""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
FW=Path(__file__).resolve().parents[2]/"firmware/tinyg"
PRE='\n#include <assert.h>\n#include <stdbool.h>\n#include <stdint.h>\n#include <string.h>\n#include <math.h>\n#define STAT_OK 0\n#define STAT_INPUT_LESS_THAN_MIN_VALUE 108\n#define STAT_INPUT_VALUE_RANGE_ERROR 110\n#define STAT_UNRECOGNIZED_NAME 100\n#define TYPE_FLOAT 4\n#define MODEL 0\n#define INCHES 0\n#define MILLIMETERS 1\n#define MM_PER_INCH 25.4f\n#define JERK_MULTIPLIER 1000000.f\n#define JERK_MATCH_PRECISION 1000.f\n#define EPSILON .00001f\n#define _to_millimeters(a) (units==INCHES?(a)*MM_PER_INCH:(a))\n#define PSTR(s) s\n#define min(a,b) ((a)<(b)?(a):(b))\n#define max(a,b) ((a)>(b)?(a):(b))\ntypedef int stat_t;\ntypedef struct {float value_flt;uint32_t value_int;int index,precision,valuetype,depth;} nvObj_t;\nstatic float stored=123;\nstatic int units=MILLIMETERS;\n#define table_target (&stored)\n#define table_precision 0\n#define GET_TABLE_WORD(x) table_##x\nstatic int cm_get_units_mode(int which){return units;}\nstatic int _get_axis_type(int index){return index>=3;}\nstatic int _get_axis(int index){return index;}\nstatic struct {struct {float jerk_max,recip_jerk;}a[6];float chordal_tolerance;}cm;\n'
class Regression(unittest.TestCase):
 def test_real_code(self):
  code=PRE
  code+=function((FW/'config.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) set_flt\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  code+=function((FW/'config_app.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) set_flu\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  import re
  s=(FW/'canonical_machine.c').read_text()
  row=next(x for x in (FW/'config_app.c').read_text().splitlines() if '"ct"' in x and '{' in x)
  setter=row.split(',')[6].strip()
  if setter!='set_flu':code+=function(s,rf'stat_t {setter}\(nvObj_t \*nv\)')
  code+='\n#define ct_set '+setter+'\n'
  code+='\nint main(void){\n for(units=0;units<2;units++){\n  float bad[]={-1,0,-0.f,NAN,INFINITY};\n  for(int i=0;i<5;i++){stored=123;nvObj_t n={.value_flt=bad[i]};assert(ct_set(&n)!=0);assert(stored==123);}\n  nvObj_t n={.value_flt=.01f};assert(ct_set(&n)==0);assert(stored>0&&isfinite(stored));\n }\n}\n'
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/"check.c";p.write_text(code);exe=Path(d)/"check"
   subprocess.run(["cc","-std=gnu99","-O1","-fsanitize=address,undefined,float-cast-overflow",str(p),"-lm","-o",str(exe)],check=True)
   subprocess.run([str(exe)],check=True)

from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]
FW=ROOT/'firmware/tinyg'
FIX=Path(__file__).parent/'planner_startup'

class ArcGuard(unittest.TestCase):
    def test_bad_tolerance_before_arc_calculation(self):
        pre=(FIX/'check.c').read_text().split('static void exec_pending')[0]
        pre=pre.replace('void cm_cycle_end(void) {}', 'void cm_cycle_end(void) { cm.cycle_state=CYCLE_OFF; cm.machine_state=MACHINE_PROGRAM_STOP; }')
        pre += '''
#include "util.h"
void cm_cycle_start(void) { cm.cycle_state=CYCLE_MACHINING; cm.machine_state=MACHINE_CYCLE; }
stat_t cm_alarm_P(stat_t s,const char *msg) { cm.machine_state=MACHINE_ALARM; return s; }
void cm_request_feedhold(void) {}
arc_t arc;
'''
        source=(FW/'plan_arc.c').read_text()
        for sig in [r'static void _compute_arc_offsets_from_radius\(\)\s*',r'static float _estimate_arc_time \(float arc_time\)\s*',r'static stat_t _compute_arc\(const bool radius_f\)\s*',r'stat_t cm_arc_callback\(\)\s*']:
            pre+=function(source,sig + r"(?=\{)")
        pre += r'''
int main(void){
 float bad[]={0,-0.f,-1,NAN,INFINITY};
 for(int i=0;i<5;i++){
  setup();memset(&arc,0,sizeof arc);cm.chordal_tolerance=bad[i];
  arc.plane_axis_0=0;arc.plane_axis_1=1;arc.linear_axis=2;arc.offset[0]=5;
  arc.gm.motion_mode=MOTION_MODE_CW_ARC;arc.gm.feed_rate=100;
  assert(_compute_arc(false)==STAT_INPUT_VALUE_RANGE_ERROR);
  assert(!mp_has_runnable_buffer());
 }
 setup();cm.chordal_tolerance=.01;arc.offset[0]=5;assert(_compute_arc(false)==STAT_OK);
}
'''
        with tempfile.TemporaryDirectory() as d:
            c=Path(d)/'arc.c'; c.write_text(pre); exe=Path(d)/'arc'
            subprocess.run(['cc','-std=gnu99','-O1','-g','-funsigned-char','-fsanitize=address,undefined',
                            '-D_POSIX_C_SOURCE=200809L','-include','tgmath.h','-I'+str(FIX),'-I'+str(FW),str(c),
                            *[str(FW/f) for f in ['planner.c','plan_line.c','plan_zoid.c','util.c']],'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
