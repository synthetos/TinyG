"""N25: rotary radius must keep the linear-to-degree scale finite and nonzero. TinyG configuration wiki aRA/Radius mode: a circumference maps to 360 degrees. Limit radius to 1000000 mm (1 km); even one mm then exceeds the planner numeric-zero tolerance in degrees. Validate after inch conversion and before storage."""
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
  s=(FW/'canonical_machine.c').read_text()
  row=next(l for l in (FW/'config_app.c').read_text().splitlines() if '"ara"' in l and '{' in l)
  setter=row.split(',')[6].strip()
  code+='\n#define MAX_ROTARY_RADIUS 1000000.0f\n'
  if setter!='set_flu':code+=function(s,rf'stat_t {setter}\(nvObj_t \*nv\)')
  code+='\n#define radius_set '+setter+'\n'
  code+='\nint main(void){for(units=0;units<2;units++){\n float bad[]={1e38f,1e30f,1e-40f,0,-1,NAN,INFINITY};\n for(int k=0;k<7;k++){stored=123;nvObj_t n={.value_flt=bad[k]};assert(radius_set(&n)!=0&&stored==123);}\n nvObj_t n={.value_flt=10};assert(radius_set(&n)==0);float scale=360.f/(2.f*3.14159265f*stored);assert(isfinite(scale)&&scale>0);\n}units=MILLIMETERS;nvObj_t n={.value_flt=1000000};assert(radius_set(&n)==0);assert(360.f/(2.f*3.14159265f*stored)>EPSILON);\n n.value_flt=1000001;assert(radius_set(&n)==110&&stored==1000000);\n}\n'
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/"check.c";p.write_text(code);exe=Path(d)/"check"
   subprocess.run(["cc","-std=gnu99","-O1","-fsanitize=address,undefined,float-cast-overflow",str(p),"-lm","-o",str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
