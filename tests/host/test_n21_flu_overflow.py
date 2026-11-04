"""N21: finite input must remain finite in canonical units and expanded jerk. TinyG wiki TinyG-Configuration-for-Firmware-Version-0.97, units conversion and xJM/xJH million-unit notation; set_flu and jerk setters must validate before storage."""
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
  code+=function((FW/'canonical_machine.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) cm_set_axis_jerk\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  code+=function((FW/'canonical_machine.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) cm_set_xjm\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  code+=function((FW/'canonical_machine.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) cm_set_xjh\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  code+='\nint main(void){\n units=INCHES;stored=123;nvObj_t n={.value_flt=1e38f};assert(set_flu(&n)==110&&stored==123&&isfinite(n.value_flt));\n for(int a=0;a<3;a++)for(int k=0;k<2;k++){stored=123;n=(nvObj_t){.index=a,.value_flt=1e38f};assert((k?cm_set_xjh(&n):cm_set_xjm(&n))==110&&stored==123);}\n units=MILLIMETERS;n.value_flt=1e30f;assert(set_flu(&n)==0&&isfinite(stored));\n units=INCHES;n.index=0;n.value_flt=20;assert(cm_set_xjm(&n)==0&&isfinite(cm.a[0].recip_jerk)&&cm.a[0].recip_jerk>0);\n float bad[]={NAN,INFINITY,-INFINITY};for(int i=0;i<3;i++){stored=123;n.value_flt=bad[i];assert(set_flu(&n)==110&&stored==123);}\n}\n'
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/"check.c";p.write_text(code);exe=Path(d)/"check"
   subprocess.run(["cc","-std=gnu99","-O1","-fsanitize=address,undefined,float-cast-overflow",str(p),"-lm","-o",str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
