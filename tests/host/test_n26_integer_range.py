"""N26: validate integers before narrowing their stored width. TinyG configuration wiki SL defines soft-limit enable as 0 or 1. Large integer values must not silently disable limits or change another byte setting."""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
FW=Path(__file__).resolve().parents[2]/"firmware/tinyg"
PRE='\n#include <assert.h>\n#include <stdbool.h>\n#include <stdint.h>\n#include <string.h>\n#include <math.h>\n#define STAT_OK 0\n#define STAT_INPUT_LESS_THAN_MIN_VALUE 108\n#define STAT_INPUT_VALUE_RANGE_ERROR 110\n#define STAT_UNRECOGNIZED_NAME 100\n#define TYPE_FLOAT 4\n#define MODEL 0\n#define INCHES 0\n#define MILLIMETERS 1\n#define MM_PER_INCH 25.4f\n#define JERK_MULTIPLIER 1000000.f\n#define JERK_MATCH_PRECISION 1000.f\n#define EPSILON .00001f\n#define _to_millimeters(a) (units==INCHES?(a)*MM_PER_INCH:(a))\n#define PSTR(s) s\n#define min(a,b) ((a)<(b)?(a):(b))\n#define max(a,b) ((a)>(b)?(a):(b))\ntypedef int stat_t;\ntypedef struct {float value_flt;uint32_t value_int;int index,precision,valuetype,depth;} nvObj_t;\nstatic float stored=123;\nstatic int units=MILLIMETERS;\n#define table_target (&stored)\n#define table_precision 0\n#define GET_TABLE_WORD(x) table_##x\nstatic int cm_get_units_mode(int which){return units;}\nstatic int _get_axis_type(int index){return index>=3;}\nstatic int _get_axis(int index){return index;}\nstatic struct {struct {float jerk_max,recip_jerk;}a[6];float chordal_tolerance;}cm;\n'
class Regression(unittest.TestCase):
 def test_real_code(self):
  code=PRE
  code+=function((FW/'config.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) set_ui8\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  code+=function((FW/'config.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) set_u16\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  code+=function((FW/'config.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) set_01\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  row=next(l for l in (FW/'config_app.c').read_text().splitlines() if '"sl"' in l and '{' in l)
  code+='\n#define sl_set '+row.split(',')[6].strip()+'\n'
  code+='\nint main(void){nvObj_t n={0};uint8_t *b=(uint8_t*)&stored;uint16_t *w=(uint16_t*)&stored;\n uint32_t bad[]={256,65536,2147483648UL,UINT32_MAX};\n for(int i=0;i<4;i++){*b=1;n.value_int=bad[i];assert(set_ui8(&n)==110&&*b==1);}\n n.value_int=255;assert(set_ui8(&n)==0&&*b==255);\n for(int i=1;i<4;i++){*w=123;n.value_int=bad[i];assert(set_u16(&n)==110&&*w==123);}\n n.value_int=65535;assert(set_u16(&n)==0&&*w==65535);\n *b=1;n.value_int=2;assert(sl_set(&n)==110&&*b==1);\n n.value_int=0;assert(sl_set(&n)==0&&*b==0);n.value_int=1;assert(sl_set(&n)==0&&*b==1);\n}\n'
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/"check.c";p.write_text(code);exe=Path(d)/"check"
   subprocess.run(["cc","-std=gnu99","-O1","-fsanitize=address,undefined,float-cast-overflow",str(p),"-lm","-o",str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
