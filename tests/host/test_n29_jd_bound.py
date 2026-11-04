"""N29: bound junction deviations before squaring them. TinyG configuration wiki xJD and JA define the controlling corner circle. Accept 0.00001 through 1000000 canonical units, keep ordinary junction results unchanged, and prevent an extreme acceleration product from returning infinity."""
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
  row=next(l for l in (FW/'config_app.c').read_text().splitlines() if '"xjd"' in l and '{' in l)
  setter=row.split(',')[6].strip()
  code+='\n#define MAX_JUNCTION_DEVIATION 1000000.0f\n'
  if setter!='set_flu':code+=function(s,rf'stat_t {setter}\(nvObj_t \*nv\)')
  code+='\n#define jd_set '+setter+'\n'
  code+='\nint main(void){for(units=0;units<2;units++)for(int a=0;a<6;a++){\n float bad[]={1e30f,-1,0,1e-38f,NAN,INFINITY};\n for(int k=0;k<6;k++){stored=123;nvObj_t n={.index=a,.value_flt=bad[k]};assert(jd_set(&n)!=0&&stored==123);}\n nvObj_t n={.index=a,.value_flt=.05f};assert(jd_set(&n)==0&&stored>0&&isfinite(stored*stored));\n}units=MILLIMETERS;nvObj_t n={.value_flt=1000000};assert(jd_set(&n)==0);\n}\n'
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/"check.c";p.write_text(code);exe=Path(d)/"check"
   subprocess.run(["cc","-std=gnu99","-O1","-fsanitize=address,undefined,float-cast-overflow",str(p),"-lm","-o",str(exe)],check=True)
   subprocess.run([str(exe)],check=True)

class JunctionMath(unittest.TestCase):
 def test_real_junction_finite_at_setting_bounds(self):
  code=r'''
#include <math.h>
#include <float.h>
#include <assert.h>
#include <stdint.h>
#define AXES 6
enum {AXIS_X,AXIS_Y,AXIS_Z,AXIS_A,AXIS_B,AXIS_C};
#define square(x) ((x)*(x))
static struct{float junction_acceleration;struct{float junction_dev;}a[AXES];}cm;
'''
  code+=function((FW/'plan_line.c').read_text(),r'static float _get_junction_vmax\(const float a_unit\[\], const float b_unit\[\]\)\s*(?=\{)')
  code+=r'''
int main(void){float a[6]={1,0},b[6]={0,1};
 for(int i=0;i<6;i++)cm.a[i].junction_dev=1000000;
 cm.junction_acceleration=FLT_MAX;float v=_get_junction_vmax(a,b);assert(isfinite(v)&&v>0);
 cm.junction_acceleration=200000;for(int i=0;i<6;i++)cm.a[i].junction_dev=.05f;
 float s=sqrtf(.5f);float expected=sqrtf(.05f*s/(1-s)*cm.junction_acceleration);
 assert(fabsf(_get_junction_vmax(a,b)-expected)<.001f);
}
'''
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'check.c';p.write_text(code);exe=p.with_suffix('')
   subprocess.run(['cc','-std=gnu99','-O1','-fsanitize=address,undefined',str(p),'-lm','-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
