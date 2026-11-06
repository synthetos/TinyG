"""N09: invalid junction acceleration must not reach the planner.

TinyG wiki TinyG-Configuration-for-Firmware-Version-0.97, $JA: ja sets the positive acceleration used in
v=sqrt(radius*ja). Tests the configured setter before storage, including inches.
"""
from pathlib import Path
import re, subprocess, tempfile, unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]/'firmware/tinyg'
class JunctionGuard(unittest.TestCase):
    def test_configured_setter(self):
        table=(ROOT/'config_app.c').read_text()
        row=next(l for l in table.splitlines() if '"ja"' in l and 'junction_acceleration' in l)
        setter=re.search(r'get_flt,\s*(\w+)',row).group(1)
        code=r'''
#include <math.h>
#include <stdint.h>
#include <assert.h>
#define STAT_OK 0
#define STAT_INPUT_LESS_THAN_MIN_VALUE 145
#define STAT_INPUT_VALUE_RANGE_ERROR 146
#define MM_PER_INCH 25.4f
#define INCHES 1
#define MODEL 0
#define TYPE_FLOAT 3
#define GET_TABLE_WORD(x) FIELD_##x
#define FIELD_target (&cm.junction_acceleration)
#define FIELD_precision 0
#define _to_millimeters(a) ((cm.gm.units_mode==INCHES)?(a)*MM_PER_INCH:(a))
typedef int stat_t;
typedef struct {float value_flt;int precision,valuetype;} nvObj_t;
struct {struct {int units_mode;} gm;float junction_acceleration;} cm;
int cm_get_units_mode(int which) {return cm.gm.units_mode;}
'''
        code+=function(table,r'stat_t set_flu\(nvObj_t \*nv\)')
        if setter!='set_flu': code+=function((ROOT/'canonical_machine.c').read_text(),r'stat_t '+setter+r'\(nvObj_t \*nv\)')
        code+=r'''
int main(void) {
 float invalid[]={-1,0,-0.0f,NAN,INFINITY,-INFINITY};
 for(int units=0;units<2;units++) {
  cm.gm.units_mode=units;
  for(unsigned i=0;i<sizeof(invalid)/sizeof(*invalid);i++) {
   cm.junction_acceleration=123;nvObj_t n={.value_flt=invalid[i]};
   assert(SETTER(&n)!=STAT_OK);assert(cm.junction_acceleration==123);
  }
  nvObj_t n={.value_flt=10};assert(SETTER(&n)==0);
  assert(cm.junction_acceleration==(units?254.0f:10.0f));
 }
 cm.junction_acceleration=123;nvObj_t n={.value_flt=1e38f};
 assert(SETTER(&n)!=0 && cm.junction_acceleration==123);
}
'''.replace('SETTER',setter)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'check.c';p.write_text(code);exe=Path(d)/'check'
            subprocess.run(['cc','-std=c99','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(p),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
