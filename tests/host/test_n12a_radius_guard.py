"""N12a: invalid rotary radius must not reach coordinate conversion.

TinyG wiki TinyG-Configuration-for-Firmware-Version-0.97, Radius mode: linear travel
is converted to degrees using the configured radius. Tests the configured setter before storage, including inches.
"""
from pathlib import Path
import re, subprocess, tempfile, unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]/'firmware/tinyg'
class RadiusGuard(unittest.TestCase):
    def test_invalid_radius_cannot_convert_target(self):
        src=(ROOT/'canonical_machine.c').read_text()
        code=r'''
#include <math.h>
#include <assert.h>
#include <string.h>
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "util.h"
cmSingleton_t cm;
#define copy_vector(a,b) memcpy(a,b,sizeof(float)*AXES)
#define _to_millimeters(x) (x)
float cm_get_active_coord_offset(uint8_t axis){return 0;}
'''
        code+=function(src,r'static float _calc_ABC\(')
        code+=function(src,r'(?:void|stat_t) cm_set_model_target\(')
        ret='return cm_set_model_target(t,f);' if 'stat_t cm_set_model_target(' in src else 'cm_set_model_target(t,f);return STAT_OK;'
        code+='\nstatic stat_t convert(float*t,bool*f){'+ret+'}\n'
        code+=r'''
int main(void){
 float t[AXES]={5,0,0,1,0,0};bool f[AXES]={true,false,false,true,false,false};
 cm.a[0].axis_mode=AXIS_STANDARD;cm.a[3].axis_mode=AXIS_RADIUS;
 cm.gm.distance_mode=INCREMENTAL_MODE;cm.gmx.position[0]=2;cm.gmx.position[3]=3;
 float bad[]={0,-1,NAN,INFINITY};
 for(int n=0;n<4;n++){
  cm.a[3].radius=bad[n];assert(convert(t,f)!=STAT_OK);
  assert(cm.gm.target[0]==2 && cm.gm.target[3]==3);
  assert(!isfinite(_calc_ABC(3,t))); /* invalid values never reach a divide */
 }
 cm.a[3].radius=10;assert(convert(t,f)==STAT_OK);
 assert(cm.gm.target[0]==7 && fabs(cm.gm.target[3]-(3+180/M_PI/10))<.0001);
}
'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'target.c';p.write_text(code);exe=p.with_suffix('')
            fixture=Path(__file__).parent/'planner_startup'
            subprocess.run(['cc','-std=gnu99','-D_POSIX_C_SOURCE=200809L','-I'+str(fixture),'-I'+str(ROOT),str(p),'-lm','-o',str(exe)],check=True)
            r=subprocess.run([str(exe)],capture_output=True,text=True);self.assertEqual(r.returncode,0,r.stderr)
    def test_configured_setter(self):
        table=(ROOT/'config_app.c').read_text()
        row=next(l for l in table.splitlines() if '"ara"' in l and 'radius' in l)
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
#define FIELD_target (&cm.radius)
#define FIELD_precision 0
#define _to_millimeters(a) ((cm.gm.units_mode==INCHES)?(a)*MM_PER_INCH:(a))
typedef int stat_t;
typedef struct {float value_flt;int precision,valuetype;} nvObj_t;
struct {struct {int units_mode;} gm;float radius;} cm;
int cm_get_units_mode(int which) {return cm.gm.units_mode;}
'''
        header=(ROOT/'canonical_machine.h').read_text()
        for line in header.splitlines():
            if line.startswith('#define MAX_ROTARY_RADIUS '): code+=line+'\n'
        code+=function(table,r'stat_t set_flu\(nvObj_t \*nv\)')
        if setter!='set_flu': code+=function((ROOT/'canonical_machine.c').read_text(),r'stat_t '+setter+r'\(nvObj_t \*nv\)')
        code+=r'''
int main(void) {
 float invalid[]={-1,0,-0.0f,NAN,INFINITY,-INFINITY};
 for(int units=0;units<2;units++) {
  cm.gm.units_mode=units;
  for(unsigned i=0;i<sizeof(invalid)/sizeof(*invalid);i++) {
   cm.radius=123;nvObj_t n={.value_flt=invalid[i]};
   assert(SETTER(&n)!=STAT_OK);assert(cm.radius==123);
  }
  nvObj_t n={.value_flt=10};assert(SETTER(&n)==0);
  assert(cm.radius==(units?254.0f:10.0f));
 }
 cm.radius=123;nvObj_t n={.value_flt=1e38f};
 assert(SETTER(&n)!=0 && cm.radius==123);
}
'''.replace('SETTER',setter)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'check.c';p.write_text(code);exe=Path(d)/'check'
            subprocess.run(['cc','-std=c99','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(p),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
