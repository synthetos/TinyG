"""N12b: derived motor geometry must remain finite and usable after each SET.

TinyG Configuration wiki 0.97, Motor Settings: steps/unit is derived from step
angle, travel/revolution and microsteps. A step rate is geometry times velocity;
160000 steps/mm is valid at a sufficiently low feed, not an invalid geometry.
"""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]
class MotorGeometry(unittest.TestCase):
 def test_setters_and_startup(self):
  source=(ROOT/'firmware/tinyg/stepper.c').read_text()
  code=r'''
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include <float.h>
#define __AVR 1
#define STAT_OK 0
#define STAT_COMMAND_NOT_ACCEPTED 106
#define STAT_INPUT_LESS_THAN_MIN_VALUE 108
#define STAT_INPUT_VALUE_RANGE_ERROR 110
#define FREQUENCY_DDA 50000
#define EPSILON 0.00001f
#define PSTR(s) (s)
#define _to_millimeters(a) ((inches)?(a)*25.4f:(a))
typedef int stat_t;
typedef struct {char token[6],group[4];union {float value_flt;uint32_t value_int;};void *target;} nvObj_t;
typedef struct {float step_angle,travel_rev,steps_per_unit,units_per_step;uint32_t microsteps;} motor_t;
struct {motor_t mot[6];} st_cfg;
static bool inches;
static unsigned resets,pin_writes;
void st_refresh_move_time(void) {} // rate-cache side effect is checked by the step-rate fixture
bool _stepper_configuration_busy(void){return false;}
void stepper_reset(void){resets++;}
stat_t set_flt(nvObj_t *nv){*(float *)nv->target=nv->value_flt;return 0;}
stat_t set_flu(nvObj_t *nv){*(float *)nv->target=_to_millimeters(nv->value_flt);return 0;}
stat_t set_u32(nvObj_t *nv){*(uint32_t *)nv->target=nv->value_int;return 0;}
void nv_add_message_P(const char *s){}
void _set_hw_microsteps(int m,int value){pin_writes++;}
'''
  for m in range(1,7):
   code+=f'#define MOTOR_{m} {m-1}\n'
   for field,value in [('STEP_ANGLE','1.8f'),('TRAVEL_PER_REV','1.25f'),('MICROSTEPS','8')]:
    code+=f'#define M{m}_{field} {value}\n'
  names=['_get_motor','_set_motor_steps_per_unit','st_set_sa','st_set_tr','st_set_mi']
  for helper in ['_init_motor_geometry','_motor_geometry_valid']:
   if 'static '+('void' if helper.startswith('_init') else 'bool')+' '+helper+'(' in source:names.insert(0,helper)
  for name in names:
   code+=function(source,r'(?:static )?(?:bool|int8_t|uint8_t|void|stat_t) '+name+r'\([^;{}]*\)[^\n]*\n(?=\{)')+'\n'
  code+=r'''
static void seed(void){inches=false;st_cfg.mot[0]=(motor_t){1.8f,40,40,0.025f,8};}
static void unchanged(motor_t before,unsigned r,unsigned p){
 assert(memcmp(&before,&st_cfg.mot[0],sizeof before)==0);assert(resets==r&&pin_writes==p);
}
int main(void){
 nvObj_t nv={.token="1sa"};seed();motor_t before=st_cfg.mot[0];
 unsigned r=resets,p=pin_writes;
 nv.target=&st_cfg.mot[0].step_angle;nv.value_flt=1e-38f;
 assert(st_set_sa(&nv)==STAT_INPUT_VALUE_RANGE_ERROR);unchanged(before,r,p);
 nv.value_flt=1e-12f;assert(st_set_sa(&nv)==STAT_INPUT_VALUE_RANGE_ERROR);unchanged(before,r,p);
 nv.target=&st_cfg.mot[0].travel_rev;nv.value_flt=1e-38f;
 assert(st_set_tr(&nv)==STAT_INPUT_VALUE_RANGE_ERROR);unchanged(before,r,p);
 inches=true;nv.value_flt=FLT_MAX/2;
 assert(st_set_tr(&nv)==STAT_INPUT_VALUE_RANGE_ERROR);unchanged(before,r,p);
 seed();st_cfg.mot[0].travel_rev=.01f;st_cfg.mot[0].steps_per_unit=160000;
 before=st_cfg.mot[0];nv.target=&st_cfg.mot[0].microsteps;nv.value_int=UINT32_MAX;
 assert(st_set_mi(&nv)==STAT_INPUT_VALUE_RANGE_ERROR);unchanged(before,r,p);
 // High density can run slowly: no dimensionally incorrect 50000 steps/unit cap.
 nv.value_int=8;assert(st_set_mi(&nv)==STAT_OK);
 assert(fabsf(st_cfg.mot[0].steps_per_unit-160000)<1);
 // Unsupported microsteps are refused before changing scaling or pins.
 seed();nv.target=&st_cfg.mot[0].microsteps;nv.value_int=20000000;
 before=st_cfg.mot[0];r=resets;p=pin_writes;
 assert(st_set_mi(&nv)==STAT_INPUT_VALUE_RANGE_ERROR);unchanged(before,r,p);
 // The internal geometry calculation still uses float multiplication rather
 // than wrapping a 32-bit integer product, even for an out-of-range old value.
 st_cfg.mot[0].microsteps=20000000;_set_motor_steps_per_unit(&nv);
 assert(fabsf(st_cfg.mot[0].steps_per_unit-100000000)<16);
'''
  if '_init_motor_geometry(' in source:
   code+=r'''
 memset(&st_cfg,0,sizeof st_cfg);_init_motor_geometry();
 // A bad old EEPROM angle is rejected even before tr/mi have been loaded.
 nv.target=&st_cfg.mot[0].step_angle;nv.value_flt=1e-38f;
 assert(st_set_sa(&nv)==STAT_INPUT_VALUE_RANGE_ERROR);assert(st_cfg.mot[0].step_angle==1.8f);
 nv.value_flt=.9f;assert(st_set_sa(&nv)==STAT_OK);
 nv.target=&st_cfg.mot[0].travel_rev;nv.value_flt=5;assert(st_set_tr(&nv)==STAT_OK);
 nv.target=&st_cfg.mot[0].microsteps;nv.value_int=8;assert(st_set_mi(&nv)==STAT_OK);
 assert(st_cfg.mot[0].steps_per_unit==640);
'''
  code+='}\n'
  with tempfile.TemporaryDirectory() as d:
   c=Path(d)/'check.c';exe=Path(d)/'check';c.write_text(code)
   subprocess.run(['cc','-Wall','-Werror','-fsanitize=address,undefined',str(c),'-o',str(exe)],check=True)
   result=subprocess.run([str(exe)],capture_output=True,text=True)
   self.assertEqual(result.returncode,0,result.stderr)
