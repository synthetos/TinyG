"""Motor setters must load at boot despite startup's queued offset command."""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function, RUNTIME_STUBS
ROOT=Path(__file__).resolve().parents[2]
class StepperBootGeometry(unittest.TestCase):
 def test_boot_load_and_runtime_guard(self):
  source=(ROOT/'firmware/tinyg/stepper.c').read_text()
  main=(ROOT/'firmware/tinyg/main.c').read_text()
  code=r'''
#include <stdint.h>
static void st_refresh_move_time(void) {}
#include <stdbool.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#define __AVR 1
#define STAT_OK 0
#define STAT_INPUT_VALUE_RANGE_ERROR 110
#define FREQUENCY_DDA 50000
#define EPSILON 0.00001f
#define _to_millimeters(a) (a)
#define STAT_COMMAND_NOT_ACCEPTED 106
#define STAT_INPUT_LESS_THAN_MIN_VALUE 108
#define STAT_INPUT_VALUE_RANGE_ERROR 110
#define PREP_BUFFER_OWNED_BY_LOADER 1
#define STD_IN 0
#define STD_OUT 1
#define STD_ERR 2
#define PSTR(s) (s)
#define cli() (SREG &= ~128)
typedef int stat_t;
typedef struct {char token[6],group[4];union {float value_flt;uint32_t value_int;};void *target;} nvObj_t;
struct {uint32_t dda_ticks_downcount;} st_run;
struct {int buffer_state;} st_pre;
struct {struct {float step_angle,travel_rev,steps_per_unit,units_per_step;uint32_t microsteps;} mot[4];} st_cfg;
static uint8_t SREG;
static bool queued,stepper_configured;
static unsigned configured,computed;
bool mp_has_runnable_buffer(void){assert(!(SREG&128));return queued;}
void stepper_reset(void){assert(!st_run.dda_ticks_downcount);st_pre.buffer_state=0;}
stat_t set_flt(nvObj_t *nv){*(float *)nv->target=nv->value_flt;return 0;}
stat_t set_flu(nvObj_t *nv){return set_flt(nv);}
stat_t set_u32(nvObj_t *nv){*(uint32_t *)nv->target=nv->value_int;return 0;}
void nv_add_message_P(const char *s){assert(0);}
void _set_hw_microsteps(int m,int value){assert(m>=0&&m<4&&value==8);computed++;}
static bool st_motor_is_rotary(const void *nv){return false;}  /* the motors here are linear */
'''
  for m in range(1,7):
   code+=f'#define MOTOR_{m} {m-1}\n#define M{m}_STEP_ANGLE 1.8f\n#define M{m}_TRAVEL_PER_REV 1.25f\n#define M{m}_MICROSTEPS 8\n'
  for name in ('_init_motor_geometry','_motor_geometry_valid','st_runtime_isbusy','_stepper_configuration_busy','_get_motor','_set_motor_steps_per_unit','st_set_sa','st_set_tr','st_set_mi'):
   code+=function(source,r'(?:static )?(?:bool|int8_t|uint8_t|void|stat_t) '+name+r'\([^;{}]*\)[^\n]*\n(?=\{)')+'\n'
  if 'void stepper_init_complete(' in source:
   code+=function(source,r'void stepper_init_complete\([^;{}]*\)(?=\s*\{)')+'\n'
  code+=r'''
void config_init(void){
 assert(queued);assert(!(SREG&128));
 // Values returned by EEPROM reads; config_init ignores rejected setters.
 for(int m=0;m<4;m++){
  nvObj_t nv={0};nv.token[0]='1'+m;
  nv.target=&st_cfg.mot[m].step_angle;nv.value_flt=.9f;st_set_sa(&nv);
  nv.target=&st_cfg.mot[m].travel_rev;nv.value_flt=5.0f;st_set_tr(&nv);
  nv.target=&st_cfg.mot[m].microsteps;nv.value_int=8;st_set_mi(&nv);
 }
 configured++;
}
void controller_init(int a,int b,int c){}
struct {bool configured;} sw;
void adc_init(void){}
void spindle_init_outputs(void){}
void switch_init(void){}
void canonical_machine_reset(void){queued=true;}
void spindle_init(void){}
void spindle_reset(void){}
void xio_usb_startup_complete(void){}
void PMIC_SetVectorLocationToApplication(void){}
void PMIC_EnableHighLevel(void){}
void PMIC_EnableMediumLevel(void){}
void PMIC_EnableLowLevel(void){}
void sei(void){assert(configured==1);SREG|=128;}
'''
  code+=RUNTIME_STUBS+function(main,r'static void _application_init_startup\([^;{}]*\)(?=\s*\{)')+'\n'
  code+=r'''
int main(void){
 // canonical_machine_init already calls reset before _application_init_startup.
 _init_motor_geometry();canonical_machine_reset();_application_init_startup();
 for(int m=0;m<4;m++){
  assert(st_cfg.mot[m].step_angle==.9f && st_cfg.mot[m].travel_rev==5 && st_cfg.mot[m].microsteps==8);
  assert(fabs(st_cfg.mot[m].steps_per_unit-640)<.001);
 }
 assert(computed==4 && stepper_configured);
 for(int irq=0;irq<2;irq++)for(int busy=0;busy<4;busy++){
  SREG=irq ? 0x95 : 0x15;uint8_t saved=SREG;
  queued=busy==1;st_run.dda_ticks_downcount=busy==2;st_pre.buffer_state=busy==3;
  nvObj_t nv={.token="1sa",.value_flt=1.8f,.target=&st_cfg.mot[0].step_angle};
  st_cfg.mot[0].step_angle=.9f;
  assert(st_set_sa(&nv)==(busy ? 106:0));assert(SREG==saved);
  assert(st_cfg.mot[0].step_angle==(busy ? .9f:1.8f));
  nv.target=&st_cfg.mot[0].travel_rev;nv.value_flt=5;st_cfg.mot[0].travel_rev=10;
  assert(st_set_tr(&nv)==(busy ? 106:0));assert(SREG==saved);
  assert(st_cfg.mot[0].travel_rev==(busy ? 10:5));
  nv.target=&st_cfg.mot[0].microsteps;nv.value_int=8;st_cfg.mot[0].microsteps=4;
  assert(st_set_mi(&nv)==(busy ? 106:0));assert(SREG==saved);
  assert(st_cfg.mot[0].microsteps==(busy ? 4:8));
 }
 // The boot exception concerns queued commands only, never running/prepared steps.
 stepper_configured=false;queued=true;SREG=0x15;
 st_run.dda_ticks_downcount=1;st_pre.buffer_state=0;assert(_stepper_configuration_busy());
 st_run.dda_ticks_downcount=0;st_pre.buffer_state=1;assert(_stepper_configuration_busy());
}
'''
  with tempfile.TemporaryDirectory() as d:
   c=Path(d)/'check.c';exe=Path(d)/'check';c.write_text(code)
   subprocess.run(['cc','-Wall','-Werror','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(c),'-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
