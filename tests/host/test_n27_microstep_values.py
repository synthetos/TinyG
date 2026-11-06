"""N27: microstep configuration must match the onboard driver pins. TinyG configuration wiki 1MI lists 1,2,4,8. This test requires unsupported external-driver values to be refused rather than warned and narrowed; invalid settings must not change scaling, reset the stepper or touch pins."""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
FW=Path(__file__).resolve().parents[2]/'firmware/tinyg'
class Microsteps(unittest.TestCase):
 def test_real_setter_and_pins(self):
  code=r'''#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#define __AVR 1
#define STAT_OK 0
#define STAT_COMMAND_NOT_ACCEPTED 106
#define STAT_INPUT_LESS_THAN_MIN_VALUE 108
#define STAT_INPUT_VALUE_RANGE_ERROR 110
#define PSTR(s) s
#define MICROSTEP_BIT_0_bm 1
#define MICROSTEP_BIT_1_bm 2
typedef int stat_t;
typedef struct {uint32_t value_int;int index;} nvObj_t;
static struct {struct {float step_angle,travel_rev,steps_per_unit,units_per_step;uint32_t microsteps;}mot[4];} st_cfg;
static struct port {uint8_t OUTSET,OUTCLR;} ports[4];
static struct {struct port *st_port[4];} hw={{ports,ports+1,ports+2,ports+3}};
#define table_target (&st_cfg.mot[nv->index].microsteps)
#define GET_TABLE_WORD(x) table_##x
static int resets;
static void stepper_reset(void){resets++;}
static void st_refresh_move_time(void) {}
static bool _stepper_configuration_busy(void){return false;}
static int _get_motor(nvObj_t*n){return n->index;}
static void nv_add_message_P(const char*s){}
'''
  for header,name in [('hardware.h','FREQUENCY_DDA'),('util.h','EPSILON')]:
   code+=next(line for line in (FW/header).read_text().splitlines() if line.startswith('#define '+name))+'\n'
  source=(FW/'stepper.c').read_text()
  if 'static bool _motor_geometry_valid(' in source:
   code+=function(source,r'static bool _motor_geometry_valid\(')+'\n'
  for file,name in [('config.c','set_u32'),('stepper.c','_set_motor_steps_per_unit'),('stepper.c','_set_hw_microsteps'),('stepper.c','st_set_mi')]:
   code+=function((FW/file).read_text(),r'(?:static )?(?:void|stat_t) '+name+r'\([^;{}]*\)[^;{}]*(?=\{)')+'\n'
  code+=r'''
int main(void){uint32_t bad[]={0,3,16,32,256,2147483648UL,UINT32_MAX};
 for(int m=0;m<4;m++){
  st_cfg.mot[m].step_angle=1.8f;st_cfg.mot[m].travel_rev=360;st_cfg.mot[m].microsteps=8;nvObj_t n={.index=m};_set_motor_steps_per_unit(&n);
  for(int i=0;i<7;i++){ports[m].OUTSET=0xaa;ports[m].OUTCLR=0x55;int before=resets;float steps=st_cfg.mot[m].steps_per_unit;
   n.value_int=bad[i];assert(st_set_mi(&n)!=0);assert(st_cfg.mot[m].microsteps==8&&st_cfg.mot[m].steps_per_unit==steps&&resets==before);assert(ports[m].OUTSET==0xaa&&ports[m].OUTCLR==0x55);}
  for(uint32_t mi=1;mi<=8;mi*=2){n.value_int=mi;assert(st_set_mi(&n)==0);assert(st_cfg.mot[m].microsteps==mi&&isfinite(st_cfg.mot[m].steps_per_unit)&&st_cfg.mot[m].steps_per_unit>0);}
 }
}
'''
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'check.c';p.write_text(code);exe=p.with_suffix('')
   subprocess.run(['cc','-std=gnu99','-O1','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(p),'-lm','-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
