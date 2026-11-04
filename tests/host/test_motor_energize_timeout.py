"""Exercise explicit motor hold duration with simulated ticks; no physical timing claim."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
PRELUDE = r'''
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stddef.h>
#define __AVR 1
uint8_t SREG = 0x80;
#define cli() (SREG &= (uint8_t)~128)
#define MOTORS 4
#define MOTOR_1 0
#define MOTOR_2 1
#define MOTOR_3 2
#define MOTOR_4 3
#define MOTOR_ENABLE_BIT_bm 1
#define MOTOR_DISABLED 0
#define MOTOR_ALWAYS_POWERED 1
#define MOTOR_POWERED_IN_CYCLE 2
#define MOTOR_POWERED_ONLY_WHEN_MOVING 3
#define MOTOR_OFF 0
#define MOTOR_IDLE 1
#define MOTOR_RUNNING 2
#define MOTOR_POWER_TIMEOUT_START 3
#define MOTOR_POWER_TIMEOUT_COUNTDOWN 4
#define MOTION_HOLD 2
#define SR_REQUEST_TIMED 1
#define STAT_OK 0
#define STAT_INPUT_VALUE_RANGE_ERROR 146
#define TYPE_NULL 0
#define TYPE_INTEGER 2
typedef int stat_t;
typedef struct {uint32_t value_int; int valuetype;} nvObj_t;
struct {uint8_t OUT;} ports[MOTORS];
#define PORT_MOTOR_1_VPORT ports[0]
#define PORT_MOTOR_2_VPORT ports[1]
#define PORT_MOTOR_3_VPORT ports[2]
#define PORT_MOTOR_4_VPORT ports[3]
struct {float motor_power_timeout; struct {int power_mode;} mot[MOTORS];} st_cfg;
struct {uint32_t energize_start,energize_duration; uint8_t energize_mask;
 struct {int power_state;uint32_t power_systick,power_duration;} mot[MOTORS];} st_run;
static uint32_t tick; static int motion; static bool busy;
uint32_t SysTickTimer_getValue(void) {return tick;}
int cm_get_motion_state(void) {return motion;}
bool st_runtime_isbusy(void) {return busy;}
void sr_request_status_report(int request) {(void)request;}
static bool _motor_is_enabled(uint8_t m) {return !(ports[m].OUT & 1);}
'''
EXERCISE = r'''
static void enabled(bool want) {for(int m=0;m<MOTORS;m++) assert(_motor_is_enabled(m)==want);}
static void me(uint32_t seconds,int type) {nvObj_t n={seconds,type};assert(st_set_me(&n)==STAT_OK);}
static void md(void) {nvObj_t n={0,TYPE_NULL};assert(st_set_md(&n)==STAT_OK);}
int main(void) {
 st_cfg.motor_power_timeout=2;
 for(int m=0;m<MOTORS;m++) st_cfg.mot[m].power_mode=MOTOR_POWERED_ONLY_WHEN_MOVING;
 tick=1000;me(30,TYPE_INTEGER);enabled(true);
 tick=4000;st_motor_power_callback();enabled(true);
 tick=30999;st_motor_power_callback();enabled(true);
 tick=31000;st_motor_power_callback();enabled(false);
 // Explicit duration may also be shorter than mt.
 tick=40000;me(1,TYPE_INTEGER);tick=41000;st_motor_power_callback();enabled(false);
 // Default energize requests cancel the override and retain the configured idle timeout.
 for(int type=0;type<=TYPE_INTEGER;type+=TYPE_INTEGER) {
  tick=50000;me(30,TYPE_INTEGER);me(0,type);st_motor_power_callback();
  tick=52001;st_motor_power_callback();enabled(false);
 }
 tick=60000;me(30,TYPE_INTEGER);md();tick++;st_motor_power_callback();enabled(false);
 // Partial md cancels that motor only.
 me(30,TYPE_INTEGER);nvObj_t n={2,TYPE_INTEGER};assert(st_set_md(&n)==0);
 tick++;st_motor_power_callback();assert(!_motor_is_enabled(1));assert(_motor_is_enabled(0));md();
 // Duration arithmetic survives the tick counter wrapping.
 tick=UINT32_MAX-500;me(1,TYPE_INTEGER);tick=498;st_motor_power_callback();enabled(true);
 tick=499;st_motor_power_callback();enabled(false);
 // Disabled motors stay off, even on a timed request.
 st_cfg.mot[1].power_mode=MOTOR_DISABLED;me(30,TYPE_INTEGER);assert(!_motor_is_enabled(1));md();
 st_cfg.mot[1].power_mode=MOTOR_POWERED_ONLY_WHEN_MOVING;
 // Expiry must not remove holding torque during a move or feedhold.
 tick=10000;me(1,TYPE_INTEGER);busy=true;tick=11000;st_motor_power_callback();enabled(true);
 busy=false;tick=13001;st_motor_power_callback();enabled(false);
 tick=20000;me(1,TYPE_INTEGER);motion=MOTION_HOLD;tick=21000;st_motor_power_callback();enabled(true);
 motion=0;tick=23001;st_motor_power_callback();enabled(false);
 // Reject values that cannot be converted to milliseconds, without changing outputs.
 n.value_int=4294968UL;assert(st_set_me(&n)==STAT_INPUT_VALUE_RANGE_ERROR);enabled(false);
 n.value_int=UINT32_MAX;assert(st_set_me(&n)==STAT_INPUT_VALUE_RANGE_ERROR);enabled(false);
 me(4294967UL,TYPE_INTEGER);enabled(true);md();
 return 0;
}
'''

class MotorEnergizeTimeout(unittest.TestCase):
    def test_explicit_and_default_deadlines(self):
        s=(ROOT/'firmware/tinyg/stepper.c').read_text()
        code=PRELUDE
        for name in ('_deenergize_motor','_energize_motor','st_energize_motors','st_deenergize_motors','st_motor_power_callback','st_set_md','st_set_me'):
            code+=function(s,r'(?:static )?(?:void|stat_t) '+name+r'\([^;{}]*\)(?=\s*(?://[^\n]*\n\s*)?\{)')+'\n'
        with tempfile.TemporaryDirectory() as d:
            c=Path(d)/'motor.c';exe=Path(d)/'motor';c.write_text(code+EXERCISE)
            subprocess.run(['cc','-std=c99','-fsanitize=address,undefined',str(c),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

if __name__=='__main__': unittest.main()
