"""N72: homing sets the zero only after the zero backoff ran its full distance.

A switch edge during homing stops the current move, so a glitch in the zero backoff ended it early and the zero was
set there (GitHub #37). _homing_axis_set_zero must compare the runtime position with where the backoff started:
reaching the move's target (either direction, and in degrees for a radius-mode rotary, where the backoff is
a linear distance) homes; stopping short flushes the held rest of the move and fails with 240.
"""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]
PRE=r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#define HOMING_ZERO_BACKOFF_TOLERANCE 0.01f
typedef int stat_t;
enum {STAT_OK,STAT_EAGAIN,SW_INACTIVE,SW_ACTIVE,SW_DISABLED,RUNTIME,SR_REQUEST_TIMED};
#define STAT_HOMING_CYCLE_FAILED 240
struct {bool homed[6];struct {float target[6];} gm;} cm;
static float pos,scale=1;
struct {stat_t (*func)(int8_t);int8_t homing_switch;float zero_backoff,zero_target,search_velocity,saved_jerk;bool set_coordinates;} hm;
static int raw,reads,moves,zeroed,error;static float jerk;
static int read_switch(int n){assert(n==0);reads++;return raw;}
static stat_t _homing_axis_move(int8_t a,float d,float v){moves++;cm.gm.target[a]=pos+d*scale;return STAT_EAGAIN;}
static stat_t _set_homing_func(stat_t (*f)(int8_t)){hm.func=f;return STAT_EAGAIN;}
static stat_t _homing_axis_start(int8_t a){return STAT_OK;}
static void cm_set_position(int8_t a,float v){zeroed++;}
static int flushes;
static float cm_get_absolute_position(int a,int8_t b){return pos;}
static void cm_queue_flush(void){flushes++;}
static void cm_set_axis_jerk(int8_t a,float j){jerk=j;}
static void sr_request_status_report(int n){}
static stat_t _homing_error_exit(int8_t a,stat_t s){error=s;return s;}
static stat_t _homing_axis_set_zero(int8_t);
'''
class ZeroBackoffDistance(unittest.TestCase):
 def test_short_zero_backoff_fails(self):
  src=(ROOT/'firmware/tinyg/cycle_homing.c').read_text(errors='replace');code=PRE
  for name in ('_homing_abort','_homing_axis_zero_backoff','_homing_axis_set_zero'):
   code+=function(src,r'static stat_t '+name+r'\([^;{]*\)(?=[^;{]*\{)')+'\n'
  code+=r"""
static int run(float backoff,float travelled){ /* travelled in machine units */
 hm.homing_switch=0;hm.saved_jerk=20;hm.set_coordinates=true;hm.zero_backoff=backoff;raw=SW_INACTIVE;
 reads=moves=zeroed=error=flushes=0;cm.homed[0]=false;jerk=5;pos=10;
 assert(_homing_axis_zero_backoff(0)==STAT_EAGAIN && moves==1);
 pos=10+travelled;
 return hm.func(0);
}
int main(void){
 assert(run(5,5)!=240 && cm.homed[0] && zeroed==1 && !flushes);          /* full backoff: homed */
 assert(run(-5,-5)!=240 && cm.homed[0] && zeroed==1 && !flushes);        /* negative direction */
 assert(run(5,5.005f)!=240 && cm.homed[0]);                              /* within tolerance */
 assert(run(5,2)==240 && error==240 && flushes==1 && !zeroed && !cm.homed[0] && jerk==20);  /* cut short */
 assert(run(-5,-1)==240 && flushes==1 && !cm.homed[0]);
 assert(run(0,0)!=240 && cm.homed[0]);                                   /* no zero backoff configured */
 scale=5.7296f;                                                           /* radius mode: 2 mm at ara 10 = 11.459 deg */
 assert(run(2,11.459f)!=240 && cm.homed[0]);                             /* full backoff in degrees: homed */
 assert(run(2,4)==240 && flushes==1 && !cm.homed[0]);                    /* cut short */
 scale=1;
 return 0;
}
"""
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'zero.c';p.write_text(code);exe=p.with_suffix('')
   subprocess.run(['cc','-std=c99','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(p),'-o',str(exe)],check=True)
   r=subprocess.run([str(exe)],capture_output=True,text=True);self.assertEqual(r.returncode,0,r.stderr)
if __name__=='__main__':unittest.main()
