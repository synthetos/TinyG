"""N15: a latch move must release its switch before zero backoff or homing success.

TinyG wiki 'Homing and Limits Setup and Troubleshooting', 'Homing - How it's
supposed to work': latch stops when the switch clears, before zero backoff.
The current error table has no did-not-release code; use HOMING_CYCLE_FAILED.
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
struct {stat_t (*func)(int8_t);int8_t homing_switch;float zero_backoff,zero_target,search_velocity,saved_jerk;bool set_coordinates;} hm;
static int raw,reads,moves,zeroed,error;static float jerk;
static int read_switch(int n){assert(n==0);reads++;return raw;}
static stat_t _homing_axis_move(int8_t a,float d,float v){moves++;cm.gm.target[a]=1+d;return STAT_EAGAIN;}
static stat_t _set_homing_func(stat_t (*f)(int8_t)){hm.func=f;return STAT_EAGAIN;}
static stat_t _homing_axis_start(int8_t a){return STAT_OK;}
static void cm_set_position(int8_t a,float v){zeroed++;}
static float cm_get_absolute_position(int a,int8_t b){return 1;}
static void cm_queue_flush(void){}
static void cm_set_axis_jerk(int8_t a,float j){jerk=j;}
static void sr_request_status_report(int n){}
static stat_t _homing_error_exit(int8_t a,stat_t s){error=s;return s;}
static stat_t _homing_axis_set_zero(int8_t);
'''
class HomingRelease(unittest.TestCase):
 def test_release_required_before_zero_backoff(self):
  src=(ROOT/'firmware/tinyg/cycle_homing.c').read_text();code=PRE
  for name in ('_homing_abort','_homing_axis_zero_backoff','_homing_axis_set_zero'):
   code+=function(src,r'static stat_t '+name+r'\([^;{]*\)(?=[^;{]*\{)')+'\n'
  code+=r'''
int main(void){
 hm.homing_switch=0;hm.saved_jerk=20;hm.set_coordinates=true;
 for(int failed=0;failed<3;failed++){
  raw=failed==0?SW_INACTIVE:failed==1?SW_ACTIVE:SW_DISABLED;
  reads=moves=zeroed=error=0;cm.homed[0]=false;jerk=5;hm.func=0;
  int s=_homing_axis_zero_backoff(0);
  if(failed){assert(s==240 && error==240);assert(!moves && !zeroed && !cm.homed[0]);assert(jerk==20);}
  else{assert(s==STAT_EAGAIN && moves==1);hm.func(0);assert(cm.homed[0] && zeroed==1);}
 }
}
'''
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'latch.c';p.write_text(code);exe=p.with_suffix('')
   subprocess.run(['cc','-std=c99','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(p),'-o',str(exe)],check=True)
   r=subprocess.run([str(exe)],capture_output=True,text=True);self.assertEqual(r.returncode,0,r.stderr)
if __name__=='__main__':unittest.main()
