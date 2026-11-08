"""N14: jog alarm exit preserves ALARM and restores modal state without advancing the jog.

Real jog callback/finalizer and canonical cycle completion code; motion completion
is controlled so the alarm can land before a move, while moving or after stop.
"""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]
PRE=r'''

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
typedef int stat_t;
enum {STAT_OK,STAT_NOOP,STAT_EAGAIN,CYCLE_OFF,CYCLE_JOG,CYCLE_MACHINING,MACHINE_ALARM,MACHINE_PROGRAM_STOP,MOTION_STOP,FEEDHOLD_OFF,FEEDHOLD_PENDING,FEEDHOLD_HOLD,MODEL,MOTION_MODE_CANCEL,SR_REQUEST_ASAP,SW_ACTIVE,RUNTIME};
#define PSTR(s) s
#define printf_P printf
#define FLAGS_ONE ((bool[]){true})
struct {int machine_state,cycle_state,hold_state; bool program_paused,hold_fast,hold_fast_request,end_hold_requested; bool homed[6]; struct {float feed_rate;} gm;} cm;
struct {stat_t (*func)(int8_t);int8_t axis; float saved_jerk,saved_feed_rate;int saved_coord_system,saved_units_mode,saved_distance_mode,saved_feed_rate_mode;} jog;
static bool moving;
static bool cm_get_runtime_busy(void){return moving;}
static void mp_flush_planner(void){}
static void cm_set_axis_jerk(int a,float b){}
static void cm_set_coord_system(int a){}
static void cm_set_units_mode(int a){}
static void cm_set_distance_mode(int a){}
static void cm_set_feed_rate_mode(int a){}
static void cm_set_motion_mode(int a,int b){}
static void cm_set_motion_state(int a){}
static struct {unsigned state;bool resume_cancelled, internal_hold;} hold_outputs;
static void mp_zero_segment_velocity(void){}
static void sr_request_status_report(int a){}

static int cm_is_alarmed(void){return cm.machine_state==MACHINE_ALARM?1:STAT_OK;}
static stat_t unexpected(int8_t a){assert(0);return 0;}
static stat_t _jogging_finalize_exit(int8_t);
'''
class JogAlarm(unittest.TestCase):
 def test_alarm_exit(self):
  j=(ROOT/'firmware/tinyg/cycle_jogging.c').read_text()
  c=(ROOT/'firmware/tinyg/canonical_machine.c').read_text()
  code=PRE
  for src,sig in [(c,r'static void _exec_program_finalize\([^;{]*\)(?=\s*\{)'),(c,r'void cm_cycle_end\(\)'),(j,r'static stat_t _jogging_finalize_exit\([^;{]*\)(?=[^;{]*\{)'),(j,r'stat_t cm_jogging_callback\(')]:code+=function(src,sig)+'\n'
  code+=r'''
int main(void){
 for(int phase=0;phase<2;phase++){
  cm.cycle_state=CYCLE_JOG;cm.machine_state=MACHINE_ALARM;cm.hold_state=FEEDHOLD_HOLD;
  jog.func=phase?_jogging_finalize_exit:unexpected;jog.saved_feed_rate=123;
  moving=true;assert(cm_jogging_callback()==STAT_EAGAIN);assert(cm.cycle_state==CYCLE_JOG);
  moving=false;cm.hold_state=FEEDHOLD_PENDING;
  assert(cm_jogging_callback()==STAT_EAGAIN);assert(cm.machine_state==MACHINE_ALARM);
  cm.hold_state=FEEDHOLD_HOLD;
  assert(cm_jogging_callback()==STAT_OK);assert(cm.machine_state==MACHINE_ALARM);
  assert(cm.cycle_state==CYCLE_OFF && cm.gm.feed_rate==123);
  assert(cm_jogging_callback()==STAT_NOOP);
 }
 cm.cycle_state=CYCLE_JOG;cm.machine_state=MACHINE_PROGRAM_STOP;cm.hold_state=FEEDHOLD_OFF;
 jog.func=_jogging_finalize_exit;assert(cm_jogging_callback()==STAT_OK);
 assert(cm.cycle_state==CYCLE_OFF && cm.machine_state==MACHINE_PROGRAM_STOP);
}
'''
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'jog.c';p.write_text(code);exe=p.with_suffix('')
   subprocess.run(['cc','-std=c99','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(p),'-o',str(exe)],check=True)
   r=subprocess.run([str(exe)],capture_output=True,text=True);self.assertEqual(r.returncode,0,r.stderr)
if __name__=='__main__':unittest.main()
