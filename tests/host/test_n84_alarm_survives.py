"""N84: only $clear or a reset ends an ALARM. Queued work finishing behind it must not.

cm_arc_callback() ran on after an alarm until the flush aborted the arc, and its cm_cycle_start() wrote MACHINE_CYCLE
over the ALARM as soon as the stop freed a planner buffer. _exec_program_finalize() (cycle end, M0, M2) wrote
MACHINE_PROGRAM_STOP over it the same way. Outputs are inhibited only in ALARM, so a queued M3 could then run.
Real cm_arc_callback(), _exec_program_finalize() and cm_cycle_end(); no timing claim.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT = Path(__file__).resolve().parents[2]
FW = ROOT/'firmware/tinyg'


def _run(code):
    with tempfile.TemporaryDirectory() as d:
        p = Path(d)
        (p/'check.c').write_text(code)
        subprocess.run(['cc', '-std=c99', '-I', str(FW), '-fsanitize=address,undefined', '-fstrict-flex-arrays=3',
                        str(p/'check.c'), '-o', str(p/'check'), '-lm'], check=True)
        subprocess.run([str(p/'check')], check=True)


class AlarmSurvives(unittest.TestCase):
    def test_arc_generator_stops_at_an_alarm(self):
        code = r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#define BLOCK_IDLE 0
#define BLOCK_RUNNING 1
#define STAT_OK 0
#define STAT_EAGAIN 2
#define STAT_NOOP 3
#define PLANNER_BUFFER_HEADROOM 4
#define AXES 6
enum {MACHINE_CYCLE,MACHINE_ALARM};
typedef int stat_t;
struct {int machine_state;} cm;
typedef struct {float target[AXES];} GCodeState_t;
struct {int run_state,plane_axis_0,plane_axis_1,linear_axis,segment_count;float theta,segment_theta,center_0,center_1,
        radius,segment_linear_travel;float position[AXES];GCodeState_t gm;} arc;
uint8_t SREG;
#define __AVR 1
#define cli() (SREG&=~128)
int buffers=32,cycle_starts,cycle_ends,alines;
stat_t cm_is_alarmed(void){return cm.machine_state==MACHINE_ALARM;}
int mp_get_planner_buffers_available(void){return buffers;}
void cm_cycle_start(void){cycle_starts++;cm.machine_state=MACHINE_CYCLE;}
void cm_cycle_end(void){cycle_ends++;}
stat_t mp_aline(GCodeState_t *g){(void)g;alines++;return STAT_OK;}
bool mp_has_runnable_buffer(void){return true;}
void copy_vector(float *d,const float *s){for(int i=0;i<AXES;i++)d[i]=s[i];}
'''
        code += function((FW/'plan_arc.c').read_text(), r'stat_t cm_arc_callback\(\)\s*(?=\{)')
        code += r'''
int main(void){
 arc.plane_axis_1=1;arc.linear_axis=2;arc.segment_count=10;arc.run_state=BLOCK_RUNNING;
 cm.machine_state=MACHINE_ALARM;
 cm_arc_callback();
 assert(cm.machine_state==MACHINE_ALARM);
 assert(cycle_starts==0&&alines==0);
 assert(arc.run_state==BLOCK_IDLE);                  // a later $clear must not restart the arc
 // Without an alarm a segment is still queued.
 arc.run_state=BLOCK_RUNNING;cm.machine_state=MACHINE_CYCLE;
 assert(cm_arc_callback()==STAT_EAGAIN);
 assert(alines==1&&cm.machine_state==MACHINE_CYCLE);
 return 0;
}
'''
        _run(code)

    def test_cycle_end_and_program_stop_keep_the_alarm(self):
        source = (FW/'canonical_machine.c').read_text()
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define __AVR 1
#define STAT_OK 0
#define SR_REQUEST_ASAP 1
#define FLAGS_ONE 0
enum {MACHINE_READY,MACHINE_CYCLE,MACHINE_ALARM,MACHINE_PROGRAM_STOP,MACHINE_PROGRAM_END,MACHINE_SHUTDOWN,MACHINE_PANIC};
enum {MOTION_STOP,MOTION_RUN,MOTION_HOLD};
enum {CYCLE_OFF,CYCLE_MACHINING};
enum {FEEDHOLD_OFF,FEEDHOLD_SYNC};
typedef int stat_t;
struct {int machine_state,motion_state,cycle_state,hold_state;bool hold_fast,hold_fast_request,end_hold_requested,
        program_paused;} cm;
uint8_t SREG;
#define cli() (SREG&=~128)
stat_t cm_is_alarmed(void){return (cm.machine_state==MACHINE_ALARM)||(cm.machine_state==MACHINE_SHUTDOWN)||
                                  (cm.machine_state==MACHINE_PANIC);}
void cm_set_motion_state(int s){cm.motion_state=s;}
static volatile bool hold_resume_cancelled;
void mp_zero_segment_velocity(void){}
void sr_request_status_report(int v){(void)v;}
'''
        code += function(source, r'static void _exec_program_finalize\(float \*value, bool \*flags\)(?=\s*\{)')
        code += function(source, r'void cm_cycle_end\(\)(?=\s*\{)')
        code += r'''
int main(void){
 int held[]={MACHINE_ALARM,MACHINE_SHUTDOWN,MACHINE_PANIC};
 for(int i=0;i<3;i++){
  cm.machine_state=held[i];cm.cycle_state=CYCLE_MACHINING;cm.motion_state=MOTION_HOLD;cm.hold_state=FEEDHOLD_SYNC;
  cm_cycle_end();
  assert(cm.machine_state==held[i]);
  float end[]={MACHINE_PROGRAM_END};
  _exec_program_finalize(end,0);
  assert(cm.machine_state==held[i]);
  assert(cm.hold_state==FEEDHOLD_SYNC);              // the flush owns the hold
 }
 // $clear leaves ALARM first, so its own cycle end still works, as does a normal one.
 cm.machine_state=MACHINE_PROGRAM_STOP;cm.cycle_state=CYCLE_MACHINING;cm.hold_state=FEEDHOLD_OFF;
 cm_cycle_end();
 assert(cm.machine_state==MACHINE_PROGRAM_STOP&&cm.cycle_state==CYCLE_OFF&&cm.motion_state==MOTION_STOP);
 return 0;
}
'''
        _run(code)


if __name__ == '__main__':
    unittest.main()
