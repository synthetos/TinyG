"""N85: a flush must not wait in FEEDHOLD_SYNC for a line that will never run.

An alarm (or '!' then '%') between moves leaves the hold in SYNC with the planner flush pending. SYNC only ends when an
aline runs, but _run_requested_queue_flush() refuses to flush in SYNC and cancels the dwell on every pass, and the
cancelled dwell was the only wakeup for the exec. Nothing moves, nothing is flushed, and $clear leaves the hold latched.
A stop that runs out of line in DECEL_CONTINUE with no line next never ended either. Real cm_start_hold() and
_end_decel_continue(); no timing claim.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT = Path(__file__).resolve().parents[2]


class FlushInSync(unittest.TestCase):
    def test_flush_ends_sync_only_when_nothing_is_moving(self):
        source = (ROOT/'firmware/tinyg/canonical_machine.c').read_text()
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define __AVR 1
#define STAT_OK 0
#define SR_REQUEST_ASAP 1
enum {FEEDHOLD_OFF,FEEDHOLD_REQUESTED,FEEDHOLD_SYNC,FEEDHOLD_DECEL_TO_ZERO,FEEDHOLD_HOLD};
enum {MOTION_STOP,MOTION_RUN,MOTION_HOLD};
enum {PREP_BUFFER_OWNED_BY_EXEC,PREP_BUFFER_OWNED_BY_LOADER};
enum {FLUSH_OFF,FLUSH_REQUESTED};
enum {BLOCK_IDLE,BLOCK_RUNNING};
struct {int hold_state,motion_state,machine_state,queue_flush_state;bool end_hold_requested;} cm;
struct {int block_state;} mr;
struct {int buffer_state;} st_pre;
uint8_t SREG;
bool runnable,busy;
#define cli() (SREG&=~128)
bool mp_has_runnable_buffer(void){return runnable;}
bool mp_runtime_is_idle(void){return !busy;}
void cm_set_motion_state(int s){cm.motion_state=s;}
void sr_request_status_report(int v){(void)v;}
'''
        code += function(source, r'void cm_start_hold\(\)(?=\s*\{)')
        code += r'''
static void state(int flush,int block,bool b,int prep){
 cm.hold_state=FEEDHOLD_SYNC;cm.motion_state=MOTION_HOLD;cm.queue_flush_state=flush;
 mr.block_state=block;busy=b;st_pre.buffer_state=prep;runnable=true;SREG=128;
}
int main(void){
 // Queued blocks, nothing moving, a flush waiting: this is a stop, so the flush can run.
 state(FLUSH_REQUESTED,BLOCK_IDLE,false,PREP_BUFFER_OWNED_BY_EXEC);
 cm_start_hold();assert(cm.hold_state==FEEDHOLD_HOLD);assert(SREG==128);
 // A plain hold keeps waiting for the next line to reach its hold point.
 state(FLUSH_OFF,BLOCK_IDLE,false,PREP_BUFFER_OWNED_BY_EXEC);
 cm_start_hold();assert(cm.hold_state==FEEDHOLD_SYNC);
 // A line in the runtime, a busy stepper or dwell, or a prepared segment still has to finish.
 state(FLUSH_REQUESTED,BLOCK_RUNNING,false,PREP_BUFFER_OWNED_BY_EXEC);
 cm_start_hold();assert(cm.hold_state==FEEDHOLD_SYNC);
 state(FLUSH_REQUESTED,BLOCK_IDLE,true,PREP_BUFFER_OWNED_BY_EXEC);
 cm_start_hold();assert(cm.hold_state==FEEDHOLD_SYNC);
 state(FLUSH_REQUESTED,BLOCK_IDLE,false,PREP_BUFFER_OWNED_BY_LOADER);
 cm_start_hold();assert(cm.hold_state==FEEDHOLD_SYNC);
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)
            (p/'check.c').write_text(code)
            subprocess.run(['cc', '-std=c99', '-fsanitize=address,undefined', '-fstrict-flex-arrays=3', str(p/'check.c'),
                            '-o', str(p/'check')], check=True)
            subprocess.run([str(p/'check')], check=True)


class DecelContinue(unittest.TestCase):
    def test_a_stop_continues_only_into_a_line(self):
        source = (ROOT/'firmware/tinyg/plan_exec.c').read_text()
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
enum {FEEDHOLD_OFF,FEEDHOLD_REQUESTED,FEEDHOLD_SYNC,FEEDHOLD_DECEL_CONTINUE,FEEDHOLD_DECEL_TO_ZERO,FEEDHOLD_PENDING,FEEDHOLD_HOLD};
enum {BLOCK_TYPE_NULL,BLOCK_TYPE_ALINE,BLOCK_TYPE_DWELL,BLOCK_TYPE_COMMAND};
struct {int hold_state;} cm;
struct {int block_type;} next;
struct {__typeof__(next) *r;} mb = {&next};
bool runnable;
bool mp_has_runnable_buffer(void){return runnable;}
'''
        code += function(source, r'static void _end_decel_continue\(void\)(?=\s*\{)')
        code += r'''
static int after(int hold,bool queued,int type){
 cm.hold_state=hold;runnable=queued;next.block_type=type;
 _end_decel_continue();
 return cm.hold_state;
}
int main(void){
 // The next line carries the stop on.
 assert(after(FEEDHOLD_DECEL_CONTINUE,true,BLOCK_TYPE_ALINE)==FEEDHOLD_DECEL_CONTINUE);
 // M codes, a dwell, a block still being written or nothing at all: nothing would ever end the state.
 assert(after(FEEDHOLD_DECEL_CONTINUE,true,BLOCK_TYPE_COMMAND)==FEEDHOLD_SYNC);
 assert(after(FEEDHOLD_DECEL_CONTINUE,true,BLOCK_TYPE_DWELL)==FEEDHOLD_SYNC);
 assert(after(FEEDHOLD_DECEL_CONTINUE,true,BLOCK_TYPE_NULL)==FEEDHOLD_SYNC);
 assert(after(FEEDHOLD_DECEL_CONTINUE,false,BLOCK_TYPE_NULL)==FEEDHOLD_SYNC);
 // No other hold state is touched.
 assert(after(FEEDHOLD_OFF,true,BLOCK_TYPE_DWELL)==FEEDHOLD_OFF);
 assert(after(FEEDHOLD_DECEL_TO_ZERO,false,BLOCK_TYPE_NULL)==FEEDHOLD_DECEL_TO_ZERO);
 assert(after(FEEDHOLD_PENDING,true,BLOCK_TYPE_COMMAND)==FEEDHOLD_PENDING);
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)
            (p/'check.c').write_text(code)
            subprocess.run(['cc', '-std=c99', '-fsanitize=address,undefined', '-fstrict-flex-arrays=3', str(p/'check.c'),
                            '-o', str(p/'check')], check=True)
            subprocess.run([str(p/'check')], check=True)


if __name__ == '__main__':
    unittest.main()
