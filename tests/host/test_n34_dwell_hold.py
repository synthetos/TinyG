"""Hold admission during a stopped cycle/dwell, with real request and startup code."""
from pathlib import Path
import subprocess
import tempfile
import os
import unittest
from source_extract import function

ROOT = Path(os.environ.get('TINYG_FIRMWARE', Path(__file__).resolve().parents[2] / 'firmware/tinyg'))

class DwellHold(unittest.TestCase):
    def test_stopped_cycle_and_lone_dwell_hold(self):
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define __AVR 1
#define STAT_OK 0
#define SR_REQUEST_ASAP 1
enum {MOTION_STOP, MOTION_RUN, MOTION_HOLD};
enum {CYCLE_OFF, CYCLE_MACHINING, CYCLE_HOMING};
enum {MACHINE_READY=1,MACHINE_ALARM,MACHINE_PROGRAM_STOP,MACHINE_PROGRAM_END,MACHINE_CYCLE};
enum {FEEDHOLD_OFF, FEEDHOLD_REQUESTED, FEEDHOLD_SYNC, FEEDHOLD_HOLD};
enum {PREP_BUFFER_OWNED_BY_EXEC,PREP_BUFFER_OWNED_BY_LOADER};
enum {BLOCK_TYPE_NULL,BLOCK_TYPE_DWELL,BLOCK_TYPE_ALINE};
enum {CONTROLLER_READY,CONTROLLER_PAUSED};
struct {int hold_state,motion_state,cycle_state,machine_state;bool hold_fast,hold_fast_request,end_hold_requested;} cm;
struct {int controller_state;} cs;
static struct {bool resume_cancelled, internal_hold;} hold_outputs;
struct {int buffer_state,block_type;} st_pre;
uint8_t SREG;
static bool startup_pending,busy,aline;
static int reports;
static void cli(void){SREG &= (uint8_t)~128;}
void cm_set_motion_state(int s){assert(!(SREG&128));cm.motion_state=s;}
void cm_cycle_start(void){cm.machine_state=MACHINE_CYCLE;cm.cycle_state=CYCLE_MACHINING;}
int cm_is_alarmed(void){return cm.machine_state==MACHINE_ALARM;}
bool mp_runtime_is_idle(void){return !busy;}
bool st_runtime_isbusy(void){return busy;}
bool st_aline_in_flight(void){return aline;}
void sr_request_status_report(int k){assert(k==SR_REQUEST_ASAP);reports++;}
'''
        code += function((ROOT/'planner.c').read_text(), r'bool mp_hold_startup\(void\)(?=\s*\{)')
        code += function((ROOT/'canonical_machine.c').read_text(), r'void cm_request_feedhold\(void\)(?=\s*\{)')
        code += r'''
static void reset(uint8_t irq){
 cm.hold_state=FEEDHOLD_OFF;cm.motion_state=MOTION_STOP;cm.cycle_state=CYCLE_OFF;
 cm.machine_state=MACHINE_READY;cm.hold_fast=cm.hold_fast_request=true;
 cs.controller_state=CONTROLLER_PAUSED;st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;
 st_pre.block_type=BLOCK_TYPE_NULL;startup_pending=busy=aline=false;reports=0;SREG=irq;
}
static void held(uint8_t irq){assert(SREG==irq);assert(cm.hold_state==FEEDHOLD_HOLD);
 assert(cm.motion_state==MOTION_HOLD);assert(cs.controller_state==CONTROLLER_READY);
 assert(!cm.hold_fast&&!cm.hold_fast_request);assert(reports==1);}
int main(void){
 for(int irq=0;irq<=128;irq+=128){
  reset(irq);cm.cycle_state=CYCLE_MACHINING;cm.machine_state=MACHINE_CYCLE;busy=true;
  cm_request_feedhold();held(irq);assert(busy); // dwell timer keeps running
  reset(irq);cm.cycle_state=CYCLE_MACHINING;cm.machine_state=MACHINE_CYCLE;cm.motion_state=MOTION_RUN;busy=true;
  cm_request_feedhold();held(irq); // a dwell between lines inherits MOTION_RUN
  cm_request_feedhold();assert(reports==1); // repeated ! does not restart anything
  reset(irq);busy=true;cm.machine_state=MACHINE_PROGRAM_STOP;
  cm_request_feedhold();held(irq);assert(cm.machine_state==MACHINE_CYCLE);
  assert(cm.cycle_state==CYCLE_MACHINING); // lone dwell's planner already ended its cycle
  reset(irq);st_pre.buffer_state=PREP_BUFFER_OWNED_BY_LOADER;st_pre.block_type=BLOCK_TYPE_DWELL;
  cm_request_feedhold();held(irq); // dwell prepared but not yet loaded
  reset(irq);cm_request_feedhold();assert(cm.hold_state==FEEDHOLD_OFF&&cm.motion_state==MOTION_STOP);
  assert(SREG==irq&&reports==0); // idle ! cannot block the next job
  reset(irq);cm.cycle_state=CYCLE_HOMING;cm_request_feedhold();assert(cm.hold_state==FEEDHOLD_OFF);
  reset(irq);cm.machine_state=MACHINE_ALARM;busy=true;cm_request_feedhold();assert(cm.machine_state==MACHINE_ALARM);
  reset(irq);cm.motion_state=MOTION_RUN;busy=aline=true;
  cm_request_feedhold();assert(cm.hold_state==FEEDHOLD_REQUESTED&&SREG==irq);
  reset(irq);startup_pending=true;cm.cycle_state=CYCLE_MACHINING;cm.machine_state=MACHINE_CYCLE;
  cm_request_feedhold();held(irq);assert(!startup_pending);
 }
}
'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'check.c').write_text(code)
            subprocess.run(['cc','-std=c99','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(p/'check.c'),'-o',str(p/'check')],check=True)
            subprocess.run([str(p/'check')],check=True)


    def test_executor_waits_for_dwell_and_preserves_its_wakeup(self):
        code=r"""
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
typedef int stat_t;
#define STAT_OK 0
#define STAT_NOOP 1
#define STAT_INTERNAL_ERROR 99
#define PSTR(s) (s)
enum {MOTION_STOP,MOTION_RUN,MOTION_HOLD};
enum {FEEDHOLD_OFF,FEEDHOLD_HOLD};
enum {BLOCK_TYPE_NULL,BLOCK_TYPE_ALINE,BLOCK_TYPE_DWELL,BLOCK_TYPE_COMMAND};
struct {bool program_paused;int hold_state,motion_state;} cm;
typedef struct mpBuf {int block_type;} mpBuf_t;
static struct {bool exec_ran;} mb;
static mpBuf_t block;
static bool busy,aline,startup;
static int staged,called;
bool mp_startup_is_pending(void){return startup;}
mpBuf_t *mp_get_run_buffer(void){return &block;}
void st_prep_null(void){staged++;}
bool st_runtime_isbusy(void){return busy;}
bool st_aline_in_flight(void){return aline;}
void cm_set_motion_state(int s){cm.motion_state=s;}
int cm_panic_P(int status,const char *s){return status;}
static int run(mpBuf_t *b){called++;return STAT_OK;}
static int mp_exec_aline(mpBuf_t *b){return run(b);}
static int mp_exec_dwell(mpBuf_t *b){return run(b);}
static int mp_exec_command(mpBuf_t *b){return run(b);}
"""
        code += function((ROOT/'plan_exec.c').read_text(),r'stat_t mp_exec_move\(\)(?=\s*\{)')
        code+=r"""
int main(void){
 block.block_type=BLOCK_TYPE_ALINE;busy=true;
 assert(mp_exec_move()==STAT_OK);assert(staged==1&&called==0); // bubble wakes exec when dwell ends
 cm.hold_state=FEEDHOLD_HOLD;busy=false;
 assert(mp_exec_move()==STAT_NOOP&&called==0); // no queued block runs in HOLD
 cm.hold_state=FEEDHOLD_OFF;assert(mp_exec_move()==STAT_OK&&called==1);
 busy=true;aline=true;assert(mp_exec_move()==STAT_OK&&called==2); // ordinary segment pipeline unchanged
}
"""
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'check.c').write_text(code)
            subprocess.run(['cc','-std=c99','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(p/'check.c'),'-o',str(p/'check')],check=True)
            subprocess.run([str(p/'check')],check=True)


    def test_flush_waits_for_moving_runtime_and_drops_prep(self):
        from test_queue_flush_input import PRELUDE
        from source_extract import RUNTIME_STUBS
        # This busy stub models a stepping line, which must still drain;
        # actual dwell cancellation is exercised by test_n51_dwell_flush.
        code=PRELUDE.replace('bool mp_runtime_is_idle(void) { return true; }',
                            'static bool busy; bool mp_runtime_is_idle(void) { return !busy; }')
        code=code.replace('void st_prep_null(void) {}',
                          'static int prepared; void st_prep_null(void) {assert(!(SREG&128)); prepared=0;}')
        code+=RUNTIME_STUBS
        source=(ROOT/'canonical_machine.c').read_text()
        for signature in [r'void cm_request_queue_flush\(\)',r'void cm_request_planner_flush\(\)',
                          r'static void _take_input_flush_request\(\)',r'void cm_queue_flush\(\)']:
            code+=function(source,signature)
        code+=r"""
int main(void){
 cm.hold_state=FEEDHOLD_HOLD;busy=true;prepared=1;SREG=128;
 cm_request_queue_flush();cm_queue_flush();
 assert(busy&&prepared&&planner_flushes==0&&cm.hold_state==FEEDHOLD_HOLD);
 busy=false;cm_queue_flush();
 assert(!prepared&&planner_flushes==1&&cm.hold_state==FEEDHOLD_OFF);
 assert(cm.queue_flush_state==FLUSH_OFF&&SREG==128);
}
"""
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'check.c').write_text(code)
            subprocess.run(['cc','-std=c99','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(p/'check.c'),'-o',str(p/'check')],check=True)
            subprocess.run([str(p/'check')],check=True)
