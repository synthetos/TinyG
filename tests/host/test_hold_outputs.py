"""Deterministic hold/output sequencing; physical timing requires external capture."""

from source_extract import OVERRIDE_STUBS
from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = Path(os.environ.get('TINYG_FIRMWARE', ROOT/'firmware/tinyg'))


class HoldOutputs(unittest.TestCase):
    def test_motion_only_hold_resume_and_flush(self):
        source=(FIRMWARE/'canonical_machine.c').read_text()
        definitions=[]
        for name in ['_stop_outputs',
                     'cm_get_combined_state', 'cm_start_hold', 'cm_end_hold', 'cm_queue_flush',
                     '_take_input_flush_request', '_service_input_flush', '_run_requested_queue_flush',
                     'cm_feedhold_sequencing_callback', 'cm_request_feedhold', 'cm_request_cycle_feedhold', 'cm_request_end_hold', '_exec_program_finalize']:
            signature=r'(?m)^(?:static )?(?:void|stat_t|bool|cmCombinedState) '+name+r'\([^;{}\n]*\)\s*'
            match=re.search(signature+r'\{', source)
            if match is None and name == 'cm_request_cycle_feedhold':
                # Before provenance exists, cycle contacts use the ordinary request path.
                definitions.append('void cm_request_cycle_feedhold(void) {cm_request_feedhold();}')
                continue
            self.assertIsNotNone(match,name)
            definitions.append(function(source[match.start():],signature))
        state='static volatile bool hold_resume_cancelled;'
        code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#define __AVR 1
#define cli() (SREG &= ~128)
#define STAT_OK 0
#define STAT_STATE_MANAGEMENT_ASSERTION_FAILURE 1
#define BLOCK_TYPE_NULL 0
static void cm_panic_P(int status, const char *message) {assert(0);}
#define SPINDLE_OFF 0
#define SPINDLE_CW 1
#define SPINDLE_CCW 2
#define FEEDHOLD_OFF 0
#define FEEDHOLD_REQUESTED 1
#define FEEDHOLD_SYNC 2
#define FEEDHOLD_PENDING 3
#define FEEDHOLD_HOLD 4
#define MOTION_STOP 0
#define MOTION_RUN 1
#define MOTION_HOLD 2
#define FLUSH_OFF 0
#define FLUSH_REQUESTED 1
#define AXIS_X 0
#define BLOCK_IDLE 0
#define CYCLE_MACHINING 1
#define CYCLE_OFF 0
#define CYCLE_HOMING 2
#define CYCLE_PROBE 3
#define MACHINE_PROGRAM_STOP 3
#define SR_REQUEST_ASAP 1
#define CONTROLLER_READY 1
#define BLOCK_TYPE_COMMAND 2
#define BLOCK_TYPE_DWELL 3
#define AXES 6
#define FLAGS_NONE ((bool*)0)
typedef int stat_t;
struct { unsigned hold_state, motion_state, queue_flush_state; bool end_hold_requested, program_paused, job_kill_requested, queue_flush_input, queue_flush_input_due, hold_fast_request, hold_fast, job_kill_report_pending;
         unsigned cycle_state, limit_requested, shutdown_requested, machine_state;
         struct { uint8_t spindle_mode, mist_coolant, flood_coolant; } gm; } cm;
uint8_t SREG;
#define PREP_BUFFER_OWNED_BY_LOADER 1
#define STAT_KILL_JOB 207
#define PSTR(x) (x)
struct {int buffer_state, block_type;} st_pre;
struct {int block_state;} mr;
struct {int controller_state;} cs;
static volatile uint8_t coolant_out_mist, coolant_out_flood;
static uint8_t output_mode; static float output_speed;

bool mp_hold_startup(void) {return false;}
void cm_alarm_P(int status, const char *message) {assert(0);}
static void _take_input_flush_request(void);
void xio_cancel_pgm_source(void) {}
void xio_flush_input(void) {}

static bool queued, busy, alarmed, aline_busy;
static uint32_t now;
static int starts, exits, stops, spindle_starts, off_calls, exec_requests;
static int replan_interrupt, replans;
static bool flush_interrupt;
static bool paused_before_service;
void cm_request_feedhold(void); void cm_request_cycle_feedhold(void); void cm_request_end_hold(void);
stat_t mp_replan_held_block(void) {
    assert(SREG & 128);  // serial reception must run during the whole planner walk
    assert(cm.hold_state==FEEDHOLD_HOLD && cm.program_paused==paused_before_service);
    replans++;
    switch(replan_interrupt) {
        case 1: cm_request_feedhold(); break; // RX '!'
        case 2: cm.queue_flush_state=FLUSH_REQUESTED; break; // RX '%'
        case 3: cm.job_kill_requested=true; break; // RX EOT
        case 4: cm.hold_fast_request=true; break; // limit ISR
        case 5: cm.hold_state=FEEDHOLD_PENDING; break;
        case 6: alarmed=true; break;
        case 7: cm_request_feedhold(); cm_request_end_hold(); break;
        case 8: cm.limit_requested=1; break;
        case 9: cm.shutdown_requested=1; break;
        case 10: return 110;
    }
    return STAT_OK;
}
void cm_start_hold(void); void cm_end_hold(void); void cm_queue_flush(void);
static void _stop_outputs(void);
void spindle_reset(void) { cm.gm.spindle_mode=output_mode=0; output_speed=0; off_calls++; }
void spindle_get_runtime(uint8_t *mode,float *speed) {*mode=output_mode;*speed=output_speed;}
void spindle_control_immediate(uint8_t mode,float speed) { assert(!(SREG&128)); cm.gm.spindle_mode=output_mode=mode; output_speed=speed; spindle_starts++; }
void _exec_flood_coolant_control(float *v,bool *f) { cm.gm.flood_coolant=coolant_out_flood=v[0]; if(!v[0]) cm.gm.mist_coolant=coolant_out_mist=0; }
void _exec_mist_coolant_control(float *v,bool *f) { cm.gm.mist_coolant=coolant_out_mist=v[0]; }
stat_t cm_is_alarmed(void) { return alarmed; }
bool cm_outputs_inhibited(void) { return alarmed || cm.queue_flush_state || cm.job_kill_requested || cm.limit_requested || cm.shutdown_requested; }
uint32_t SysTickTimer_getValue(void) { assert(!(SREG&128)); return now; }
bool mp_has_runnable_buffer(void) { return queued; }
bool mp_runtime_is_idle(void) { return !busy; }
uint8_t st_runtime_isbusy(void){return busy;}
bool st_aline_in_flight(void){return aline_busy;}
void st_cancel_dwell(void){if(!aline_busy)busy=false;}
void st_prep_null(void){}
void sr_request_status_report(int x){}
void mp_zero_segment_velocity(void){}
stat_t rpt_exception(int s,const char*m){return s;}
stat_t rpt_exception_P(int s,const char*m){return s;}
void cm_set_motion_state(int mode) { cm.motion_state=mode; }
void cm_cycle_start(void) { assert(!(SREG&128)); assert(cm.hold_state==FEEDHOLD_HOLD); starts++; }
void cm_cycle_end(void) { stops++; }
void mp_exit_hold_state(void) { assert(!(SREG&128)); exits++; cm.hold_state=FEEDHOLD_OFF; cm.motion_state=queued?MOTION_RUN:MOTION_STOP; }
void st_request_exec_move(void) { exec_requests++; }
void mp_flush_planner(void) { queued=false; cm.motion_state=MOTION_STOP;
 if(flush_interrupt){flush_interrupt=false;cm.queue_flush_state=FLUSH_REQUESTED;} }
void cm_set_position(uint8_t a,float p) {}
float mp_get_runtime_absolute_position(uint8_t a) { return 0; }
void qr_request_queue_report(int d) {}
'''+state+'\n'+'\n'.join(definitions)+r'''
static void setup(void) {
    memset(&cm,0,sizeof(cm)); hold_resume_cancelled=false;
    cm.gm.spindle_mode=output_mode=SPINDLE_CW; output_speed=1000;
    cm.gm.flood_coolant=cm.gm.mist_coolant=coolant_out_mist=coolant_out_flood=1;
    cm.hold_state=FEEDHOLD_HOLD; cm.motion_state=MOTION_HOLD; cm.program_paused=true;
    queued=true; busy=alarmed=aline_busy=false; now=100;
    memset(&st_pre,0,sizeof(st_pre));memset(&mr,0,sizeof(mr));
    flush_interrupt=false;replan_interrupt=replans=0; starts=exits=stops=spindle_starts=off_calls=exec_requests=0; SREG=0x95;
}
static void service(void) { paused_before_service=cm.program_paused; cm_feedhold_sequencing_callback(); assert(SREG==0x95); }
static void unchanged(void) {
    assert(output_mode==SPINDLE_CW && output_speed==1000);
    assert(coolant_out_mist && coolant_out_flood && off_calls==0 && spindle_starts==0);
}
int main(void) {
    setup();cm.program_paused=false;
    cm.hold_state=FEEDHOLD_PENDING;service();unchanged();
    cm.hold_state=FEEDHOLD_HOLD;service();unchanged();
    cm_request_end_hold();service();unchanged();
    assert(starts==1 && exits==1 && cm.hold_state==FEEDHOLD_OFF);
    setup();service();cm.queue_flush_state=FLUSH_REQUESTED;service();unchanged();
    assert(!queued && cm.hold_state==FEEDHOLD_OFF && !cm.end_hold_requested);
    // Every stop source during the interruptible planner walk vetoes motion release.
    for(int interrupt=1;interrupt<=10;interrupt++) {
        setup();service();replan_interrupt=interrupt;cm_request_end_hold();service();
        assert(replans==1 && !starts && !exits && !exec_requests && cm.program_paused);
        unchanged(); // alarm output shutdown belongs to the alarm entry, not this callback
        if(interrupt==1 || interrupt==7) {
            replan_interrupt=0;cm_request_end_hold();service();
            assert(starts==1 && replans==2);unchanged();
        }
    }
    // Masked callers must not enable interrupts or run the planner walk.
    setup();cm_request_end_hold();SREG=0x15;cm_end_hold();
    assert(SREG==0x15 && !replans && !starts);
    SREG=0x95;service();assert(replans==1 && starts==1);unchanged();
    // Runtime outputs stay unchanged even when the queued model already says M5/M9.
    setup();cm.gm.spindle_mode=cm.gm.mist_coolant=cm.gm.flood_coolant=0;
    service();cm_request_end_hold();service();unchanged();
    // A held lone dwell reports HOLD; its remaining timer reports RUN after resume.
    setup();queued=false;busy=true;cm.program_paused=false;
    cm.machine_state=MACHINE_PROGRAM_STOP;cm.cycle_state=CYCLE_OFF;
    service();assert(cm_get_combined_state()==COMBINED_HOLD);
    cm_request_end_hold();service();assert(exits==1 && !starts && busy);unchanged();
    assert(cm_get_combined_state()==COMBINED_RUN);
    busy=false;assert(cm_get_combined_state()==COMBINED_PROGRAM_STOP);
    // Explicit M0/M1 stays PROGRAM_STOP until resume, even with queued work.
    setup();cm.machine_state=MACHINE_PROGRAM_STOP;cm.cycle_state=CYCLE_OFF;
    service();assert(cm_get_combined_state()==COMBINED_PROGRAM_STOP);
    cm_request_end_hold();service();assert(!cm.program_paused && exits==1);
    assert(cm_get_combined_state()==COMBINED_RUN);unchanged();
    setup();queued=false;busy=true;cm.queue_flush_state=FLUSH_REQUESTED;service();
    assert(!busy && !queued && !starts);unchanged();
    setup();cm.hold_state=FEEDHOLD_OFF;cm.motion_state=MOTION_STOP;cm.cycle_state=0;
    cm_request_feedhold();assert(cm.hold_state==FEEDHOLD_OFF);unchanged();
    // Probe/homing contacts, operator holds, and direct or requested flushes preserve outputs.
    for(int cycle=CYCLE_HOMING;cycle<=CYCLE_PROBE;cycle++) {
        setup();cm.program_paused=false;cm.cycle_state=cycle;
        cm.hold_state=FEEDHOLD_OFF;cm.motion_state=MOTION_RUN;busy=aline_busy=true;
        cm_request_cycle_feedhold();cm.hold_state=FEEDHOLD_HOLD;busy=aline_busy=false;
        service();unchanged();cm_request_feedhold();service();unchanged();
        cm_request_end_hold();service();assert(exits==1);unchanged();
        setup();cm.cycle_state=cycle;flush_interrupt=true;cm_queue_flush();unchanged();
        assert(!queued && cm.queue_flush_state==FLUSH_OFF);
    }
    setup();cm_request_end_hold();cm_request_cycle_feedhold();service();
    assert(!cm.end_hold_requested && !starts);unchanged();
    // Alarm/kill entry retains an independent immediate output stop, with no restoration.
    setup();SREG=0x15;_stop_outputs();assert(SREG==0x15);
    assert(!output_mode && !coolant_out_mist && !coolant_out_flood);
    SREG=0x95;cm_request_end_hold();service();assert(!output_mode && !coolant_out_flood);
}
'''
        # Use firmware state values for the real combined-state getter alongside E060 sequencing.
        header = (FIRMWARE/'canonical_machine.h').read_text()
        enums = ''
        for name in ('cmCombinedState', 'cmMachineState', 'cmCycleState', 'cmMotionState'):
            definition = re.search(r'typedef enum \{[^{}]*\}\s*' + name + r';', header).group()
            for token in re.findall(r'\b(?:COMBINED|MACHINE|CYCLE|MOTION)_[A-Z_]+', definition):
                code = re.sub(r'(?m)^#define ' + token + r' .*\n', '', code)
            enums += definition + '\n'
        code = enums + code
        with tempfile.TemporaryDirectory() as d:
            c=Path(d)/'check.c'; c.write_text(OVERRIDE_STUBS+code); out=Path(d)/'check'
            subprocess.run(['cc','-std=c99','-g','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(c),'-o',str(out)],check=True)
            subprocess.run([str(out)],check=True)


if __name__=='__main__': unittest.main()
