"""Deterministic hold/output sequencing; physical timing requires external capture."""
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
    def test_sequence(self):
        source=(FIRMWARE/'canonical_machine.c').read_text()
        definitions=[]
        for name in ['_hold_outputs_cancel', '_hold_outputs_pause', '_hold_outputs_resume', '_hold_outputs_finish_resume',
                     'cm_start_hold', 'cm_end_hold', 'cm_queue_flush',
                     '_take_input_flush_request', '_service_input_flush', '_run_requested_queue_flush',
                     'cm_feedhold_sequencing_callback', 'cm_request_feedhold', 'cm_request_end_hold']:
            signature=r'(?m)^(?:static )?(?:void|stat_t|bool) '+name+r'\([^;{}\n]*\)\s*'
            match=re.search(signature+r'\{', source)
            self.assertIsNotNone(match,name)
            definitions.append(function(source[match.start():],signature))
        state=re.search(r'static struct \{[^}]*\} hold_outputs;',source).group()
        code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#define __AVR 1
#define cli() (SREG &= ~128)
#define STAT_OK 0
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
#define AXES 6
#define FLAGS_NONE ((bool*)0)
typedef int stat_t;
struct { unsigned hold_state, motion_state, queue_flush_state; bool end_hold_requested, program_paused, job_kill_requested, queue_flush_input, queue_flush_input_due, hold_fast_request;
         struct { uint8_t spindle_mode, mist_coolant, flood_coolant; } gm; } cm;
struct { bool pause_on_hold, coolant_pause_on_hold; float dwell_seconds; } spindle_cfg;
uint8_t SREG;
#define PREP_BUFFER_OWNED_BY_LOADER 1
#define STAT_KILL_JOB 207
#define PSTR(x) (x)
struct {int buffer_state;} st_pre;

bool mp_hold_startup(void) {return false;}
void cm_alarm_P(int status, const char *message) {assert(0);}
static void _take_input_flush_request(void);
void xio_flush_input(void) {}

static bool queued, busy, alarmed;
static uint32_t now;
static int starts, exits, stops, spindle_starts, off_calls, exec_requests;
static int replan_interrupt, replans;
static bool paused_before_service;
void cm_request_feedhold(void); void cm_request_end_hold(void);
void mp_replan_held_block(void) {
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
    }
}
void cm_start_hold(void); void cm_end_hold(void); void cm_queue_flush(void);
static void _hold_outputs_cancel(void);
static void _hold_outputs_pause(void);
static bool _hold_outputs_resume(void);
void spindle_reset(void) { cm.gm.spindle_mode=0; off_calls++; }
void spindle_control_immediate(uint8_t mode) { assert(!(SREG&128)); cm.gm.spindle_mode=mode; spindle_starts++; }
void _exec_flood_coolant_control(float *v,bool *f) { cm.gm.flood_coolant=v[0]; if(!v[0]) cm.gm.mist_coolant=0; }
void _exec_mist_coolant_control(float *v,bool *f) { cm.gm.mist_coolant=v[0]; }
stat_t cm_is_alarmed(void) { return alarmed; }
uint32_t SysTickTimer_getValue(void) { assert(!(SREG&128)); return now; }
bool mp_has_runnable_buffer(void) { return queued; }
bool mp_runtime_is_idle(void) { return !busy; }
void cm_set_motion_state(int mode) { cm.motion_state=mode; }
void cm_cycle_start(void) { assert(!(SREG&128)); assert(cm.hold_state==FEEDHOLD_HOLD); starts++; }
void cm_cycle_end(void) { stops++; }
void mp_exit_hold_state(void) { assert(!(SREG&128)); exits++; cm.hold_state=FEEDHOLD_OFF; cm.motion_state=queued?MOTION_RUN:MOTION_STOP; }
void st_request_exec_move(void) { exec_requests++; }
void mp_flush_planner(void) { queued=false; cm.motion_state=MOTION_STOP; }
void cm_set_position(uint8_t a,float p) {}
float mp_get_runtime_absolute_position(uint8_t a) { return 0; }
void qr_request_queue_report(int d) {}
'''+state+'\n'+'\n'.join(definitions)+r'''
static void setup(void) {
    memset(&cm,0,sizeof(cm)); memset(&hold_outputs,0,sizeof(hold_outputs));
    spindle_cfg.pause_on_hold=true; spindle_cfg.coolant_pause_on_hold=true; spindle_cfg.dwell_seconds=1;
    cm.gm.spindle_mode=SPINDLE_CW; cm.gm.flood_coolant=cm.gm.mist_coolant=1;
    cm.hold_state=FEEDHOLD_HOLD; cm.motion_state=MOTION_HOLD; cm.program_paused=true;
    queued=true; busy=alarmed=false; now=100;
    replan_interrupt=replans=0; starts=exits=stops=spindle_starts=off_calls=exec_requests=0; SREG=0x95;
}
static void service(void) { paused_before_service=cm.program_paused; cm_feedhold_sequencing_callback(); assert(SREG==0x95); }
int main(void) {
    setup();
    cm.hold_state=FEEDHOLD_PENDING; service();
    assert(cm.gm.spindle_mode==SPINDLE_CW && cm.gm.flood_coolant);
    cm.hold_state=FEEDHOLD_HOLD; service();
    assert(!cm.gm.spindle_mode && !cm.gm.mist_coolant && !cm.gm.flood_coolant);
    service(); assert(off_calls==1); // saved state isn't overwritten by repeated polling
    cm_request_end_hold(); service();
    assert(cm.gm.spindle_mode==SPINDLE_CW && cm.hold_state==FEEDHOLD_HOLD && starts==0);
    assert(!cm.gm.flood_coolant && !cm.gm.mist_coolant);
    now+=1000; cm_request_end_hold(); service();
    assert(starts==0 && spindle_starts==1); // repeated ~ doesn't restart or bypass delay
    now+=10; service();
    assert(starts==1 && exits==1 && !cm.end_hold_requested);
    assert(cm.gm.flood_coolant && cm.gm.mist_coolant && !hold_outputs.state);
    setup(); service(); cm_request_end_hold(); service(); now+=500;
    cm_request_feedhold(); service(); // a new ! cancels spin-up and stops output again
    assert(!cm.end_hold_requested && !cm.gm.spindle_mode && hold_outputs.state==1);
    now+=5000; service(); assert(!starts);
    cm_request_end_hold(); service(); now+=1010; service(); assert(starts==1);
    setup(); now=UINT32_MAX-100; service(); cm_request_end_hold(); service();
    now+=1000; service(); assert(!starts); now+=10; service(); assert(starts==1);
    setup(); spindle_cfg.pause_on_hold=false; spindle_cfg.coolant_pause_on_hold=false;
    service(); assert(cm.gm.spindle_mode==SPINDLE_CW && cm.gm.flood_coolant);
    cm_request_end_hold(); service(); assert(starts==1 && !spindle_starts);
    setup(); cm.gm.spindle_mode=SPINDLE_OFF; service(); cm_request_end_hold(); service();
    assert(starts==1 && !spindle_starts); // no spindle means no needless delay
    setup(); spindle_cfg.dwell_seconds=0; service(); cm_request_end_hold(); service(); assert(starts==1);
    for(int during_spinup=0;during_spinup<2;during_spinup++) {
        setup(); service(); if(during_spinup) { cm_request_end_hold(); service(); }
        cm.queue_flush_state=FLUSH_REQUESTED; service();
        assert(!queued && !cm.gm.spindle_mode && !cm.gm.flood_coolant && !cm.gm.mist_coolant);
        assert(!hold_outputs.state && !cm.end_hold_requested && !starts);
        now+=10000; service(); assert(!cm.gm.spindle_mode && !starts);
        setup(); service(); if(during_spinup) { cm_request_end_hold(); service(); }
        alarmed=true; service(); assert(!cm.gm.spindle_mode && !hold_outputs.state);
        assert(!exits && cm.hold_state==FEEDHOLD_HOLD);
        // Exception paths discard saved modes; clear+~ cannot resurrect prior outputs.
        alarmed=false; queued=true; cm.hold_state=FEEDHOLD_HOLD;
        cm_request_end_hold(); service(); assert(!cm.gm.spindle_mode && !cm.gm.flood_coolant);
    }
    // Interrupts during the planner walk must veto the hand-over without losing saved outputs.
    for(int interrupt=1;interrupt<=7;interrupt++) {
        setup(); service(); cm_request_end_hold(); service(); now+=1010;
        replan_interrupt=interrupt; service();
        assert(replans==1 && !starts && !exits && !exec_requests && cm.program_paused);
        assert(!cm.gm.mist_coolant && !cm.gm.flood_coolant);
        if(interrupt==1 || interrupt==7) {
            assert(!cm.gm.spindle_mode && hold_outputs.state==1);
            replan_interrupt=0; cm_request_end_hold(); service();
            now+=1000; service(); assert(!starts);
            now+=10; service(); assert(starts==1 && replans==2);
        }
    }
    // A masked caller must defer the expensive walk, never force interrupts on.
    setup(); service(); cm_request_end_hold(); service(); now+=1010;
    SREG=0x15; cm_end_hold(); assert(SREG==0x15 && !replans && !starts);
    SREG=0x95; service(); assert(replans==1 && starts==1);
    // Already-masked callers must remain masked after nested cancellation.
    setup(); SREG=0x15; _hold_outputs_cancel(); assert(SREG==0x15);
}
'''
        with tempfile.TemporaryDirectory() as d:
            c=Path(d)/'check.c'; c.write_text(code); out=Path(d)/'check'
            subprocess.run(['cc','-std=c99','-g','-fsanitize=address,undefined',str(c),'-o',str(out)],check=True)
            subprocess.run([str(out)],check=True)


if __name__=='__main__': unittest.main()
