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
struct { bool pause_on_hold, coolant_pause_on_hold; float dwell_seconds; } spindle_cfg;
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
static void _hold_outputs_cancel(void);
static void _hold_outputs_pause(void);
static bool _hold_outputs_resume(void);
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
    memset(&cm,0,sizeof(cm)); memset(&hold_outputs,0,sizeof(hold_outputs));
    spindle_cfg.pause_on_hold=true; spindle_cfg.coolant_pause_on_hold=true; spindle_cfg.dwell_seconds=1;
    cm.gm.spindle_mode=output_mode=SPINDLE_CW; output_speed=1000;
    cm.gm.flood_coolant=cm.gm.mist_coolant=coolant_out_mist=coolant_out_flood=1;
    cm.hold_state=FEEDHOLD_HOLD; cm.motion_state=MOTION_HOLD; cm.program_paused=true;
    queued=true; busy=alarmed=aline_busy=false; now=100;
    memset(&st_pre,0,sizeof(st_pre));memset(&mr,0,sizeof(mr));
    flush_interrupt=false;replan_interrupt=replans=0; starts=exits=stops=spindle_starts=off_calls=exec_requests=0; SREG=0x95;
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
    // Disabling pause also applies when a new ! interrupts the resume planner walk.
    setup(); spindle_cfg.pause_on_hold=false; service();
    replan_interrupt=1; cm_request_end_hold(); service();
    assert(!starts&&output_mode==SPINDLE_CW&&hold_outputs.state==1);
    replan_interrupt=0; cm_request_end_hold(); service(); assert(starts==1&&output_mode==SPINDLE_CW);
    setup(); cm.gm.spindle_mode=output_mode=SPINDLE_OFF; service(); cm_request_end_hold(); service();
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
    for(int interrupt=1;interrupt<=10;interrupt++) {
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
    // The model may already contain queued M5/M30 state: capture the actual running outputs and speed.
    setup(); cm.gm.spindle_mode=0;cm.gm.mist_coolant=cm.gm.flood_coolant=0;
    service(); assert(hold_outputs.spindle_mode==SPINDLE_CW&&hold_outputs.spindle_speed==1000);
    assert(hold_outputs.mist&&hold_outputs.flood);
    cm_request_end_hold();service();now+=1010;service();
    assert(output_mode==SPINDLE_CW&&output_speed==1000&&coolant_out_mist&&coolant_out_flood);
    // A held lone dwell has no queued buffer; resume still restores the spindle and respects spin-up.
    setup();queued=false;busy=true;cm.program_paused=false;
    cm.machine_state=MACHINE_PROGRAM_STOP;cm.cycle_state=CYCLE_OFF;
    service();cm_request_end_hold();service();
    assert(cm_get_combined_state()==COMBINED_HOLD); // E060 keeps hold latched during spindle restart
    assert(!exits&&hold_outputs.state==2&&busy);
    now+=1010;service();assert(exits==1&&!starts&&output_mode==SPINDLE_CW&&busy);
    assert(cm_get_combined_state()==COMBINED_RUN); // remaining G4 timer wins over stored STOP
    busy=false;assert(cm_get_combined_state()==COMBINED_PROGRAM_STOP);
    // M0 and enabled M1 share the explicit program-pause latch, even with a dwell queued.
    setup();cm.machine_state=MACHINE_PROGRAM_STOP;cm.cycle_state=CYCLE_OFF;
    assert(queued && cm.program_paused);
    assert(cm_get_combined_state()==COMBINED_PROGRAM_STOP);
    service();cm_request_end_hold();service();
    assert(cm_get_combined_state()==COMBINED_PROGRAM_STOP); // pause remains through spindle delay
    now+=1010;service();assert(!cm.program_paused && exits==1);
    assert(cm_get_combined_state()==COMBINED_RUN);
    // A flush of that dwell cancels the timer and all saved outputs, without a spin-up or motion release in ALARM.
    setup();queued=false;busy=true;service();cm.queue_flush_state=FLUSH_REQUESTED;service();
    assert(!busy&&!hold_outputs.state&&!output_mode&&!cm.end_hold_requested);
    setup();busy=true;service();alarmed=true;cm.queue_flush_state=FLUSH_REQUESTED;service();
    assert(!queued&&!busy&&!output_mode&&!starts&&cm.motion_state==MOTION_STOP);
    assert(cm.hold_state==FEEDHOLD_OFF&&alarmed&&!hold_outputs.state);
    // An idle '!' outside a cycle stays a no-op.
    setup();cm.hold_state=FEEDHOLD_OFF;cm.motion_state=MOTION_STOP;cm.cycle_state=0;
    cm_request_feedhold();assert(cm.hold_state==FEEDHOLD_OFF&&output_mode==SPINDLE_CW);
    // Do not snapshot ahead of an already prepared output command.
    setup();st_pre.buffer_state=PREP_BUFFER_OWNED_BY_LOADER;st_pre.block_type=BLOCK_TYPE_COMMAND;
    service();assert(!hold_outputs.state&&output_mode==SPINDLE_CW);
    output_mode=cm.gm.spindle_mode=SPINDLE_OFF;st_pre.buffer_state=0;service();
    cm_request_end_hold();service();assert(exits==1&&!output_mode);
    // A canned cycle may finish at its contact hold without an explicit resume.
    // Its saved output modes must not leak into the next job's hold.
    setup();service();queued=false;
    float ended[]={MACHINE_PROGRAM_STOP};_exec_program_finalize(ended,FLAGS_NONE);
    assert(!hold_outputs.state&&!cm.end_hold_requested&&cm.hold_state==FEEDHOLD_OFF);
    output_mode=cm.gm.spindle_mode=SPINDLE_CCW;output_speed=2000;
    queued=true;cm.hold_state=FEEDHOLD_HOLD;service();
    assert(hold_outputs.spindle_mode==SPINDLE_CCW&&hold_outputs.spindle_speed==2000);
    // A probe/homing contact stops the cycle's move, not the spindle or vacuum.
    for(int cycle=CYCLE_HOMING;cycle<=CYCLE_PROBE;cycle++) {
        setup();cm.program_paused=false;cm.cycle_state=cycle;
        cm.hold_state=FEEDHOLD_OFF;cm.motion_state=MOTION_RUN;busy=aline_busy=true;
        cm_request_cycle_feedhold();cm.hold_state=FEEDHOLD_HOLD;busy=aline_busy=false;
        service();assert(output_mode==SPINDLE_CW&&coolant_out_mist&&coolant_out_flood&&!hold_outputs.state);
        // A later operator ! upgrades the internal hold, so the normal pause policy applies.
        cm_request_feedhold();service();assert(output_mode==SPINDLE_OFF&&hold_outputs.state==1);
        cm_request_end_hold();service();now+=1010;service();assert(exits==1&&output_mode==SPINDLE_CW);
        setup();cm.program_paused=false;cm.cycle_state=cycle;
        cm.hold_state=FEEDHOLD_OFF;cm.motion_state=MOTION_RUN;
        cm_request_cycle_feedhold();cm.hold_state=FEEDHOLD_HOLD;
        service();alarmed=true;service();assert(!output_mode&&!coolant_out_mist&&!coolant_out_flood);
    }
    // Direct homing housekeeping may flush a contact hold between phases without an output pause.
    setup();cm.program_paused=false;cm.cycle_state=CYCLE_HOMING;
    cm.hold_state=FEEDHOLD_OFF;cm.motion_state=MOTION_RUN;
    cm_request_cycle_feedhold();cm.hold_state=FEEDHOLD_HOLD;
    service();cm_queue_flush();assert(output_mode==SPINDLE_CW&&coolant_out_mist&&coolant_out_flood);
    // A host abort arriving during the unmasked planner flush must not be consumed as housekeeping.
    setup();cm.program_paused=false;cm.cycle_state=CYCLE_HOMING;
    cm.hold_state=FEEDHOLD_OFF;cm.motion_state=MOTION_RUN;
    cm_request_cycle_feedhold();cm.hold_state=FEEDHOLD_HOLD;
    flush_interrupt=true;cm_queue_flush();assert(!output_mode&&!coolant_out_flood);
    // An explicit host flush during the same internal hold still discards and turns everything off.
    setup();cm.program_paused=false;cm.cycle_state=CYCLE_HOMING;
    cm.hold_state=FEEDHOLD_OFF;cm.motion_state=MOTION_RUN;
    cm_request_cycle_feedhold();cm.hold_state=FEEDHOLD_HOLD;
    cm.queue_flush_state=FLUSH_REQUESTED;service();assert(!output_mode&&!coolant_out_flood&&!hold_outputs.state);
    // An internal contact may not downgrade an operator pause already in progress.
    setup();service();cm_request_cycle_feedhold();service();assert(!output_mode&&hold_outputs.state==1);
    // Contact during an operator spin-up must still cancel resume, without changing pause ownership.
    setup();service();cm_request_end_hold();service();
    assert(output_mode==SPINDLE_CW&&hold_outputs.state==2);
    cm_request_cycle_feedhold();service();
    assert(!cm.end_hold_requested&&!starts&&!output_mode&&hold_outputs.state==1);
    // Resume/flush/alarm continue to work with a pending probe block behind an ordinary line.
    setup();cm.program_paused=false;service();alarmed=true;cm.queue_flush_state=FLUSH_REQUESTED;
    service();assert(!queued&&!output_mode&&!hold_outputs.state&&cm.hold_state==FEEDHOLD_OFF);
    cm.queue_flush_input=true;service(); // a late host '%' cannot resurrect output modes
    alarmed=false;service();assert(!output_mode&&!cm.end_hold_requested);
    // Already-masked callers must remain masked after nested cancellation.
    setup(); SREG=0x15; _hold_outputs_cancel(); assert(SREG==0x15);
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
            c=Path(d)/'check.c'; c.write_text(code); out=Path(d)/'check'
            subprocess.run(['cc','-std=c99','-g','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(c),'-o',str(out)],check=True)
            subprocess.run([str(out)],check=True)


if __name__=='__main__': unittest.main()
