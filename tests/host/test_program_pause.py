"""Compile the program-pause transitions and executor gate with deterministic queue fixtures.

The runtime finalizer and physical step generator are fixtures; no timing claim is made.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


def function(source, name):
    match = re.search(r"^(?:static )?(?:stat_t|void) " + name + r"\([^;]*?\)\s*\{", source, re.M)
    assert match, name
    depth = 1
    tokens = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*.*?\*/|[{}]', re.S)
    for token in tokens.finditer(source, match.end()):
        depth += (token.group() == '{') - (token.group() == '}')
        if depth == 0:
            return source[match.start():token.end()]
    raise AssertionError(name)


class ProgramPause(unittest.TestCase):
    def test_explicit_stop_and_resume(self):
        canonical = (ROOT / 'firmware/tinyg/canonical_machine.c').read_text()
        executor = (ROOT / 'firmware/tinyg/plan_exec.c').read_text()
        code = r'''
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <stddef.h>
#define __AVR 1
uint8_t SREG=128;
#define cli() (SREG=0)
#define AXES 6
#define AXIS_X 0
#define STAT_OK 0
#define STAT_NOOP 3
#define STAT_INTERNAL_ERROR 20
#define MACHINE_PROGRAM_STOP 3
#define MACHINE_CYCLE 5
#define MACHINE_ALARM 6
#define MOTION_STOP 0
#define MOTION_RUN 1
#define MOTION_HOLD 2
#define CYCLE_OFF 0
#define CYCLE_MACHINING 1
#define FEEDHOLD_OFF 0
#define FEEDHOLD_HOLD 6
#define FLUSH_OFF 0
#define SR_REQUEST_ASAP 1
#define BLOCK_TYPE_ALINE 1
#define PSTR(x) (x)
typedef int stat_t;
struct {int machine_state, cycle_state, motion_state, hold_state, queue_flush_state;
        bool program_paused, end_hold_requested; uint8_t optional_stop_enabled;} cm;
static bool flags_one[AXES]={true};
#define FLAGS_ONE flags_one
static int fetched, ran, wakeups, finalized, queued_commands;
static bool queued, idle=true;
typedef struct mpBuf {int block_type; stat_t (*bf_func)(struct mpBuf *);} mpBuf_t;
mpBuf_t block;
mpBuf_t *mp_get_run_buffer(void) {fetched++; return queued ? &block : NULL;}
bool mp_has_runnable_buffer(void) {return queued;}
bool mp_runtime_is_idle(void) {return idle;}
void st_prep_null(void) {}
void mp_replan_held_block(void) {assert(!cm.program_paused && cm.hold_state==FEEDHOLD_HOLD);}
void st_request_exec_move(void) {wakeups++;}
void cm_set_motion_state(int state) {cm.motion_state=state;}
stat_t cm_panic_P(int status,const char *msg) {(void)msg;return status;}
void sr_request_status_report(int state) {(void)state;}
void qr_init_queue_report(void) {assert(SREG==0);}
void qr_request_queue_report(int n) {(void)n;}
void mp_flush_planner(void) {queued=false;}
float mp_get_runtime_absolute_position(int axis) {(void)axis;return 0;}
void cm_set_position(int axis,float pos) {(void)axis;(void)pos;}
void (*stop_callback)(float *, bool *);
float queued_value[AXES];
void mp_queue_command(void (*fn)(float *,bool *),float *value,bool *flags) {
    assert(flags==FLAGS_ONE); queued_commands++; stop_callback=fn;
    for(int i=0;i<AXES;i++) queued_value[i]=value[i];
}
static void _exec_program_finalize(float *value,bool *flags) {
    assert(SREG==0); (void)flags; finalized++; cm.machine_state=(int)value[0];
    cm.cycle_state=CYCLE_OFF; cm.motion_state=MOTION_STOP;
    cm.hold_state=FEEDHOLD_OFF; cm.end_hold_requested=false;
}
void cm_cycle_start(void);
void cm_cycle_end(void);
void mp_exit_hold_state(void);
static stat_t run(mpBuf_t *bf) {(void)bf;ran++;return STAT_OK;}
'''
        for name in ('_exec_program_stop', 'cm_program_stop', 'cm_optional_program_stop',
                     'cm_cycle_start', 'cm_cycle_end', 'cm_request_end_hold', 'cm_end_hold', 'cm_queue_flush'):
            code += '\n' + function(canonical, name)
        for name in ('mp_exec_move', 'mp_exit_hold_state'):
            code += '\n' + function(executor, name)
        code += r'''
void run_stop_callback(void) {
    uint8_t previous=SREG; SREG=0;
    stop_callback(queued_value,FLAGS_ONE);
    SREG=previous;
}
int main(void) {
    block.bf_func=run; queued=true;
    for(int type=1;type<=3;type++) {
        block.block_type=type; cm.program_paused=false;
        cm.hold_state=FEEDHOLD_OFF; cm.cycle_state=CYCLE_MACHINING;
        cm_program_stop();
        assert(!cm.program_paused); // reception is not the runtime boundary
        for(int i=1;i<AXES;i++) assert(queued_value[i]==0);
        run_stop_callback();
        assert(cm.program_paused && cm.hold_state==FEEDHOLD_HOLD && cm.motion_state==MOTION_HOLD);
        int previous_fetched=fetched, previous_ran=ran, previous_finalized=finalized;
        for(int i=0;i<20;i++) {
            cm_cycle_start(); // planning a later line cannot resume execution
            cm_cycle_end();   // a no-op/zero-length line cannot clear the pause
            assert(SREG==128);
            assert(mp_exec_move()==STAT_NOOP);
        }
        assert(fetched==previous_fetched && ran==previous_ran && finalized==previous_finalized);
        assert(cm.machine_state==MACHINE_PROGRAM_STOP);
        cm_request_end_hold(); assert(cm.end_hold_requested);
        cm_end_hold();
        assert(!cm.program_paused && !cm.end_hold_requested && cm.hold_state==FEEDHOLD_OFF);
        assert(mp_exec_move()==STAT_OK && ran==previous_ran+1);
    }
    // M1 is ignored with the switch off; it must not even queue a synchronization barrier.
    int previous_commands=queued_commands;
    cm.optional_stop_enabled=0; cm_optional_program_stop();
    assert(queued_commands==previous_commands && !cm.program_paused);
    cm.optional_stop_enabled=1;
    // Empty-queue resume does not leave a pause on the next job.
    cm_optional_program_stop(); run_stop_callback(); queued=false;
    cm_request_end_hold(); cm_end_hold();
    assert(!cm.program_paused && cm.motion_state==MOTION_STOP);
    queued=true; assert(mp_exec_move()==STAT_OK);
    // Ordinary cycle completion remains automatic, never an explicit pause.
    cm.cycle_state=CYCLE_MACHINING; cm_cycle_end();
    assert(!cm.program_paused && cm.hold_state==FEEDHOLD_OFF);
    assert(mp_exec_move()==STAT_OK);
    // Nested callers must not accidentally re-enable interrupts.
    SREG=0; cm_cycle_start(); assert(SREG==0); cm_cycle_end(); assert(SREG==0); SREG=128;
    // A flush releases the explicit pause after clearing queued work.
    cm_program_stop(); run_stop_callback();
    cm.queue_flush_state=1; cm_queue_flush();
    assert(!cm.program_paused && !queued && cm.hold_state==FEEDHOLD_OFF && cm.queue_flush_state==0);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / 'pause.c'
            c.write_text(code)
            exe = Path(tmp) / 'pause'
            subprocess.run(['cc', '-std=c99', '-Wall', '-Werror', '-fsanitize=address,undefined',
                            str(c), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
