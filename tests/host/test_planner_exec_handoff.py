"""E180: the exec never sees a block half replanned, and a started block is never rewritten.

The real planner.c, plan_line.c and plan_zoid.c are linked. plan_zoid.c is built with sqrt() routed
through a hook, so an "exec interrupt" can arrive in the middle of mp_calculate_trapezoid(), as the
real exec interrupt can. At every such moment each committed block the exec could start must hold a
plan whose head, body and tail add up to its length (a block caught part way used to read as zero
length and its move was lost). Once, the hook also starts the first queued block, as the exec does
(block_state RUNNING); the rest of the replan must leave that block's plan untouched.

With the real plan_exec.c linked, and plan_line.c built with a call placed right after the block's new
plan is computed (before it is committed), the real mp_exec_move() runs there: a short body-only block
then runs and is freed inside its own replan. Its cleared slot must stay cleared, and the
block after it must enter at the exit velocity the exec actually used.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = Path(os.environ.get('TINYG_FIRMWARE', ROOT / 'firmware/tinyg'))
FIXTURE = Path(__file__).parent / 'planner_startup'

CHECK = r'''
#include <assert.h>
#include <math.h>
#include <avr/interrupt.h>
#include "tinyg.h"
#include "config.h"
#include "controller.h"
#include "canonical_machine.h"
#include "planner.h"
#include "plan_arc.h"
#include "kinematics.h"
#include "stepper.h"
#include "encoder.h"
#include "report.h"
#include "xmega/xmega_rtc.h"

cmSingleton_t cm;
stPrepSingleton_t st_pre;
float st_get_move_time(const float axis_length[]) { return 0; }
rtClock_t rtc;
controller_t cs;
uint8_t SREG;
char text_item[256];

uint8_t cm_get_path_control(const GCodeState_t *gcode_state) { return (PATH_CONTINUOUS); }
void cm_set_motion_state(const cmMotionState motion_state) { cm.motion_state = motion_state; }
void cm_cycle_end(void) {}
void cm_abort_arc(void) {}
stat_t cm_is_alarmed(void) { return cm.machine_state == MACHINE_ALARM; }
stat_t cm_panic_P(const stat_t status, const char *msg_P) { return (status); }
void en_set_encoder_steps(uint8_t motor, float steps) {}
void kn_inverse_kinematics(const float travel[], float steps[]) {}
stat_t rpt_exception(stat_t status, const char *msg) { return (status); }
stat_t rpt_exception_P(stat_t status, const char *msg) { return (status); }
void qr_request_queue_report(int8_t buffers) {}
bool st_aline_in_flight(void) { return true; }
uint8_t st_runtime_isbusy(void) { return true; }       /* the exec is busy: blocks stay queued */
void st_request_exec_move(void) {}
void st_prep_null(void) {}
void st_prep_command(void *bf) {}
void st_prep_dwell(float microseconds) {}
void stepper_reset(void) {}
stat_t mp_exec_aline(mpBuf_t *bf) { return STAT_OK; }
void cm_request_feedhold(void) {}
stat_t mp_exec_move(void) { return STAT_NOOP; }

static int interrupts, inconsistent, start_once;
static mpBuf_t *started;
static float started_plan[7];

static void plan_of(const mpBuf_t *bf, float out[7]) {
    float v[7] = {bf->entry_velocity, bf->cruise_velocity, bf->exit_velocity,
                  bf->head_length, bf->body_length, bf->tail_length, bf->naiive_move_time};
    memcpy(out, v, sizeof(v));
}

/* what the exec interrupt can see at this instant */
static void exec_interrupt(void) {
    interrupts++;
    for (int i = 0; i < PLANNER_BUFFER_POOL_SIZE; i++) {
        mpBuf_t *bf = &mb.bf[i];
        if (bf->buffer_state < MP_BUFFER_QUEUED || bf->block_type != BLOCK_TYPE_ALINE) continue;
        if (fabsf(bf->head_length + bf->body_length + bf->tail_length - bf->length) > 1e-4f * (1 + bf->length)) {
            inconsistent++;
        }
    }
    if (start_once && started == NULL) {                /* the exec starts the next block now */
        mpBuf_t *bf = mp_get_run_buffer();
        if (bf != NULL) { bf->replannable = false; bf->block_state = BLOCK_RUNNING; started = bf; plan_of(bf, started_plan); }
    }
}
double hook_sqrt(double x) { exec_interrupt(); return sqrt(x); }

static void setup(void) {
    memset(&cm, 0, sizeof(cm)); memset(&st_pre, 0, sizeof(st_pre));
    planner_init();
    cm.machine_state = MACHINE_READY; cm.hold_state = FEEDHOLD_OFF; cm.cycle_state = CYCLE_MACHINING;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;
    for (int a = 0; a < AXES; a++) {
        cm.a[a].velocity_max = cm.a[a].feedrate_max = 800;
        cm.a[a].jerk_max = 20; cm.a[a].recip_jerk = 1.0f / (20 * JERK_MULTIPLIER);
        cm.a[a].junction_dev = .05;
    }
    cm.junction_acceleration = 100000;
    rtc.sys_ticks = 100; SREG = 0x95;
}
static void line_to(float x, float y) {
    GCodeState_t gm = {0}; gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;
    gm.feed_rate_mode = UNITS_PER_MINUTE_MODE; gm.feed_rate = 800; gm.target[0] = x; gm.target[1] = y;
    assert(mp_aline(&gm) == STAT_OK); assert(SREG == 0x95);
}

int main(void) {
    /* dense outline: short segments turning gently, each replan rewrites the queued blocks */
    setup();
    float x = 0, y = 0;
    for (int i = 0; i < 20; i++) { x += 0.08f * cosf(i * 0.05f); y += 0.08f * sinf(i * 0.05f); line_to(x, y); }
    assert(interrupts > 0);
    assert(inconsistent == 0);

    /* the exec starts the queued block (planned to stop at its end) in the middle of the replan
       that a following collinear block triggers, which would raise its exit velocity */
    setup(); interrupts = inconsistent = 0;
    line_to(2.0f, 0);
    float alone[7]; plan_of(mp_get_run_buffer(), alone);
    start_once = 1; started = NULL;
    line_to(4.0f, 0);
    assert(started != NULL);
    float now[7]; plan_of(started, now);
    assert(memcmp(now, started_plan, sizeof(now)) == 0);  /* the started block keeps the plan it started with */
    assert(inconsistent == 0);
    /* control: without the start, the same replan does change that block's plan */
    setup(); line_to(2.0f, 0); line_to(4.0f, 0);
    float replanned[7]; plan_of(mp_get_run_buffer(), replanned);
    assert(memcmp(alone, replanned, sizeof(alone)) != 0);
    return 0;
}
'''


REAL_EXEC = r'''
#include "util.h"
stat_t sr_request_status_report(uint8_t request_type) { return STAT_OK; }
float en_read_encoder(uint8_t motor) { return 0; }
stat_t st_prep_line(float travel_steps[], float following_error[], float segment_time) { return STAT_OK; }
stat_t status_code;
static int inject, did, exec_status;
static mpBuf_t *victim;
static float used_exit;
double hook_sqrt(double x) { return sqrt(x); }
void hook_after_plan(mpBuf_t *bp) {          /* the exec runs (and finishes) the block being replanned */
    if (inject && bp == victim) {
        inject = 0; did = 1; used_exit = victim->exit_velocity;
        st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;
        exec_status = mp_exec_move();
    }
}
static stat_t line_to(float x, float feed) {
    GCodeState_t gm = {0}; gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;
    gm.feed_rate_mode = UNITS_PER_MINUTE_MODE; gm.feed_rate = feed; gm.target[0] = x;
    return mp_aline(&gm);
}
/* a 4.1 ms body-only first block (0.000203373 mm at 800 mm/min, jerk 20000) that the exec runs and
   frees right after its new plan is computed; then a following block of next_len at next_feed */
static void run_case(float next_len, float next_feed) {
    const float len = 0.000203373f;
    memset(&cm, 0, sizeof(cm)); memset(&st_pre, 0, sizeof(st_pre)); planner_init();
    cm.machine_state = MACHINE_READY; cm.hold_state = FEEDHOLD_OFF; cm.cycle_state = CYCLE_MACHINING;
    cm.motion_state = MOTION_RUN; st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;
    for (int a = 0; a < AXES; a++) {
        cm.a[a].velocity_max = cm.a[a].feedrate_max = 800; cm.a[a].jerk_max = 20000;
        cm.a[a].recip_jerk = 1 / (20000 * JERK_MULTIPLIER); cm.a[a].junction_dev = .05;
    }
    cm.junction_acceleration = 100000; rtc.sys_ticks = 100; SREG = 0x95; did = 0;
    assert(line_to(len, 800) == STAT_OK);
    victim = mp_get_run_buffer();
    assert(victim && victim->head_length == 0 && victim->tail_length == 0);
    assert(victim->body_length / victim->cruise_velocity <= NOM_SEGMENT_TIME);
    inject = 1;
    assert(line_to(len + next_len, next_feed) == STAT_OK);
    assert(did && exec_status == STAT_OK);
    assert(victim->buffer_state == MP_BUFFER_EMPTY && victim->length == 0 && victim->exit_velocity == 0);
    mpBuf_t *next = mp_get_run_buffer();
    assert(next && next != victim);
    if (fabsf(next->entry_velocity - used_exit) >= 1e-3f) {
        fprintf(stderr, "next %g at F%g: entry %g, exec used %g\n", next_len, next_feed, next->entry_velocity, used_exit);
        assert(0);
    }
}
int main(void) {
    run_case(2.0f, 800);                       /* a long next block */
    run_case(0.0001f, 1);                      /* short slow next blocks: the one-segment (B") path */
    run_case(0.00008f, 1);
    run_case(0.00012f, 1);
    return 0;
}
'''


class PlannerExecHandoff(unittest.TestCase):
    def test_exec_sees_whole_plans_only(self):
        cc = shutil.which('cc')
        if not cc:
            self.skipTest('host C compiler not installed')
        common = ['-std=gnu99', '-O1', '-g', '-funsigned-char', '-fsanitize=address,undefined',
                  '-D_POSIX_C_SOURCE=200809L', '-DHAS_STARTUP', '-I' + str(FIXTURE), '-I' + str(FIRMWARE)]
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            (d / 'check.c').write_text(CHECK)
            objs = []
            for name, extra in [('check.c', []), ('planner.c', ['-include', 'tgmath.h']),
                                ('plan_line.c', ['-include', 'tgmath.h']), ('util.c', ['-include', 'tgmath.h']),
                                ('plan_zoid.c', ['-include', 'math.h', '-Dsqrt=hook_sqrt'])]:
                src = d / name if name == 'check.c' else FIRMWARE / name
                obj = d / (name + '.o')
                subprocess.run([cc, *common, *extra, '-c', str(src), '-o', str(obj)], check=True)
                objs.append(str(obj))
            exe = d / 'handoff'
            subprocess.run([cc, '-fsanitize=address,undefined', *objs, '-lm', '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


    def test_exec_finishes_a_block_during_its_replan(self):
        cc = shutil.which('cc')
        if not cc:
            self.skipTest('host C compiler not installed')
        code = CHECK[:CHECK.index('static int interrupts')]
        code = code.replace('stat_t mp_exec_aline(mpBuf_t *bf) { return STAT_OK; }', '')
        code = code.replace('stat_t mp_exec_move(void) { return STAT_NOOP; }', '')
        code += REAL_EXEC
        line = (FIRMWARE / 'plan_line.c').read_text()
        marker = 'mp_calculate_trapezoid(&plan);'
        self.assertEqual(line.count(marker), 1)
        line = 'struct mpBuffer; void hook_after_plan(struct mpBuffer *bp);\n' + \
            line.replace(marker, marker + ' hook_after_plan(bp);')
        common = ['-std=gnu99', '-O1', '-g', '-funsigned-char', '-fsanitize=address,undefined',
                  '-D_POSIX_C_SOURCE=200809L', '-DHAS_STARTUP', '-I' + str(FIXTURE), '-I' + str(FIRMWARE)]
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            (d / 'check.c').write_text(code)
            (d / 'plan_line.c').write_text(line)
            objs = []
            for name, extra in [('check.c', []), ('planner.c', ['-include', 'tgmath.h']),
                                ('plan_line.c', ['-include', 'tgmath.h']), ('plan_exec.c', ['-include', 'tgmath.h']),
                                ('feedlimit.c', []),
                                ('util.c', ['-include', 'tgmath.h']),
                                ('plan_zoid.c', ['-include', 'math.h', '-Dsqrt=hook_sqrt'])]:
                src = d / name if name in ('check.c', 'plan_line.c') else FIRMWARE / name
                obj = d / (name + '.o')
                subprocess.run([cc, *common, *extra, '-c', str(src), '-o', str(obj)], check=True,
                               capture_output=True)
                objs.append(str(obj))
            exe = d / 'finish'
            subprocess.run([cc, '-fsanitize=address,undefined', *objs, '-lm', '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
