"""E181: planning a short move must not rewrite the run buffer's flags behind the exec.

mp_aline() (plan_line.c) calls mp_get_run_buffer() from the main loop for moves under the minimum
block time. mp_get_run_buffer() promotes a queued block to running: a read-modify-write of the flag
byte holding buffer_state:3, block_type:3 and block_state:2, which the exec interrupt also writes.
Compiled: lds r30/r31 mb.r; ldd r25,Z+12; test; andi/ori; std Z+12. If the exec starts the block (or
runs and frees a one-segment block) between the ldd and the std, the stale byte is written back:
block_state RUNNING -> INITIALIZING (the block is not freed at its end and runs again) or a freed
buffer becomes RUNNING again (the planner can no longer get that buffer and panics).

The real planner.c, plan_line.c, plan_zoid.c and util.c are linked. A main-loop call of
mp_get_run_buffer() runs the compiled sequence above with an exec "interrupt" at the window; a model of
the interrupt flag defers it when the main loop has interrupts off. Planning must also leave a queued
run block queued: starting it is the exec's job.
"""
from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = Path(os.environ.get('TINYG_FIRMWARE', ROOT / 'firmware/tinyg'))
FIXTURE = Path(__file__).parent / 'planner_startup'
DEF = r"[^;{}]*\)(?=\s*(?://[^\n]*\s*)?\{)"

PRELUDE = r'''
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
static int panics;
stat_t cm_panic_P(const stat_t status, const char *msg_P) { panics++; return (status); }
void en_set_encoder_steps(uint8_t motor, float steps) {}
void kn_inverse_kinematics(const float travel[], float steps[]) {}
stat_t rpt_exception(stat_t status, const char *msg) { return (status); }
stat_t rpt_exception_P(stat_t status, const char *msg) { return (status); }
void qr_request_queue_report(int8_t buffers) {}
uint8_t st_runtime_isbusy(void) { return true; }
void st_request_exec_move(void) {}
void st_prep_null(void) {}
void st_prep_command(void *bf) {}
void st_prep_dwell(float microseconds) {}
void stepper_reset(void) {}
stat_t mp_exec_aline(mpBuf_t *bf) { return STAT_OK; }
void cm_request_feedhold(void) {}
stat_t mp_exec_move(void) { return STAT_NOOP; }

/* the interrupt, fired at the instrumented point; deferred while the main loop has interrupts off */
static void (*isr)(void);
static int deferred;
void irq_window(void) {
    if (isr == NULL) return;
    if (SREG & 0x80) { void (*f)(void) = isr; isr = NULL; f(); }
    else { deferred = 1; }
}
static void after_main_call(void) {      /* the interrupt happens anyway: if not inside a window, then now */
    if (isr) { void (*f)(void) = isr; isr = NULL; deferred = 0; f(); }
}

/* E181: mp_get_run_buffer() as compiled, when the main loop calls it (planner.c is built with the
   real one renamed); the exec context calls the real one */
mpBuf_t *real_mp_get_run_buffer(void);
static int in_main;
mpBuf_t *mp_get_run_buffer(void) {
    if (!in_main) return real_mp_get_run_buffer();
    uint8_t saved = SREG;                               /* a fix may mask: honour it */
    mpBuf_t *p = mb.r;                                  /* lds r30/r31 mb.r */
    uint8_t bs = p->buffer_state, bt = p->block_type, bk = p->block_state;   /* ldd r25, Z+12 */
    irq_window();
    if ((bs == MP_BUFFER_QUEUED) || (bs == MP_BUFFER_PENDING)) {
        p->buffer_state = MP_BUFFER_RUNNING; p->block_type = bt; p->block_state = bk;   /* std Z+12 */
        bs = MP_BUFFER_RUNNING;
    }
    SREG = saved;
    return (bs == MP_BUFFER_RUNNING) ? p : NULL;
}

/* the exec interrupt, with the real planner functions (plan_exec.c mp_exec_move / mp_exec_aline) */
static void exec_starts_block(void) {
    in_main = 0;
    mpBuf_t *bf = mp_get_run_buffer();                  /* QUEUED/PENDING -> RUNNING */
    assert(bf != NULL);
    bf->replannable = false; bf->block_state = BLOCK_RUNNING;   /* plan_exec.c:208-209 */
    in_main = 1;
}
static void exec_runs_one_segment_block(void) {
    exec_starts_block();
    in_main = 0;
    mp_free_run_buffer();                               /* the block's only segment: done, freed */
    in_main = 1;
}
static void exec_frees_running_block(void) {
    in_main = 0; mp_free_run_buffer(); in_main = 1;
}

static void setup(void) {
    memset(&cm, 0, sizeof(cm)); memset(&st_pre, 0, sizeof(st_pre));
    planner_init();
    cm.machine_state = MACHINE_READY; cm.hold_state = FEEDHOLD_OFF; cm.cycle_state = CYCLE_MACHINING;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;
    for (int a = 0; a < AXES; a++) {
        cm.a[a].velocity_max = cm.a[a].feedrate_max = 20000;
        cm.a[a].jerk_max = 5000; cm.a[a].recip_jerk = 1.0f / (5000 * JERK_MULTIPLIER);
        cm.a[a].junction_dev = .05;
    }
    cm.junction_acceleration = 100000;
    rtc.sys_ticks = 100; SREG = 0x95; isr = NULL; deferred = 0; in_main = 1; panics = 0;
}
static stat_t line_to(float x, float feed) {
    GCodeState_t gm = {0}; gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;
    gm.feed_rate_mode = UNITS_PER_MINUTE_MODE; gm.feed_rate = feed; gm.target[0] = x;
    stat_t s = mp_aline(&gm);
    after_main_call();
    return s;
}
static int empty_buffers(void) {
    int n = 0;
    for (int i = 0; i < PLANNER_BUFFER_POOL_SIZE; i++) n += (mb.bf[i].buffer_state == MP_BUFFER_EMPTY);
    return n;
}
'''

E181_MAIN = r'''
int main(void) {
    /* the exec starts the queued block while a short move is being planned */
    setup();
    assert(line_to(10.0f, 600) == STAT_OK);             /* the run block, queued, not started */
    mpBuf_t *a = mb.r;
    isr = exec_starts_block;
    line_to(10.02f, 6000);                              /* 0.02 mm at F6000: planned through line 136 */
    assert(a->buffer_state == MP_BUFFER_RUNNING);
    assert(a->block_state == BLOCK_RUNNING);            /* stock: INITIALIZING again -> the block reruns */

    /* planning alone leaves the queued run block to the exec */
    setup();
    assert(line_to(10.0f, 600) == STAT_OK);
    a = mb.r;
    uint8_t before = a->buffer_state;
    line_to(10.02f, 6000);
    assert(a->buffer_state == before);                  /* stock: promoted to RUNNING by the main loop */

    /* a one-segment block starts and finishes in that window */
    setup();
    assert(line_to(0.01f, 6000) != STAT_OK || 1);
    setup();
    assert(line_to(10.0f, 600) == STAT_OK);
    a = mb.r;
    isr = exec_runs_one_segment_block;
    line_to(10.02f, 6000);
    assert(a->buffer_state == MP_BUFFER_EMPTY);         /* stock: the freed buffer is RUNNING again */
    for (int n = mb.buffers_available; n > 0; n--) {
        assert(mp_get_write_buffer() != NULL);          /* every buffer the planner counts can be had */
    }
    assert(panics == 0);
    return 0;
}
'''

def _cc():
    return shutil.which('cc')


class RunBufferPeek(unittest.TestCase):
    def _planner_exe(self, d, main, planner_source):
        common = ['-std=gnu99', '-O1', '-g', '-funsigned-char', '-fsanitize=address,undefined','-fstrict-flex-arrays=3',
                  '-D_POSIX_C_SOURCE=200809L', '-DHAS_STARTUP', '-I' + str(FIXTURE), '-I' + str(FIRMWARE)]
        (d / 'check.c').write_text(PRELUDE + main)
        (d / 'planner_check.c').write_text(planner_source)
        objs = []
        for name, src, extra in [('check.c', d / 'check.c', []),
                                 ('planner.c', d / 'planner_check.c',
                                  ['-include', 'tgmath.h', '-Dmp_get_run_buffer=real_mp_get_run_buffer']),
                                 ('plan_line.c', FIRMWARE / 'plan_line.c', ['-include', 'tgmath.h']),
                                 ('plan_zoid.c', FIRMWARE / 'plan_zoid.c', ['-include', 'tgmath.h']),
                                 ('util.c', FIRMWARE / 'util.c', ['-include', 'tgmath.h'])]:
            obj = d / (name + '.o')
            subprocess.run([_cc(), *common, *extra, '-c', str(src), '-o', str(obj)], check=True)
            objs.append(str(obj))
        exe = d / 'check'
        subprocess.run([_cc(), '-fsanitize=address,undefined','-fstrict-flex-arrays=3', *objs, '-lm', '-o', str(exe)], check=True)
        return subprocess.run([str(exe)], capture_output=True, text=True)

    def _planner_source(self):
        return (FIRMWARE / 'planner.c').read_text()

    def test_e181_run_buffer_flags_written_from_the_main_loop(self):
        if not _cc():
            self.skipTest('host C compiler not installed')
        with tempfile.TemporaryDirectory() as d:
            result = self._planner_exe(Path(d), E181_MAIN, self._planner_source())
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
