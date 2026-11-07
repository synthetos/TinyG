"""E196: a previous block the exec frees while mp_aline() reads its direction changes the new block's plan.

mp_aline() (plan_line.c) computes the junction limit from bf->pv->unit. The exec frees a block as soon as its
last segment is prepared (plan_exec.c, mp_free_run_buffer), while that segment and the one before it may still
be stepping, and the free clears the whole buffer, direction included. A cleared direction reads as a
right-angle corner: the new block's entry_vmax drops, and with it exit_vmax, which later replans keep using.
A zero exit requested for the last block is not a zero exit either: a short block's profile can leave it at
speed (here about 2.96 mm/min).

Fix: after the junction calculation, if the previous buffer is empty (freed before or during the read, or no
block queued before), the junction is computed again from the runtime's copy of the direction (mr.unit). With
the previous buffer empty nothing else is queued, so mr.unit is the last block's direction and stays put.

Case (found in review): the real planner and zoid, rates 800, jerk 20000, junction acceleration 1. Straight
+X blocks 0.000203373 mm, +0.005 mm and +1 mm at F800. While the second block is added, a hook does what the
exec does when the first block's last segment is prepared: the runtime holds its direction and exit, and
mp_free_run_buffer() frees it. Freed before the direction is read and freed just after it must give the
second block the same entry/exit limits (the straight-on 800) and the same planned entry, cruise and exit.
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
#include <stdio.h>
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
#include "util.h"
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
void cm_abort_arc(void) {} void cm_run_short_move(void) {}
stat_t cm_is_alarmed(void) { return cm.machine_state == MACHINE_ALARM; }
stat_t cm_panic_P(const stat_t status, const char *msg_P) { return (status); }
void en_set_encoder_steps(uint8_t motor, float steps) {}
void kn_inverse_kinematics(const float travel[], float steps[]) {}
stat_t rpt_exception(stat_t status, const char *msg) { return (status); }
stat_t rpt_exception_P(stat_t status, const char *msg) { return (status); }
void qr_request_queue_report(int8_t buffers) {}
uint8_t st_runtime_isbusy(void) { return true; }       /* the exec is busy: blocks stay queued */
void st_request_exec_move(void) {}
void st_prep_null(void) {}
void st_prep_command(void *bf) {}
void st_prep_dwell(float microseconds) {}
void stepper_reset(void) {}
stat_t mp_exec_aline(mpBuf_t *bf) { return STAT_OK; }
void cm_request_feedhold(void) {}
stat_t mp_exec_move(void) { return STAT_NOOP; }

static mpBuf_t *first;
static int free_once;

void exec_frees_previous(void) {                         /* the first block's last segment was just prepared */
    if (!free_once) return;
    free_once = 0;
    assert(mb.r == first);
    copy_vector(mr.unit, first->unit);                   /* the runtime set these at the block's start */
    mr.exit_velocity = first->exit_velocity;
    mp_free_run_buffer();
}

static mpBuf_t *line_to(float x) {
    GCodeState_t gm = {0}; gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;
    gm.feed_rate_mode = UNITS_PER_MINUTE_MODE; gm.feed_rate = 800; gm.target[0] = x;
    mpBuf_t *w = mb.w;                                   /* the slot mp_aline() will fill */
    assert(mp_aline(&gm) == STAT_OK); assert(SREG == 0x95);
    return w;
}

int main(int argc, char **argv) {
    memset(&cm, 0, sizeof(cm)); memset(&st_pre, 0, sizeof(st_pre)); memset(&mr, 0, sizeof(mr));
    planner_init();
    cm.machine_state = MACHINE_READY; cm.hold_state = FEEDHOLD_OFF; cm.cycle_state = CYCLE_MACHINING;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;
    for (int a = 0; a < AXES; a++) {
        cm.a[a].velocity_max = cm.a[a].feedrate_max = 800;
        cm.a[a].jerk_max = 20000; cm.a[a].recip_jerk = 1.0f / (20000 * JERK_MULTIPLIER);
        cm.a[a].junction_dev = .05;
    }
    cm.junction_acceleration = 1;
    rtc.sys_ticks = 100; SREG = 0x95;

    const float len = 0.000203373f;
    first = line_to(len);
    free_once = (argc > 1);
    mpBuf_t *b = line_to(len + 0.005f);
    assert(free_once == 0);                              /* the hook ran (or was not asked for) */
    line_to(len + 0.005f + 1.0f);
    printf("%.9g %.9g %.9g %.9g %.9g\n", b->entry_vmax, b->exit_vmax,
           b->entry_velocity, b->cruise_velocity, b->exit_velocity);
    return 0;
}
'''

# where the free happens: before the direction is read, and just after the junction calculation
BEFORE = "junction_velocity = _get_junction_vmax(bf->pv->unit"
AFTER = "bool corner = ("                                  # the limits are written out with the feed-limit tags


class FreedPreviousDirection(unittest.TestCase):
    def test_freed_previous_block_leaves_the_plan_alone(self):
        kept = self.plan(AFTER)                          # the direction was read before the free
        self.assertEqual(kept[:2], ['800', '800'], kept)  # straight on: no corner limit
        self.assertEqual(self.plan(BEFORE), kept, "freed before the direction was read")

    def plan(self, hook):
        cc = shutil.which('cc')
        if not cc:
            self.skipTest('host C compiler not installed')
        common = ['-std=gnu99', '-O1', '-g', '-funsigned-char', '-fsanitize=address,undefined','-fstrict-flex-arrays=3',
                  '-D_POSIX_C_SOURCE=200809L', '-DHAS_STARTUP', '-I' + str(FIXTURE), '-I' + str(FIRMWARE)]
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            (d / 'check.c').write_text(CHECK)
            line = (FIRMWARE / 'plan_line.c').read_text(encoding='latin-1')
            self.assertEqual(line.count(hook), 1, hook)
            line = "void exec_frees_previous(void);\n" + line.replace(hook, "exec_frees_previous(); " + hook)
            (d / 'plan_line.c').write_text(line, encoding='latin-1')
            objs = []
            for name, extra in [('check.c', []), ('planner.c', ['-include', 'tgmath.h']),
                                ('plan_line.c', ['-include', 'tgmath.h']), ('util.c', ['-include', 'tgmath.h']),
                                ('plan_zoid.c', ['-include', 'math.h'])]:
                src = d / name if name in ('check.c', 'plan_line.c') else FIRMWARE / name
                obj = d / (name + '.o')
                r = subprocess.run([cc, *common, *extra, '-c', str(src), '-o', str(obj)], capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, r.stderr[-3000:])
                objs.append(str(obj))
            exe = d / 'plan'
            r = subprocess.run([cc, *common, *objs, '-lm', '-o', str(exe)], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr[-3000:])
            r = subprocess.run([str(exe), 'free'], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr + r.stdout)
        return r.stdout.split()


if __name__ == '__main__':
    unittest.main()
