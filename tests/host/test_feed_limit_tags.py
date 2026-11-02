"""Feed-limit tags (feedlimit.h) set by the real planner: planner.c, plan_line.c, plan_zoid.c, util.c.

Blocks are queued through mp_aline() with the exec held busy, as in test_planner_exec_handoff, and each
block's fl_tags checked after planning:
  - FL_AXIS_CAP when an axis's feedrate max makes a feed move slower than its F (a one-axis move above fr,
    a diagonal whose X share exceeds fr_x), never for a diagonal within fr, a tie, or a traverse, nor when
    an unused axis has fr = 0 (its 0/0 time is NaN; the move time and the tag ignore it as the planner did);
  - FL_JUNCTION for an entry limited by a corner, not for a straight continuation;
  - FL_EXACT_STOP under G61;
  - FL_SHORT for a move too short to reach its cruise, FL_MIN_SEGMENT for the single-segment profile
    (a tiny move between two long ones at speed), neither for a long move: fl_profile_tags() (feedlimit.c)
    on the plan, which the exec calls once per block; the planner's replan loop never sets them;
  - the planner's replan loop never changes the tags;
  - a previous block the exec frees before its direction is read, or after the planner has checked it, is
    never tagged as a corner (built with a hook at each place); the junction itself is E196's.
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
#include "feedlimit.h"
#include "xmega/xmega_rtc.h"

cmSingleton_t cm;
stPrepSingleton_t st_pre;
rtClock_t rtc;
controller_t cs;
uint8_t SREG;
char text_item[256];
static uint8_t path_control = PATH_CONTINUOUS;

uint8_t cm_get_path_control(const GCodeState_t *gcode_state) { return (path_control); }
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
uint8_t st_runtime_isbusy(void) { return true; }       /* the exec is busy: blocks stay queued */
void st_request_exec_move(void) {}
void st_prep_null(void) {}
void st_prep_command(void *bf) {}
void st_prep_dwell(float microseconds) {}
void stepper_reset(void) {}
stat_t mp_exec_aline(mpBuf_t *bf) { return STAT_OK; }
void cm_request_feedhold(void) {}
stat_t mp_exec_move(void) { return STAT_NOOP; }

static mpBuf_t *queued[64];
static int nq;

static void setup(void) {
    memset(&cm, 0, sizeof(cm)); memset(&st_pre, 0, sizeof(st_pre));
    planner_init(); nq = 0; path_control = PATH_CONTINUOUS;
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
static mpBuf_t *move(uint8_t mode, float f, float x, float y) {
    GCodeState_t gm = {0}; gm.motion_mode = mode;
    gm.feed_rate_mode = UNITS_PER_MINUTE_MODE; gm.feed_rate = f; gm.target[0] = x; gm.target[1] = y;
    mpBuf_t *w = mb.w;                                   /* the slot mp_aline() will fill */
    assert(mp_aline(&gm) == STAT_OK); assert(SREG == 0x95);
    queued[nq++] = w;
    return w;
}
static mpBuf_t *line_to(float f, float x, float y) { return move(MOTION_MODE_STRAIGHT_FEED, f, x, y); }
#define HAS(bf, t) (((bf)->fl_tags & (t)) != 0)
#define PROF(bf, t) ((fl_profile_tags((bf)->naiive_move_time, (bf)->cruise_velocity, (bf)->cruise_vmax) & (t)) != 0)

int main(void) {
    /* long straight line within fr: only the end of the list */
    setup();
    mpBuf_t *a = line_to(800, 100, 0);
    assert(a->fl_tags == 0);                                           /* a start from rest is not a corner */
    mpBuf_t *b = line_to(800, 200, 0);                                 /* a collinear continuation: no corner */
    assert(!HAS(b, FL_JUNCTION));
    mpBuf_t *c = line_to(800, 200, 100);                               /* a right-angle corner limits the entry */
    assert(HAS(c, FL_JUNCTION) && c->entry_vmax < c->cruise_vmax);

    /* axis caps */
    setup();
    assert(HAS(line_to(1200, 100, 0), FL_AXIS_CAP));                    /* X alone above fr */
    setup();
    assert(!HAS(line_to(800, 100, 0), FL_AXIS_CAP));                    /* exactly fr: a tie */
    setup();
    assert(!HAS(line_to(1000, 100, 100), FL_AXIS_CAP));                 /* diagonal: 707 per axis */
    setup(); cm.a[0].feedrate_max = 500;
    assert(HAS(line_to(1000, 100, 100), FL_AXIS_CAP));                  /* X share above fr_x */
    setup();
    assert(!HAS(move(MOTION_MODE_STRAIGHT_TRAVERSE, 0, 100, 0), FL_AXIS_CAP));  /* G0: no F to fall short of */
    setup(); cm.a[AXIS_C].feedrate_max = 0;                             /* unused C with fr 0: its time is NaN */
    mpBuf_t *z = line_to(600, 100, 0);
    assert(isfinite(z->gm.move_time) && fabs(z->gm.move_time - 100.0f / 600) < 1e-6 && !HAS(z, FL_AXIS_CAP));

    /* exact stop */
    setup(); path_control = PATH_EXACT_STOP;
    assert(HAS(line_to(800, 100, 0), FL_EXACT_STOP));

    /* short moves: a chain of 0.3 mm moves never reaches 800 mm/min with this jerk; 0.002 mm is one segment */
    setup();
    float x = 0;
    for (int i = 0; i < 6; i++) { x += 0.3f; line_to(800, x, 0); }
    assert(PROF(queued[2], FL_SHORT) && !PROF(queued[2], FL_MIN_SEGMENT));
    for (int i = 0; i < nq; i++) assert(!HAS(queued[i], FL_PROFILE_TAGS));   /* not from the replan loop */
    setup();                                             /* a tiny move between two long ones, at speed */
    line_to(800, 100, 0); line_to(800, 100.05f, 0); line_to(800, 200, 0);   /* 3.75 ms at F */
    if (!(PROF(queued[1], FL_MIN_SEGMENT) && !PROF(queued[1], FL_SHORT))) {
        fprintf(stderr, "tiny: tags %#x naiive %g entry %g exit %g\n", queued[1]->fl_tags, queued[1]->naiive_move_time,
                queued[1]->entry_velocity, queued[1]->exit_velocity); return 3;
    }
    setup();
    line_to(800, 100, 0);
    assert(!PROF(queued[0], FL_SHORT) && !PROF(queued[0], FL_MIN_SEGMENT));

    /* replanning leaves the tags as created */
    setup();
    mpBuf_t *s = line_to(1200, 100, 0), *t = line_to(1200, 100, 100);
    uint8_t ts = s->fl_tags, tt = t->fl_tags;
    for (int i = 0; i < 4; i++) line_to(1200, 100 + 0.3f * (i + 1), 100);
    assert(s->fl_tags == ts && t->fl_tags == tt && HAS(t, FL_JUNCTION | FL_AXIS_CAP));
    puts("feed-limit tags: planner sets and publishes them as planned");
    return 0;
}
'''


FREED = r'''
void exec_frees_previous(void) {                         /* the exec finishes and frees the previous block now */
    if (!free_once) return;
    free_once = 0;
    mpBuf_t *prev = mb.w->pv;                            /* mp_aline is filling mb.w's predecessor's successor */
    for (int i = 0; i < PLANNER_BUFFER_POOL_SIZE; i++) if (&mb.bf[i] == queued[nq - 1]) prev = &mb.bf[i];
    mpBuf_t *nx = prev->nx, *pv = prev->pv;
    memset(prev, 0, sizeof(mpBuf_t)); prev->nx = nx; prev->pv = pv; prev->buffer_state = MP_BUFFER_EMPTY;
}
int main(void) {
    setup();
    line_to(800, 100, 0);
    free_once = 1;                                       /* freed between reading the previous block and its use */
    mpBuf_t *b = line_to(800, 200, 0);
    assert(!HAS(b, FL_JUNCTION));                        /* straight on: never a corner */
    puts("feed-limit tags: a previous block freed mid-read is not a corner");
    return 0;
}
'''


class FeedLimitTags(unittest.TestCase):
    def test_planner_tags(self):
        self.build_run(CHECK, None)

    def test_previous_block_freed_while_read(self):
        main = CHECK[:CHECK.index("int main(void) {")] + "static int free_once;\n" + FREED
        for hook in ("junction_velocity = _get_junction_vmax(bf->pv->unit", "bool corner = ("):
            with self.subTest(freed_before=hook):
                self.build_run(main, hook)

    def build_run(self, check, hook_before):
        cc = shutil.which('cc')
        if not cc:
            self.skipTest('host C compiler not installed')
        common = ['-std=gnu99', '-O1', '-g', '-funsigned-char', '-fsanitize=address,undefined',
                  '-D_POSIX_C_SOURCE=200809L', '-DHAS_STARTUP', '-I' + str(FIXTURE), '-I' + str(FIRMWARE)]
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            (d / 'check.c').write_text(check)
            line = (FIRMWARE / 'plan_line.c').read_text(encoding='latin-1')
            if hook_before:
                assert line.count(hook_before) == 1
                line = line.replace(hook_before, "exec_frees_previous(); " + hook_before)
                line = "void exec_frees_previous(void);\n" + line
            (d / 'plan_line.c').write_text(line, encoding='latin-1')
            objs = []
            for name, extra in [('check.c', []), ('planner.c', ['-include', 'tgmath.h']),
                                ('plan_line.c', ['-include', 'tgmath.h']), ('util.c', ['-include', 'tgmath.h']),
                                ('plan_zoid.c', ['-include', 'math.h']), ('feedlimit.c', [])]:
                src = d / name if name in ('check.c', 'plan_line.c') else FIRMWARE / name
                obj = d / (name + '.o')
                r = subprocess.run([cc, *common, *extra, '-c', str(src), '-o', str(obj)], capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, r.stderr[-3000:])
                objs.append(str(obj))
            exe = d / 'tags'
            r = subprocess.run([cc, *common, *objs, '-lm', '-o', str(exe)], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr[-3000:])
            r = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr + r.stdout)


if __name__ == '__main__':
    unittest.main()
