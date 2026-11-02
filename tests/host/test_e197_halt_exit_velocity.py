"""E197: mp_halt_runtime() keeps the halted block's exit velocity, so the next move starts at speed.

mp_halt_runtime() (planner.c; cm_halt_motion() from cm_shutdown() and cm_panic()) stops the steppers and
resets the planner, but planner_reset() leaves mr.exit_velocity. {clear} leaves SHUTDOWN without a reset
(E045), and the next move's entry then comes from the runtime (plan_line.c _exit_of_previous():
mr.exit_velocity when nothing is queued before it): it is planned to start at the halted block's exit speed
from a standstill.

Case (found in review): the real planner and zoid, rates 800, jerk 20000, junction acceleration 1. The runtime
was running +X with a planned exit of 2.957 mm/min (a short block's exit) when mp_halt_runtime() stops it;
then +0.005 mm and +1 mm at F800 are queued. The first new block must start from 0.
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
uint8_t st_runtime_isbusy(void) { return true; }       /* the exec is busy: blocks stay queued */
void st_request_exec_move(void) {}
void st_prep_null(void) {}
void st_prep_command(void *bf) {}
void st_prep_dwell(float microseconds) {}
void stepper_reset(void) {}
stat_t mp_exec_aline(mpBuf_t *bf) { return STAT_OK; }
void cm_request_feedhold(void) {}
stat_t mp_exec_move(void) { return STAT_NOOP; }

static mpBuf_t *line_to(float x) {
    GCodeState_t gm = {0}; gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;
    gm.feed_rate_mode = UNITS_PER_MINUTE_MODE; gm.feed_rate = 800; gm.target[0] = x;
    mpBuf_t *w = mb.w;                                   /* the slot mp_aline() will fill */
    assert(mp_aline(&gm) == STAT_OK); assert(SREG == 0x95);
    return w;
}

int main(void) {
    memset(&cm, 0, sizeof(cm)); memset(&st_pre, 0, sizeof(st_pre)); memset(&mr, 0, sizeof(mr));
    planner_init();
    cm.machine_state = MACHINE_READY; cm.hold_state = FEEDHOLD_OFF; cm.cycle_state = CYCLE_MACHINING;
    for (int a = 0; a < AXES; a++) {
        cm.a[a].velocity_max = cm.a[a].feedrate_max = 800;
        cm.a[a].jerk_max = 20000; cm.a[a].recip_jerk = 1.0f / (20000 * JERK_MULTIPLIER);
        cm.a[a].junction_dev = .05;
    }
    cm.junction_acceleration = 1;
    rtc.sys_ticks = 100; SREG = 0x95;

    mr.exit_velocity = 2.95680189f; mr.unit[AXIS_X] = 1;  /* running +X, planned to leave at speed */
    mp_halt_runtime();                                   /* shutdown: steppers stopped, planner reset */
    cm.motion_state = MOTION_STOP; cm.cycle_state = CYCLE_OFF;
    st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;
    mpBuf_t *b = line_to(0.005f);
    line_to(1.005f);
    printf("%.9g %.9g %.9g\n", b->entry_velocity, b->cruise_velocity, b->exit_velocity);
    return 0;
}
'''


class HaltExitVelocity(unittest.TestCase):
    def test_move_after_halt_starts_from_rest(self):
        entry, cruise, exit_velocity = (float(v) for v in self.plan())
        self.assertEqual(entry, 0, f"planned to start at {entry} mm/min after the halt")

    def plan(self):
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
                                ('plan_zoid.c', ['-include', 'math.h'])]:
                src = d / name if name == 'check.c' else FIRMWARE / name
                obj = d / (name + '.o')
                r = subprocess.run([cc, *common, *extra, '-c', str(src), '-o', str(obj)], capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, r.stderr[-3000:])
                objs.append(str(obj))
            exe = d / 'plan'
            r = subprocess.run([cc, *common, *objs, '-lm', '-o', str(exe)], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr[-3000:])
            r = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr + r.stdout)
        return r.stdout.split()



if __name__ == '__main__':
    unittest.main()
