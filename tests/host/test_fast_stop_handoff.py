"""Inject limit notifications into the actual hold/limit decision blocks.

This isolates the request protocol from segment execution. Timer scheduling,
braking distances, and jerk-profile accuracy still require separate validation.
The inserted seams call the extracted limit block, not a copy of its logic.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(os.environ.get("TINYG_SOURCE", Path(__file__).resolve().parents[2]))


class FastStopHandoff(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        firmware = ROOT / "firmware/tinyg"
        exec_source = (firmware / "plan_exec.c").read_text()
        limit = function((firmware / "switch.c").read_text(),
                         r'if \(sw.s\[sw_num\].mode & SW_LIMIT_BIT\)')
        hold = function(exec_source, r'if \(cm.motion_state == MOTION_HOLD\)')
        seams = (
            ("bool fast = cm.hold_fast;", 1),
            ("if ((mr.section == SECTION_TAIL) && !fast) {", 2),
            ("float fast_ratio = fast ? _fast_stop_ratio(bf) : 1;", 3),
            ("cm.hold_fast_request = false;\n            cm.hold_fast = true;", 5),
        )
        for needle, seam in seams:
            if seam == 5:
                needle = "cm.hold_fast_request = false;\n            cm.hold_fast = true;"
                replacement = "cm.hold_fast_request = false; inject(5);\n            cm.hold_fast = true;"
            else:
                replacement = needle + f" inject({seam});"
            assert hold.count(needle) == 1, needle
            hold = hold.replace(needle, replacement)
        assert hold.count("SREG = sreg;") == 1
        hold = hold.replace("SREG = sreg;", "SREG = sreg; deliver_pending();")
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define __AVR 1
#define SW_LIMIT_BIT 1
#define fp_ZERO(x) (fabs(x)<0.000001)
enum { MOTION_STOP, MOTION_RUN, MOTION_HOLD };
enum { FEEDHOLD_OFF, FEEDHOLD_REQUESTED, FEEDHOLD_SYNC,
       FEEDHOLD_DECEL_CONTINUE, FEEDHOLD_DECEL_TO_ZERO,
       FEEDHOLD_PENDING, FEEDHOLD_HOLD };
enum { BLOCK_IDLE, BLOCK_INITIALIZING, BLOCK_RUNNING };
enum { SECTION_HEAD, SECTION_BODY, SECTION_TAIL, SECTION_NEW };
enum { STAT_OK, STAT_NOOP, SR_REQUEST_ASAP, CONTROLLER_READY };
typedef struct { int block_state; float length, entry_vmax; } mpBuf_t;
struct { int motion_state, hold_state; bool hold_fast;
         volatile bool hold_fast_request; } cm;
struct { int block_state, section, section_state;
         float entry_velocity, segment_velocity, forward_diff_5, cruise_velocity;
         float head_length, body_length, tail_length, exit_velocity;
         float target[1], position[1]; } mr;
struct { int controller_state; } cs;
struct { struct { int mode; } s[1]; } sw;
uint8_t SREG;
int at_seam, pending, injected, notifications, builds, ratios, idle;
void cm_set_motion_state(int state) { cm.motion_state=state; }
void controller_assert_limit_condition(int n) { (void)n; notifications++; }
bool mp_runtime_is_idle(void) { return idle; }
void mp_zero_segment_velocity(void) { mr.segment_velocity=0; }
void sr_request_status_report(int value) { (void)value; }
float get_axis_vector_length(float *target, float *position) {
    return fabs(target[0]-position[0]);
}
float mp_get_target_length(float from, float to, mpBuf_t *bf) {
    (void)from; (void)to; (void)bf; assert(SREG & 0x80); builds++; return 9;
}
float mp_get_target_velocity(float from, float length, mpBuf_t *bf) {
    (void)from; (void)length; (void)bf; return 1;
}
float _fast_stop_ratio(mpBuf_t *bf) {
    (void)bf; assert(SREG & 0x80); ratios++; return 9;
}
void limit_notification(void) { int sw_num=0;
'''
        code += limit + r'''
}
void inject(int seam) {
    if (seam!=at_seam || injected) return;
    injected=1;
    if (SREG & 0x80) limit_notification(); else pending=1;
}
void deliver_pending(void) {
    if (pending && (SREG & 0x80)) { pending=0; limit_notification(); }
}
void fake_cli(void) { inject(4); SREG &= ~0x80; }
#define cli() fake_cli()
int hold_step(mpBuf_t *bf) {
'''
        code += hold + r'''
    return STAT_OK;
}
void snapshot(void) {
    printf("%d %d %d %d %d %.6f %d %u %d\n", cm.hold_fast_request,
           cm.hold_fast,cm.hold_state,builds,ratios,mr.tail_length,
           pending,(unsigned)SREG,notifications);
}
int main(int argc, char **argv) {
    if(argc!=4) return 2;
    at_seam=atoi(argv[1]); int section=atoi(argv[2]); int state=atoi(argv[3]);
    SREG=0x80; sw.s[0].mode=SW_LIMIT_BIT; cm.motion_state=MOTION_HOLD;
    cm.hold_state=state; mr.section=section; mr.block_state=BLOCK_RUNNING;
    mr.segment_velocity=10; mr.tail_length=9; mr.target[0]=100;
    mpBuf_t bf={0};
    if(at_seam==0 || at_seam==5 || at_seam==6) limit_notification();
    if(state==FEEDHOLD_OFF || state==FEEDHOLD_REQUESTED) {
        cm.motion_state=MOTION_RUN; limit_notification();
    }
    if(state==FEEDHOLD_PENDING) { idle=(at_seam!=6); cm.hold_fast=true; }
    hold_step(&bf); snapshot();
    if(state==FEEDHOLD_PENDING) idle=1;
    hold_step(&bf); snapshot();
    limit_notification(); hold_step(&bf); snapshot();
    return 0;
}
'''
        path = Path(cls.tmp.name) / "handoff.c"
        path.write_text(code)
        cls.exe = path.with_suffix("")
        subprocess.run(["cc", "-std=c99", "-Wall", "-Werror",
                        "-fsanitize=address,undefined","-fstrict-flex-arrays=3", str(path), "-lm", "-o", str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_case(self, seam, section=1, state=2):
        out = subprocess.check_output([str(self.exe), str(seam), str(section), str(state)], text=True)
        return [tuple(map(float, line.split())) for line in out.splitlines()]

    def test_late_request_survives_and_rebuilds_once(self):
        for seam, section, normal_builds in ((1, 1, 1), (2, 2, 0), (3, 1, 1)):
            with self.subTest(seam=seam):
                first, second, third = self.run_case(seam, section)
                self.assertEqual(first, (1, 0, 4, normal_builds, 0, 9, 0, 128, 1))
                self.assertEqual(second, (0, 1, 4, normal_builds+1, 1, 3, 0, 128, 1))
                self.assertEqual(third, second[:-1] + (2,))

    def test_early_and_masked_requests_are_consumed_once(self):
        for seam in (0, 4, 5):
            with self.subTest(seam=seam):
                first, second, third = self.run_case(seam)
                notifications = 2 if seam == 5 else 1
                self.assertEqual(first, (0, 1, 4, 1, 1, 3, 0, 128, notifications))
                self.assertEqual(second, first)
                self.assertEqual(third, first[:-1] + (notifications+1,))

    def test_existing_deceleration_escalates(self):
        for state in (3, 4):
            with self.subTest(state=state):
                first, second, third = self.run_case(0, section=2, state=state)
                self.assertEqual(first, (0, 1, 4, 1, 1, 3, 0, 128, 1))
                self.assertEqual(second, first)
                self.assertEqual(third, first[:-1] + (2,))

    def test_limit_starts_off_or_requested_hold(self):
        for state in (0, 1):
            with self.subTest(state=state):
                first, _, _ = self.run_case(-1, state=state)
                self.assertEqual(first, (0, 1, 4, 1, 1, 3, 0, 128, 1))

    def test_pending_stop_clears_request_only_after_idle(self):
        first, second, _ = self.run_case(0, state=5)
        self.assertEqual(first[:5], (0, 0, 6, 0, 0))
        self.assertEqual(second, first)
        busy, stopped, _ = self.run_case(6, state=5)
        self.assertEqual(busy[:5], (1, 1, 5, 0, 0))
        self.assertEqual(stopped[:5], (0, 0, 6, 0, 0))


if __name__ == "__main__":
    unittest.main()
