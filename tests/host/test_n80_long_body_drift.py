"""N80: a long constant-speed section must not carry float drift into its last segment.

_exec_aline_segment() (plan_exec.c) set each segment's target as the previous target plus unit * segment_length, in
32-bit floats, and snapped the section's last segment onto its waypoint. Over the ~21,900 segments of a 1,539 mm
rapid at 843 mm/min the running sum fell 0.25 mm short in Y, so the snap asked for ~320 steps in a 5 ms segment of
250 DDA ticks and st_prep_line() panicked "step rate exceeds DDA" (bench 3, 450.03: G0 X0 Y1460 Z485 A37.623 from
the origin, Y and Z at 1280 steps/mm). This runs the shipped _exec_aline_body() and _exec_aline_segment() over that
body with a stub st_prep_line() that records every segment's steps against its DDA ticks.
"""
from pathlib import Path
import math
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]

HARNESS = r'''
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#define AXES 6
#define MOTORS 4
#define SECTIONS 3
typedef uint8_t stat_t;
#define STAT_OK 0
#define STAT_EAGAIN 1
#define MICROSECONDS_PER_MINUTE ((float)60000000)
#define uSec(a) ((float)(a * MICROSECONDS_PER_MINUTE))
#define NOM_SEGMENT_USEC ((float)5000)
#define MIN_SEGMENT_USEC ((float)2500)
#define MIN_SEGMENT_TIME (MIN_SEGMENT_USEC / MICROSECONDS_PER_MINUTE)
#define FREQUENCY_DDA (float)50000
#define EPSILON ((float)0.00001)
#define fp_ZERO(a) (fabs(a) < EPSILON)
#define copy_vector(d,s) memcpy((d),(s),sizeof(float)*AXES)
#define ritorno(a) if ((status_code = (a)) != STAT_OK) { return (status_code); }
enum { SECTION_HEAD = 0, SECTION_BODY, SECTION_TAIL };
enum { SECTION_NEW = 0, SECTION_1st_HALF, SECTION_2nd_HALF };
enum { MOTION_STOP = 0, MOTION_PLANNING, MOTION_RUN, MOTION_HOLD };
enum { CYCLE_OFF = 0, CYCLE_MACHINING };
static stat_t status_code;
/* This regression exercises the unchanged cruise path with no override request. */
static uint32_t ov_segment_ordinal;
static bool _override_take_body(void) { return false; }
static stat_t _exec_aline_head(void) { return STAT_OK; }
static struct {
    uint8_t section, section_state;
    float unit[AXES], target[AXES], position[AXES], waypoint[SECTIONS][AXES];
    float origin[AXES], origin_distance, body_base;            /* used by the fixed runtime; unused before */
    float target_steps[MOTORS], position_steps[MOTORS], commanded_steps[MOTORS], encoder_steps[MOTORS],
          following_error[MOTORS];
    float head_length, body_length, tail_length, entry_velocity, cruise_velocity, exit_velocity;
    float segments; uint32_t segment_count; float segment_velocity, segment_time;
    float forward_diff_1, forward_diff_2, forward_diff_3, forward_diff_4, forward_diff_5;
    struct { float move_time; float target[AXES]; } gm;
} mr;
static struct { uint8_t motion_state, cycle_state; } cm = { MOTION_RUN, CYCLE_MACHINING };
static struct { uint32_t fl_rec; } st_pre;
static bool fl_on = false;
static void fl_segment(float v, float l) { (void)v; (void)l; }
static float spu[MOTORS] = { 1280, 1280, 1280, 4.444444f };   /* motor 1 Z, 2 and 3 Y, 4 X (bench 3) */
static const int axis_of[MOTORS] = { 2, 1, 1, 0 };
static void kn_inverse_kinematics(const float travel[], float steps[]) {
    for (int m = 0; m < MOTORS; m++) steps[m] = travel[axis_of[m]] * spu[m];
}
static float en_read_encoder(uint8_t m) { (void)m; return 0; }
static double worst = 0; static long segments_run = 0, over = 0;
static stat_t st_prep_line(float travel_steps[], float following_error[], float segment_time) {
    (void)following_error;
    double ticks = (double)(int32_t)(segment_time * 60 * FREQUENCY_DDA);
    for (int m = 0; m < MOTORS; m++) {
        double r = fabs(travel_steps[m]) / ticks;
        if (r > worst) worst = r;
        if (fabs(travel_steps[m]) > ticks) over++;
    }
    segments_run++;
    return STAT_OK;
}
static stat_t _exec_aline_tail(void) { return STAT_OK; }
'''

MAIN = r'''
int main(void) {
    const float end[4] = { 0, 1460, 485, 37.623f };   /* X Y Z A from the origin */
    float length = sqrtf(end[0]*end[0] + end[1]*end[1] + end[2]*end[2] + end[3]*end[3]);
    memset(&mr, 0, sizeof(mr));
    for (int a = 0; a < 4; a++) mr.unit[a] = end[a] / length;
    mr.head_length = 0; mr.body_length = length; mr.tail_length = 0;
    mr.cruise_velocity = 843.25f;
    for (int a = 0; a < AXES; a++) {
        mr.waypoint[SECTION_HEAD][a] = mr.position[a];
        mr.waypoint[SECTION_BODY][a] = mr.waypoint[SECTION_TAIL][a] = (a < 4) ? end[a] : 0;
        mr.origin[a] = mr.position[a];
    }
    mr.section = SECTION_BODY; mr.section_state = SECTION_NEW;
    kn_inverse_kinematics(mr.position, mr.target_steps);
    for (long guard = 0; guard < 100000; guard++) {
        if (_exec_aline_body() == STAT_OK) break;
    }
    printf("%ld %ld %.4f %.6f %.6f\n", segments_run, over, worst, mr.position[1], mr.position[2]);
    return 0;
}
'''


class LongBodyDrift(unittest.TestCase):
    def test_last_body_segment_stays_inside_the_dda(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler required")
        src = (ROOT / "firmware/tinyg/plan_exec.c").read_text()
        code = HARNESS
        code += function(src, r"static stat_t _exec_aline_segment\(\)") + "\n"
        code += function(src, r"static stat_t _exec_aline_body\(\)") + "\n"
        code += MAIN
        with tempfile.TemporaryDirectory() as d:
            c, exe = Path(d) / "t.c", Path(d) / "t"
            c.write_text(code)
            subprocess.run([cc, "-std=gnu99", "-O1", "-fsingle-precision-constant", "-o", str(exe),
                            str(c), "-lm"], check=True, capture_output=True, text=True)
            out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout.split()
        segs, over, worst, y, z = int(out[0]), int(out[1]), float(out[2]), float(out[3]), float(out[4])
        self.assertGreater(segs, 20000, "the body ran as one long run of segments")
        self.assertAlmostEqual(y, 1460, places=2)
        self.assertAlmostEqual(z, 485, places=2)
        self.assertEqual(over, 0, f"{over} segment(s) asked for more steps than DDA ticks (worst {worst:.2f} steps per "
                                  f"tick); the last body segment carried the float drift of the running sum")
        self.assertLess(worst, 0.5, f"worst segment {worst:.2f} steps per tick: drift still reaches the snap")


if __name__ == "__main__":
    unittest.main()
