"""E149: a zero-width spindle speed range ($p1csl == $p1csh) must not produce a NaN PWM duty.

The real cm_get_spindle_pwm() (spindle.c) and pwm_set_duty() (pwm.c, AVR path, on a stub timer) are
extracted. With csl == csh the normalisation was 0/0; the NaN passed pwm_set_duty()'s < 0 and > 1
checks and the AVR float-to-integer conversion wrote CCB = 1 (measured on bench 2: ~0% duty, below
the 10% off phase). An exactly zero-width range now maps any S to phase_hi; NaN bounds or phases stay NaN and
the non-finite duty is refused, leaving the PWM output unchanged.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]

PRELUDE = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#define __AVR
#define PWM_1 0
enum { SPINDLE_OFF, SPINDLE_CW, SPINDLE_CCW };
typedef int stat_t;
#define STAT_OK 0
#define STAT_INPUT_LESS_THAN_MIN_VALUE 1
#define STAT_INPUT_EXCEEDS_MAX_VALUE 2
#define STAT_INPUT_VALUE_RANGE_ERROR 3
typedef struct { uint16_t PER, CCB; uint8_t CTRLB; } timer_t_;
static timer_t_ timer = { 40000, 0 };
struct { struct { float cw_speed_lo, cw_speed_hi, cw_phase_lo, cw_phase_hi,
                         ccw_speed_lo, ccw_speed_hi, ccw_phase_lo, ccw_phase_hi, phase_off; } c[1];
         struct { timer_t_ *timer; } p[1]; } pwm;
struct { struct { float spindle_speed; } gm; } cm;
#define PWM_TIMER_t timer_t_

#define PWMS 1
#define STAT_NO_SUCH_DEVICE 4
#define TC0_CCBEN_bm 0x20
#define PWM1_CTRLB 0x23
#define PWM2_CTRLB 3
#define SPINDLE_PWM 2
static unsigned char SREG=0x80;
static void cli(void){SREG=0;}
static int pin;
static void gpio_set_bit_off(int bit){assert(bit==SPINDLE_PWM);pin=0;}
static void gpio_set_bit_on(int bit){assert(bit==SPINDLE_PWM);pin=1;}

'''

EXERCISE = r'''
static float duty(float s) { cm.gm.spindle_speed = s; return cm_get_spindle_pwm(SPINDLE_CW); }
int main(void) {
    pwm.p[0].timer = &timer;
    pwm.c[0].cw_phase_lo = 0.125f; pwm.c[0].cw_phase_hi = 0.2f; pwm.c[0].phase_off = 0.1f;
    /* normal range: linear */
    pwm.c[0].cw_speed_lo = 1000; pwm.c[0].cw_speed_hi = 2000;
    assert(fabsf(duty(1500) - 0.1625f) < 1e-6f);
    /* zero-width range: phase_hi for any S, never NaN */
    pwm.c[0].cw_speed_hi = 1000;
    assert(isfinite(duty(1000)) && fabsf(duty(1000) - 0.2f) < 1e-6f);
    assert(fabsf(duty(5000) - 0.2f) < 1e-6f && fabsf(duty(0) - 0.2f) < 1e-6f);
    /* invalid (NaN) bounds or phases are not turned into a valid duty: the PWM keeps its value */
    timer.CCB = 1234;
    pwm.c[0].cw_speed_lo = NAN; pwm.c[0].cw_speed_hi = 2000;
    assert(pwm_set_duty(PWM_1, duty(1500)) != STAT_OK && timer.CCB == 1234);
    pwm.c[0].cw_speed_lo = 1000; pwm.c[0].cw_speed_hi = NAN;
    assert(pwm_set_duty(PWM_1, duty(1500)) != STAT_OK && timer.CCB == 1234);
    pwm.c[0].cw_speed_lo = NAN; pwm.c[0].cw_speed_hi = NAN;
    assert(pwm_set_duty(PWM_1, duty(1500)) != STAT_OK && timer.CCB == 1234);
    pwm.c[0].cw_speed_lo = 1000; pwm.c[0].cw_speed_hi = 2000; pwm.c[0].cw_phase_hi = NAN;
    assert(pwm_set_duty(PWM_1, duty(1500)) != STAT_OK && timer.CCB == 1234);
    pwm.c[0].cw_phase_hi = 0.2f;
    /* a non-finite duty is refused and leaves the compare register alone */
    timer.CCB = 1234;
    assert(pwm_set_duty(PWM_1, NAN) != STAT_OK && timer.CCB == 1234);
    assert(pwm_set_duty(PWM_1, INFINITY) != STAT_OK && timer.CCB == 1234);
    assert(pwm_set_duty(PWM_1, 0.5f) == STAT_OK && timer.CCB == 20001);
    return 0;
}
'''


class PwmSpeedRange(unittest.TestCase):
    def test_zero_width_range_is_finite(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        fw = ROOT / "firmware/tinyg"
        code = PRELUDE
        code += function((fw / "spindle.c").read_text(), r"float cm_get_spindle_pwm\([^;{}]*\)(?=\s*\{)") + "\n"
        code += function((fw / "pwm.c").read_text(), r"stat_t pwm_set_duty\([^;{}]*\)(?=\s*\{)") + "\n"
        code += EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-e149-") as tmp:
            c, exe = Path(tmp) / "pwm.c", Path(tmp) / "pwm"
            c.write_text(code)
            subprocess.run([cc, "-std=gnu99", "-Wall", "-Wno-unused-function", "-fsanitize=address,undefined","-fstrict-flex-arrays=3",
                            str(c), "-o", str(exe), "-lm"], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
