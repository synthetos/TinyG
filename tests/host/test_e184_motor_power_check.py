"""E184: the motor power timeout must not disable a motor whose move has just started.

stepper.c st_motor_power_callback() (main loop) sees a motor's idle countdown expired, then sets
MOTOR_IDLE and de-energizes it. A move that starts in between has its first segment loaded by
_load_move() (HI: enable bit cleared, power_state TIMEOUT_START); the callback then disables the motor
while that segment steps, and its steps are lost. The real functions are extracted; the loader runs
right after the callback reads the clock.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = Path(os.environ.get('TINYG_FIRMWARE', ROOT / 'firmware/tinyg'))
DEF = r"[^;{}]*\)(?=\s*(?://[^\n]*\s*)?\{)"

POWER_PRELUDE = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#define __AVR
uint8_t SREG = 0x80;
#define cli() (SREG &= (uint8_t)~128)
#define MOTORS 4
#define MOTOR_1 0
#define MOTOR_2 1
#define MOTOR_3 2
#define MOTOR_4 3
#define MOTOR_ENABLE_BIT_bm 0x01
#define STAT_OK 0
#define SR_REQUEST_TIMED 1
typedef int stat_t;
typedef enum { MOTOR_OFF = 0, MOTOR_IDLE, MOTOR_RUNNING, MOTOR_POWER_TIMEOUT_START, MOTOR_POWER_TIMEOUT_COUNTDOWN } motorPowerState;
typedef enum { MOTOR_DISABLED = 0, MOTOR_ALWAYS_POWERED, MOTOR_POWERED_IN_CYCLE, MOTOR_POWERED_ONLY_WHEN_MOVING } cmMotorPowerMode;
enum { MOTION_STOP, MOTION_RUN, MOTION_HOLD };
typedef struct { volatile uint8_t OUT; } VPORT_t;
VPORT_t vp[4];
#define PORT_MOTOR_1_VPORT vp[0]
#define PORT_MOTOR_2_VPORT vp[1]
#define PORT_MOTOR_3_VPORT vp[2]
#define PORT_MOTOR_4_VPORT vp[3]
struct { struct { cmMotorPowerMode power_mode; } mot[MOTORS]; float motor_power_timeout; } st_cfg;
struct { struct { motorPowerState power_state; uint32_t power_systick; } mot[MOTORS];
         uint32_t energize_start, energize_duration; uint8_t energize_mask; uint32_t dda_ticks_downcount; } st_run;
uint8_t st_runtime_isbusy(void) { return st_run.dda_ticks_downcount != 0; }
static int motion_state = MOTION_STOP;
int cm_get_motion_state(void) { return motion_state; }
void sr_request_status_report(int r) { (void)r; }
static uint32_t now = 10000;
static int load_on_next_tick;
static void loader_starts_a_segment(void) {      /* _load_move(): stepper.c:889-893 for MOTOR_1 */
    PORT_MOTOR_1_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
    st_run.mot[MOTOR_1].power_state = MOTOR_POWER_TIMEOUT_START;
    st_run.dda_ticks_downcount = 250; motion_state = MOTION_RUN;
}
uint32_t SysTickTimer_getValue(void) {
    uint32_t t = now;
    if (load_on_next_tick) { load_on_next_tick = 0; loader_starts_a_segment(); }   /* lands after the read */
    return t;
}
'''

POWER_MAIN = r'''
int main(void) {
    for (int m = 0; m < MOTORS; m++) { st_cfg.mot[m].power_mode = MOTOR_POWERED_ONLY_WHEN_MOVING; vp[m].OUT = MOTOR_ENABLE_BIT_bm; }
    st_cfg.motor_power_timeout = 0.5;
    /* motor 1 stopped long ago: countdown running and expired */
    PORT_MOTOR_1_VPORT.OUT &= ~MOTOR_ENABLE_BIT_bm;
    st_run.mot[MOTOR_1].power_state = MOTOR_POWER_TIMEOUT_COUNTDOWN;
    st_run.mot[MOTOR_1].power_systick = now - 1;
    load_on_next_tick = 1;                       /* the next move's first segment loads right then */
    st_motor_power_callback();
    assert((PORT_MOTOR_1_VPORT.OUT & MOTOR_ENABLE_BIT_bm) == 0);   /* stock: disabled while stepping */
    return 0;
}
'''


def _cc():
    return shutil.which('cc')


class MotorPowerCheck(unittest.TestCase):
    def test_e184_power_timeout_disables_a_motor_that_just_started(self):
        if not _cc():
            self.skipTest('host C compiler not installed')
        stepper = (FIRMWARE / 'stepper.c').read_text()
        code = POWER_PRELUDE
        for sig in (r'static void _deenergize_motor\(', r'static void _energize_motor\(',
                    r'static uint8_t _motor_is_enabled\(', r'stat_t st_motor_power_callback\('):
            code += function(stepper, sig + DEF) + '\n'
        code += POWER_MAIN
        with tempfile.TemporaryDirectory() as d:
            c, exe = Path(d) / 'power.c', Path(d) / 'power'
            c.write_text(code)
            subprocess.run([_cc(), '-std=gnu99', '-Wall', '-Wno-unused-function', '-fsanitize=address,undefined',
                            str(c), '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
