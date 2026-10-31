"""Check alarm decisions against actual AVR timer/prep predicates.

These are deterministic state tests, not a simulation of interrupt scheduling,
step delivery, or latency. In particular, the prepared-only fixture exercises
the alarm decision; it does not claim the main loop can outrun the HI loader.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(os.environ.get("TINYG_SOURCE", Path(__file__).resolve().parents[2]))


class AlarmFinalSegment(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        firmware = ROOT / "firmware/tinyg"
        machine = (firmware / "canonical_machine.c").read_text()
        stepper = (firmware / "stepper.c").read_text()
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define __AVR 1
enum { STAT_OK, MACHINE_READY, MACHINE_ALARM, MACHINE_PANIC, MACHINE_SHUTDOWN };
enum { MOTION_STOP, MOTION_RUN, MOTION_HOLD };
enum { FEEDHOLD_OFF, FEEDHOLD_REQUESTED, FEEDHOLD_HOLD, FEEDHOLD_DECEL_TO_ZERO };
enum { FLUSH_OFF, FLUSH_REQUESTED };
enum { PREP_BUFFER_OWNED_BY_EXEC, PREP_BUFFER_OWNED_BY_LOADER };
enum { BLOCK_TYPE_NULL, BLOCK_TYPE_ALINE, BLOCK_TYPE_DWELL };
#define STEP_TIMER_DISABLE 0
typedef int stat_t;
struct { int machine_state, motion_state, hold_state, queue_flush_state;
         bool end_hold_requested, queue_flush_input; } cm;
struct { unsigned dda_ticks_downcount; } st_run;
struct { int block_type, buffer_state; } st_pre;
struct { uint8_t CTRLA; } TIMER_DDA;
uint8_t SREG;
int resets, holds, requests, spindle_off, coolant_off, reports, flushes, input_flushes;
int inject_running;
void fake_cli(void) {
    // A pending HI loader can run before interrupts are masked.
    if (inject_running && (SREG & 0x80)) {
        TIMER_DDA.CTRLA=1; st_run.dda_ticks_downcount=10;
        st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;
    }
    SREG &= ~0x80;
}
#define cli() fake_cli()
void cm_set_motion_state(int state) { assert(!(SREG & 0x80)); cm.motion_state=state; }
void stepper_reset(void) {
    resets++; TIMER_DDA.CTRLA=0; st_run.dda_ticks_downcount=0;
    st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;
}
void cm_request_feedhold(void) { holds++; }
void cm_request_planner_flush(void) { requests++; }
void spindle_reset(void) { spindle_off++; }
void cm_coolant_off_immediate(void) { coolant_off++; }
void rpt_exception(int status, const char *msg) { (void)status; (void)msg; reports++; }
void xio_flush_input(void) { input_flushes++; }
void cm_queue_flush(void) { cm.queue_flush_state=FLUSH_OFF; flushes++; }
'''
        for source, signature in (
            (stepper, r'bool st_aline_in_flight\(\)'),
            (stepper, r'uint8_t st_runtime_isbusy\(\)'),
            ((firmware / "plan_line.c").read_text(), r'bool mp_runtime_is_idle\(\)'),
            (machine, r'stat_t cm_alarm\(const stat_t status, const char \*msg\)'),
            (machine, r'static void _run_requested_queue_flush\(\)(?=\s*\{)'),
        ):
            code += function(source, signature) + "\n"
        code += r'''
int main(int argc, char **argv) {
    if (argc!=4) return 2;
    int scenario=atoi(argv[1]); SREG=atoi(argv[2]);
    cm.machine_state=atoi(argv[3]); cm.motion_state=MOTION_STOP;
    if (scenario==1 || scenario==7) {
        st_pre.block_type=BLOCK_TYPE_ALINE;
        st_pre.buffer_state=PREP_BUFFER_OWNED_BY_LOADER;
    }
    if (scenario==2 || scenario==4 || scenario==5) {
        TIMER_DDA.CTRLA=1; st_run.dda_ticks_downcount=10;
    }
    if (scenario==3) {
        st_pre.block_type=BLOCK_TYPE_DWELL;
        st_run.dda_ticks_downcount=10;
    }
    if (scenario==4) cm.motion_state=MOTION_RUN;
    if (scenario==5) {
        cm.motion_state=MOTION_HOLD; cm.hold_state=FEEDHOLD_DECEL_TO_ZERO;
    }
    if (scenario==6) {
        // Stale ALINE metadata while exec owns prep is not a live segment.
        st_pre.block_type=BLOCK_TYPE_ALINE;
    }
    inject_running=(scenario==7);
    int result=cm_alarm(99,"test");
    int delayed=-1, completed=-1;
    if (scenario==2) {
        _run_requested_queue_flush(); delayed=flushes;
        TIMER_DDA.CTRLA=0; st_run.dda_ticks_downcount=0;
        _run_requested_queue_flush(); completed=flushes;
        assert(input_flushes==0);           // an alarm's flush keeps host input (E145)
        cm.queue_flush_state=FLUSH_REQUESTED; cm.queue_flush_input=true;   // a host '%'
        _run_requested_queue_flush();
        assert(input_flushes==1 && !cm.queue_flush_input);
    }
    printf("%d %d %d %d %d %d %d %d %d %d %d %d\n", result,
           cm.machine_state,cm.motion_state,cm.hold_state,resets,holds,requests,
           SREG,spindle_off+coolant_off+reports,delayed,completed,cm.queue_flush_state);
    return 0;
}
'''
        path = Path(cls.tmp.name) / "alarm.c"
        path.write_text(code)
        cls.exe = path.with_suffix("")
        subprocess.run(["cc", "-std=c99", "-Wall", "-Werror",
                        "-fsanitize=address,undefined", str(path), "-o", str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_case(self, scenario, sreg=128, state=1):
        out = subprocess.check_output([str(self.exe), str(scenario), str(sreg), str(state)], text=True)
        return tuple(map(int, out.split()))

    def test_last_segment_is_not_reset(self):
        for scenario in (1, 2, 7):
            for sreg in (0, 128):
                with self.subTest(scenario=scenario, sreg=sreg):
                    out = self.run_case(scenario, sreg)
                    self.assertEqual(out[:9], (99, 2, 2, 2, 0, 0, 0, sreg, 3))
                    if scenario == 2:
                        self.assertEqual(out[9:], (0, 1, 0))
                    else:
                        self.assertEqual(out[-1], 1)

    def test_idle_dwell_and_stale_prep_are_reset(self):
        for scenario in (0, 3, 6):
            for sreg in (0, 128):
                with self.subTest(scenario=scenario, sreg=sreg):
                    self.assertEqual(self.run_case(scenario, sreg),
                                     (99, 2, 2, 2, 1, 0, 0, sreg, 3, -1, -1, 1))

    def test_running_and_decelerating_use_controlled_hold(self):
        for scenario, motion, hold in ((4, 1, 0), (5, 2, 3)):
            for sreg in (0, 128):
                with self.subTest(scenario=scenario, sreg=sreg):
                    self.assertEqual(self.run_case(scenario, sreg),
                                     (99, 2, motion, hold, 0, 1, 1, sreg, 3, -1, -1, 0))

    def test_existing_terminal_states_are_untouched(self):
        for state in (2, 3, 4):
            with self.subTest(state=state):
                self.assertEqual(self.run_case(0, state=state),
                                 (0, state, 0, 0, 0, 0, 0, 128, 0, -1, -1, 0))


if __name__ == "__main__":
    unittest.main()
