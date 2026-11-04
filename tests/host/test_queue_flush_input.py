"""E145: only a host '%' drops buffered input; its request is consumed with the end of the flush and
the input pool is reset only at the main-loop sequencing point.

The real cm_request_queue_flush(), cm_request_planner_flush(), cm_queue_flush(), _run_requested_queue_flush()
and cm_feedhold_sequencing_callback() are extracted from canonical_machine.c. A '%' from the RX ISR is
injected while the planner is being flushed (before the atomic consume) and right after the flush; a direct
flush (homing, panic, possibly in ISR context) must never reset the pool itself, and no stale request may
reach a later alarm-only flush.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function, RUNTIME_STUBS

ROOT = Path(__file__).resolve().parents[2]

PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define __AVR
enum { MOTION_STOP, MOTION_RUN, MOTION_HOLD };
enum { FEEDHOLD_OFF, FEEDHOLD_REQUESTED, FEEDHOLD_SYNC, FEEDHOLD_DECEL_CONTINUE, FEEDHOLD_DECEL_TO_ZERO,
       FEEDHOLD_PENDING, FEEDHOLD_HOLD };
enum { FLUSH_OFF, FLUSH_REQUESTED };
#define AXIS_X 0
#define AXES 6
#define STAT_OK 0
typedef int stat_t;
struct { int hold_state, queue_flush_state; bool end_hold_requested, job_kill_report_pending; volatile bool queue_flush_input, queue_flush_input_due, job_kill_requested; } cm;
#define STAT_KILL_JOB 207
#define PSTR(s) (s)
static int kill_alarms;
static int cm_alarm_P(int status, const char *msg) { (void)msg; kill_alarms += (status == STAT_KILL_JOB); return status; }
static void rpt_exception(int s,const char*m){cm_alarm_P(s,m);}
uint8_t SREG = 0x80;
static int input_flushes, planner_flushes, input_before_planner;
static int inject, in_direct_flush;                          /* 1: '%' during the planner flush, 2: '%' in cm_end_hold */
void cm_request_queue_flush(void);
void cm_request_planner_flush(void);
static void isr_percent(void) { cm_request_queue_flush(); }
static void cli(void) { SREG &= ~0x80; }
bool mp_runtime_is_idle(void) { return true; }
void mp_flush_planner(void) { planner_flushes++; input_before_planner = input_flushes; if (inject == 1) { inject = 0; isr_percent(); } }
float mp_get_runtime_absolute_position(int axis) { (void)axis; return 0; }
void cm_set_position(int axis, float position) { (void)axis; (void)position; }
void cm_end_hold(void) { cm.hold_state = FEEDHOLD_OFF; if (inject == 2) { inject = 0; isr_percent(); } }
void qr_request_queue_report(int n) { (void)n; }
void cm_start_hold(void) {}
void xio_flush_input(void) { assert((SREG & 0x80) && !in_direct_flush); input_flushes++; }   /* main loop only */
'''

EXERCISE = r'''
static void reset(void) {
    cm.hold_state = FEEDHOLD_HOLD; cm.queue_flush_state = FLUSH_OFF; cm.queue_flush_input = false;
    cm.queue_flush_input_due = false; cm.end_hold_requested = false; in_direct_flush = 0;
    input_flushes = planner_flushes = 0; inject = 0; SREG = 0x80;
}
int main(void) {
    /* an alarm's flush (planner only) keeps input */
    reset(); cm_request_planner_flush(); cm_feedhold_sequencing_callback();
    assert(planner_flushes == 1 && input_flushes == 0 && cm.queue_flush_state == FLUSH_OFF && !cm.queue_flush_input);
    /* a host '%' drops input once, before the planner work, so a line sent right after it is not cut */
    reset(); isr_percent(); cm_feedhold_sequencing_callback();
    assert(input_flushes == 1 && !cm.queue_flush_input && cm.queue_flush_state == FLUSH_OFF);
    assert(input_before_planner == 1);
    /* '%' while an alarm's flush is pending: input dropped */
    reset(); cm_request_planner_flush(); isr_percent(); cm_feedhold_sequencing_callback();
    assert(input_flushes == 1 && !cm.queue_flush_input);
    /* '%' injected during the alarm's planner flush, and while its hold ends: still serviced */
    for (int point = 1; point <= 2; point++) {
        reset(); cm_request_planner_flush(); inject = point; cm_feedhold_sequencing_callback();
        assert(input_flushes == 1 && !cm.queue_flush_input && cm.queue_flush_state == FLUSH_OFF);
        assert(SREG & 0x80);
    }
    /* '%' after the flush ended (no hold any more): not honored, nothing stale left */
    reset(); cm_request_planner_flush(); cm_feedhold_sequencing_callback(); isr_percent();
    assert(input_flushes == 0 && !cm.queue_flush_input && cm.queue_flush_state == FLUSH_OFF);
    /* direct flush (homing, panic) never resets the pool itself; it hands a pending '%' to the
       sequencing point and leaves no request for a later alarm-only flush */
    reset(); isr_percent(); in_direct_flush = 1; cm_queue_flush(); in_direct_flush = 0;
    assert(input_flushes == 0 && !cm.queue_flush_input && cm.queue_flush_input_due && cm.queue_flush_state == FLUSH_OFF);
    cm_feedhold_sequencing_callback();
    assert(input_flushes == 1 && !cm.queue_flush_input_due);
    cm.hold_state = FEEDHOLD_HOLD; cm_request_planner_flush(); cm_feedhold_sequencing_callback();
    assert(input_flushes == 1);
    /* a direct flush with no '%' pending hands nothing off */
    reset(); in_direct_flush = 1; cm_queue_flush(); in_direct_flush = 0; cm_feedhold_sequencing_callback();
    assert(input_flushes == 0 && !cm.queue_flush_input_due);
    /* ^d: the callback raises one kill-job alarm and consumes the request (E096) */
    reset(); cm.job_kill_requested = true; cm_feedhold_sequencing_callback();
    assert(kill_alarms == 1 && !cm.job_kill_requested);
    cm_feedhold_sequencing_callback(); assert(kill_alarms == 1);
    return 0;
}
'''


class QueueFlushInput(unittest.TestCase):
    def test_host_percent_is_consumed_with_the_flush(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        source = (ROOT / "firmware/tinyg/canonical_machine.c").read_text()
        code = PRELUDE + RUNTIME_STUBS
        for signature in (r"static void _take_input_flush_request\(\)", r"static void _service_input_flush\(\)",
                          r"void cm_request_queue_flush\(\)", r"void cm_request_planner_flush\(\)",
                          r"void cm_queue_flush\(\)", r"static void _run_requested_queue_flush\(\)",
                          r"stat_t cm_feedhold_sequencing_callback\(\)"):
            code += function(source, signature + r"(?=\s*\{)") + "\n"
        code += EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-e145-") as tmp:
            c, exe = Path(tmp) / "flush.c", Path(tmp) / "flush"
            c.write_text(code)
            subprocess.run([cc, "-std=c99", "-Wall", "-Werror", "-fsanitize=address,undefined", str(c), "-o", str(exe)],
                           check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
