"""E145: only a host '%' drops buffered input, and its request is consumed with the end of the flush.

The real cm_request_queue_flush(), cm_request_planner_flush(), cm_queue_flush() and
_run_requested_queue_flush() are extracted from canonical_machine.c. A '%' from the RX ISR is injected
while the planner is being flushed (before the atomic consume) and right after the flush; the direct
flush path (homing, panic) and a later alarm-only flush must never leave or act on a stale request.
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
#include <stdbool.h>
#include <stdint.h>
#define __AVR
enum { MOTION_STOP, MOTION_RUN, MOTION_HOLD };
enum { FEEDHOLD_OFF, FEEDHOLD_REQUESTED, FEEDHOLD_HOLD, FEEDHOLD_DECEL_TO_ZERO };
enum { FLUSH_OFF, FLUSH_REQUESTED };
#define AXIS_X 0
#define AXES 6
struct { int hold_state, queue_flush_state; bool end_hold_requested; volatile bool queue_flush_input; } cm;
uint8_t SREG = 0x80;
static int input_flushes, planner_flushes;
static int inject;                          /* 1: '%' during the planner flush, 2: '%' in cm_end_hold */
void cm_request_queue_flush(void);
void cm_request_planner_flush(void);
static void isr_percent(void) { cm_request_queue_flush(); }
static void cli(void) { SREG &= ~0x80; }
bool mp_runtime_is_idle(void) { return true; }
void mp_flush_planner(void) { planner_flushes++; if (inject == 1) { inject = 0; isr_percent(); } }
float mp_get_runtime_absolute_position(int axis) { (void)axis; return 0; }
void cm_set_position(int axis, float position) { (void)axis; (void)position; }
void cm_end_hold(void) { cm.hold_state = FEEDHOLD_OFF; if (inject == 2) { inject = 0; isr_percent(); } }
void qr_request_queue_report(int n) { (void)n; }
void xio_flush_input(void) { assert(SREG & 0x80); input_flushes++; }   /* main loop, interrupts on */
'''

EXERCISE = r'''
static void reset(void) {
    cm.hold_state = FEEDHOLD_HOLD; cm.queue_flush_state = FLUSH_OFF; cm.queue_flush_input = false;
    input_flushes = planner_flushes = 0; inject = 0; SREG = 0x80;
}
int main(void) {
    /* an alarm's flush (planner only) keeps input */
    reset(); cm_request_planner_flush(); _run_requested_queue_flush();
    assert(planner_flushes == 1 && input_flushes == 0 && cm.queue_flush_state == FLUSH_OFF && !cm.queue_flush_input);
    /* a host '%' drops input once */
    reset(); isr_percent(); _run_requested_queue_flush();
    assert(input_flushes == 1 && !cm.queue_flush_input && cm.queue_flush_state == FLUSH_OFF);
    /* '%' while an alarm's flush is pending: input dropped */
    reset(); cm_request_planner_flush(); isr_percent(); _run_requested_queue_flush();
    assert(input_flushes == 1 && !cm.queue_flush_input);
    /* '%' injected during the alarm's planner flush, and while its hold ends: still serviced */
    for (int point = 1; point <= 2; point++) {
        reset(); cm_request_planner_flush(); inject = point; _run_requested_queue_flush();
        assert(input_flushes == 1 && !cm.queue_flush_input && cm.queue_flush_state == FLUSH_OFF);
        assert(SREG & 0x80);
    }
    /* '%' after the flush ended (no hold any more): not honored, nothing stale left */
    reset(); cm_request_planner_flush(); _run_requested_queue_flush(); isr_percent();
    assert(input_flushes == 0 && !cm.queue_flush_input && cm.queue_flush_state == FLUSH_OFF);
    /* direct flush (homing, panic) services a pending '%' and leaves no flag for a later alarm */
    reset(); isr_percent(); cm_queue_flush();
    assert(input_flushes == 1 && !cm.queue_flush_input && cm.queue_flush_state == FLUSH_OFF);
    cm.hold_state = FEEDHOLD_HOLD; cm_request_planner_flush(); _run_requested_queue_flush();
    assert(input_flushes == 1);
    return 0;
}
'''


class QueueFlushInput(unittest.TestCase):
    def test_host_percent_is_consumed_with_the_flush(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        source = (ROOT / "firmware/tinyg/canonical_machine.c").read_text()
        code = PRELUDE
        for signature in (r"void cm_request_queue_flush\(\)", r"void cm_request_planner_flush\(\)",
                          r"void cm_queue_flush\(\)", r"static void _run_requested_queue_flush\(\)"):
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
