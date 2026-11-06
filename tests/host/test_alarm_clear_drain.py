"""E144: clearing an alarm ends a latched hold and the cycle only when the motion system is drained.

Drained = no runnable planner block, an idle runtime AND no prepared segment owned by the loader
(the last block can be freed before its final segment loads). The real cm_clear() is extracted
from canonical_machine.c and run against controlled planner/runtime/prep states.
"""
import pathlib
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = pathlib.Path(__file__).resolve().parents[2]

PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
enum {MACHINE_READY, MACHINE_ALARM, MACHINE_PROGRAM_STOP, MACHINE_SHUTDOWN};
enum {FEEDHOLD_OFF, FEEDHOLD_HOLD};
enum {PREP_BUFFER_OWNED_BY_LOADER, PREP_BUFFER_OWNED_BY_EXEC};
struct {int machine_state, hold_state; bool end_hold_requested;} cm;
struct {int buffer_state;} st_pre;
static bool runnable, runtime_idle;
static int exits, cycle_ends;
bool mp_has_runnable_buffer(void) { return runnable; }
bool mp_runtime_is_idle(void) { return runtime_idle; }
void mp_exit_hold_state(void) { cm.hold_state = FEEDHOLD_OFF; exits++; }
void cm_cycle_end(void) { cycle_ends++; }
'''

EXERCISE = r'''
static void arm(bool r, bool idle, int prep) {
    cm.machine_state = MACHINE_ALARM; cm.hold_state = FEEDHOLD_HOLD; cm.end_hold_requested = true;
    runnable = r; runtime_idle = idle; st_pre.buffer_state = prep; exits = cycle_ends = 0;
}
int main(void) {
    /* empty planner + idle runtime, but the last segment still waits for the loader: keep the hold */
    arm(false, true, PREP_BUFFER_OWNED_BY_LOADER); cm_clear();
    assert(cm.machine_state == MACHINE_PROGRAM_STOP && cm.hold_state == FEEDHOLD_HOLD && !exits && !cycle_ends);
    /* a block still queued: keep the hold */
    arm(true, true, PREP_BUFFER_OWNED_BY_EXEC); cm_clear();
    assert(cm.hold_state == FEEDHOLD_HOLD && !exits && !cycle_ends);
    /* runtime still stepping: keep the hold */
    arm(false, false, PREP_BUFFER_OWNED_BY_EXEC); cm_clear();
    assert(cm.hold_state == FEEDHOLD_HOLD && !exits && !cycle_ends);
    /* genuinely drained: end the hold and the cycle */
    arm(false, true, PREP_BUFFER_OWNED_BY_EXEC); cm_clear();
    assert(cm.hold_state == FEEDHOLD_OFF && exits == 1 && cycle_ends == 1 && !cm.end_hold_requested);
    /* drained with no hold: still ends the cycle */
    arm(false, true, PREP_BUFFER_OWNED_BY_EXEC); cm.hold_state = FEEDHOLD_OFF; cm_clear();
    assert(exits == 0 && cycle_ends == 1);
    /* not alarmed: SHUTDOWN clears to READY and touches nothing else */
    arm(false, true, PREP_BUFFER_OWNED_BY_EXEC); cm.machine_state = MACHINE_SHUTDOWN; cm_clear();
    assert(cm.machine_state == MACHINE_READY && cm.hold_state == FEEDHOLD_HOLD && !cycle_ends);
    return 0;
}
'''


class AlarmClearDrain(unittest.TestCase):
    def test_clear_ends_hold_only_when_drained(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        source = (ROOT / "firmware/tinyg/canonical_machine.c").read_text()
        code = PRELUDE + function(source, r"void cm_clear\(\)") + EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-e144-") as tmp:
            c, exe = pathlib.Path(tmp) / "clear.c", pathlib.Path(tmp) / "clear"
            c.write_text(code)
            subprocess.run([cc, "-std=c99", "-Wall", "-Werror", "-fsanitize=address,undefined","-fstrict-flex-arrays=3",
                            str(c), "-o", str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
