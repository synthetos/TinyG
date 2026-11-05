"""E157: a jog starts only from idle or a completed feedhold, checked before any jog state changes.

The real _jog_allowed() and cm_run_jogx() are extracted from canonical_machine.c. Each machine state
is set directly, so the interleavings are fixed: a running or held-in-progress program, a homing or
probing cycle, a final segment still running after the last block was freed, and a prepared segment
owned by the loader are all refused with jogging_dest and the cycle untouched.
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
typedef int stat_t;
#define STAT_OK 0
#define STAT_COMMAND_NOT_ACCEPTED 106
stat_t status_code;
#define ritorno(a) if ((status_code = (a)) != STAT_OK) { return (status_code); }
enum { CYCLE_OFF, CYCLE_MACHINING, CYCLE_PROBE, CYCLE_HOMING, CYCLE_JOG };
enum { MOTION_STOP, MOTION_RUN, MOTION_HOLD };
enum { FEEDHOLD_OFF, FEEDHOLD_REQUESTED, FEEDHOLD_SYNC, FEEDHOLD_DECEL_CONTINUE, FEEDHOLD_DECEL_TO_ZERO,
       FEEDHOLD_PENDING, FEEDHOLD_HOLD };
enum { PREP_BUFFER_OWNED_BY_LOADER, PREP_BUFFER_OWNED_BY_EXEC };
#define AXIS_X 0
typedef struct { float value_flt; } nvObj_t;
struct { int cycle_state, motion_state, hold_state; float jogging_dest; } cm;
struct { int buffer_state; } st_pre;
static bool runnable, runtime_idle;
static int jog_starts;
bool mp_has_runnable_buffer(void) { return runnable; }
bool mp_runtime_is_idle(void) { return runtime_idle; }
stat_t set_flt(nvObj_t *nv) { cm.jogging_dest = nv->value_flt; return STAT_OK; }
stat_t cm_jogging_cycle_start(uint8_t axis) { (void)axis; jog_starts++; cm.cycle_state = CYCLE_JOG; return STAT_OK; }
'''

EXERCISE = r'''
static void idle(void) {
    cm.cycle_state = CYCLE_OFF; cm.motion_state = MOTION_STOP; cm.hold_state = FEEDHOLD_OFF;
    runnable = false; runtime_idle = true; st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;
    cm.jogging_dest = -1; jog_starts = 0;
}
static stat_t jog(void) { nvObj_t nv = { 3.0f }; return cm_run_jogx(&nv); }
static void refused(void) {
    int cycle = cm.cycle_state;
    assert(jog() == STAT_COMMAND_NOT_ACCEPTED && cm.jogging_dest == -1 && jog_starts == 0 && cm.cycle_state == cycle);
}
int main(void) {
    idle(); assert(jog() == STAT_OK && cm.jogging_dest == 3.0f && jog_starts == 1);
    idle(); cm.cycle_state = CYCLE_MACHINING; cm.motion_state = MOTION_RUN; runnable = true; runtime_idle = false; refused();
    idle(); runnable = true; refused();                                 /* queued, not started yet */
    idle(); runtime_idle = false; refused();                            /* final segment still running, block freed */
    idle(); st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER; refused(); /* prepared segment waiting for the loader */
    idle(); cm.cycle_state = CYCLE_HOMING; refused();
    idle(); cm.cycle_state = CYCLE_PROBE; refused();
    idle(); cm.cycle_state = CYCLE_JOG; refused();
    idle(); cm.cycle_state = CYCLE_MACHINING; cm.motion_state = MOTION_HOLD; cm.hold_state = FEEDHOLD_DECEL_TO_ZERO; refused();
    idle(); cm.cycle_state = CYCLE_MACHINING; cm.motion_state = MOTION_HOLD; cm.hold_state = FEEDHOLD_PENDING; refused();
    /* a completed feedhold may jog out (the held queue is discarded by the jog, as before) */
    idle(); cm.cycle_state = CYCLE_MACHINING; cm.motion_state = MOTION_HOLD; cm.hold_state = FEEDHOLD_HOLD; runnable = true;
    assert(jog() == STAT_OK && jog_starts == 1);
    idle(); cm.cycle_state = CYCLE_HOMING; cm.hold_state = FEEDHOLD_HOLD; refused();
    return 0;
}
'''


class JogAllowed(unittest.TestCase):
    def test_jog_only_from_idle_or_completed_hold(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        source = (ROOT / "firmware/tinyg/canonical_machine.c").read_text()
        code = PRELUDE
        for signature in (r"static stat_t _jog_allowed\(", r"stat_t cm_run_jogx\("):
            code += function(source, signature + r"[^;{}]*\)(?=\s*\{)") + "\n"
        code += EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-e157-") as tmp:
            c, exe = Path(tmp) / "jog.c", Path(tmp) / "jog"
            c.write_text(code)
            subprocess.run([cc, "-std=gnu99", "-Wall", "-Wno-unused-function", "-fsanitize=address,undefined","-fstrict-flex-arrays=3",
                            str(c), "-o", str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
