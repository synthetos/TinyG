"""Exercise the real homing dispatcher around feedhold completion, without hardware.

Run: python3 tests/host/test_homing_hold_completion.py
The fixture models a planner lease and the next homing phase. Firmware supplies
its busy predicates, homing dispatch, and queue-flush implementation. The exec
completion event is explicit; this does not emulate timers or switch interrupts.
"""
import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


def function(source, name):
    match = re.search(r"(?m)^[\w ]+\b" + name + r"\([^\n]*\)\s*\{", source)
    if not match:
        raise AssertionError(f"missing function {name}")
    return source[match.start():source.index("\n}", match.end()) + 2]


PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
typedef int stat_t;
enum {STAT_OK, STAT_NOOP, STAT_EAGAIN};
enum {CYCLE_OFF, CYCLE_HOMING, CYCLE_MACHINING};
enum {BLOCK_IDLE, BLOCK_INITIALIZING, BLOCK_RUNNING};
enum {FLUSH_OFF, FLUSH_REQUESTED};
#define AXES 6
#define AXIS_X 0
static struct {int cycle_state, hold_state, queue_flush_state;} cm;
static struct {int block_state;} mr;
static struct {stat_t (*func)(int8_t); int8_t axis; float saved_jerk;} hm;
static bool moving, planner_live;
static int cm_is_alarmed(void) {return STAT_OK;}
static void cm_set_axis_jerk(int8_t axis,float jerk) {}
static stat_t _homing_finalize_exit(int8_t axis) {return STAT_OK;}
static void _take_input_flush_request(void) {}
static unsigned dispatched, flushed, reports, positions, ended;
static bool st_runtime_isbusy(void) {return moving;}
static void st_prep_null(void) {}
void st_cancel_dwell(void) {}
static void mp_flush_planner(void) {planner_live=false;flushed++;}
static float mp_get_runtime_absolute_position(uint8_t axis) {return axis+0.25f;}
static void cm_set_position(uint8_t axis,float value) {assert(value==axis+0.25f);positions++;}
static void cm_end_hold(void) {cm.hold_state=FEEDHOLD_OFF;ended++;}
static void qr_request_queue_report(int value) {assert(value==0);reports++;}
'''

EXERCISE = r'''
static stat_t next_phase(int8_t axis) {
    assert(axis==2);dispatched++;
    cm_queue_flush();
    assert(!planner_live); /* next homing move replaces the old search block */
    planner_live=true;
    return STAT_EAGAIN;
}
static void reset(void) {
    cm.cycle_state=CYCLE_HOMING;cm.hold_state=FEEDHOLD_OFF;
    cm.queue_flush_state=FLUSH_OFF;mr.block_state=BLOCK_IDLE;
    moving=false;planner_live=true;
    dispatched=flushed=reports=positions=ended=0;
    hm.func=next_phase;hm.axis=2;
}
int main(void) {
    /* Last step completed, but the exec ISR has not finalized the held block. */
    reset();cm.hold_state=FEEDHOLD_PENDING;mr.block_state=BLOCK_INITIALIZING;
    assert(!cm_get_runtime_busy());
    for(unsigned visit=0;visit<8;visit++) {
        assert(cm_homing_callback()==STAT_EAGAIN);
        if(dispatched||flushed||!planner_live) {
            fputs("E114: homing flushed the held block before exec finalized HOLD\n",stderr);
            return 2;
        }
    }
    /* Exec can now finish because its planner block is still present. */
    assert(planner_live);mr.block_state=BLOCK_IDLE;cm.hold_state=FEEDHOLD_HOLD;
    assert(cm_homing_callback()==STAT_EAGAIN);
    assert(dispatched==1&&flushed==1&&reports==1&&positions==AXES&&ended==1);
    assert(cm.hold_state==FEEDHOLD_OFF);

    /* A lone preceding dwell can have freed its planner slot. Homing entry has
       set CYCLE_HOMING, so the actual runtime-busy check still delays the phase. */
    reset();moving=true;planner_live=false;
    assert(cm_homing_callback()==STAT_EAGAIN);
    assert(dispatched==0&&flushed==0&&reports==0);

    unsigned cases=0;
    for(int cycle=CYCLE_OFF;cycle<=CYCLE_MACHINING;cycle++)
    for(int hold=FEEDHOLD_OFF;hold<=FEEDHOLD_HOLD;hold++)
    for(int block=BLOCK_IDLE;block<=BLOCK_RUNNING;block++)
    for(int active=0;active<2;active++) {
        reset();cm.cycle_state=cycle;cm.hold_state=hold;
        mr.block_state=block;moving=active;
        bool allowed=cycle==CYCLE_HOMING&&!active&&block!=BLOCK_RUNNING&&
                     (hold==FEEDHOLD_OFF||hold==FEEDHOLD_HOLD);
        stat_t result=cm_homing_callback();
        assert(result==(cycle==CYCLE_HOMING?STAT_EAGAIN:STAT_NOOP));
        assert(dispatched==(unsigned)allowed&&flushed==(unsigned)allowed);
        if(!allowed)assert(planner_live&&reports==0&&positions==0&&ended==0);
        cases++;
    }
    printf("E114: delayed exec completion and %u dispatcher states passed\n",cases);
    return 0;
}
'''


class HomingHoldTests(unittest.TestCase):
    def test_dispatch_waits_for_hold_completion(self):
        compiler = shutil.which("cc")
        if not compiler:
            self.skipTest("host C compiler not installed")
        firmware = ROOT / "firmware/tinyg"
        header = (firmware / "canonical_machine.h").read_text()
        states = re.search(r"typedef enum \{\s*FEEDHOLD_OFF.*?\}", header, re.S).group()
        definitions = states.replace("typedef ", "", 1) + ";\n"
        line = (firmware / "plan_line.c").read_text()
        machine = (firmware / "canonical_machine.c").read_text()
        homing = (firmware / "cycle_homing.c").read_text()
        actual = "\n".join((function(line, "mp_get_runtime_busy"),
                              function(line, "mp_runtime_is_idle"),
                              "static uint8_t cm_get_runtime_busy(void) {return mp_get_runtime_busy();}",
                              function(machine, "cm_queue_flush"),
                              function(homing, "cm_homing_callback")))
        with tempfile.TemporaryDirectory(prefix="tinyg-homing-") as directory:
            path = pathlib.Path(directory)
            (path / "check.c").write_text(definitions + PRELUDE + actual + EXERCISE)
            subprocess.run([compiler, "-std=c99", "-O1", "-fsanitize=address,undefined","-fstrict-flex-arrays=3",
                            str(path / "check.c"), "-o", str(path / "check")], check=True)
            result = subprocess.run([str(path / "check")], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            print(result.stdout.strip())


if __name__ == "__main__":
    unittest.main()
