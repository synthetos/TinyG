"""E166: one report, one instant.

A status report reads its positions one axis at a time. Interrupts change what it reads: the exec
moves the runtime position and loads the next block's offsets and units, the step loader runs
queued offset changes and the program end, and a motion state change switches the model a report
reads. The real snapshot code and position getters from canonical_machine.c are compiled with
cm_set_motion_state(); the runtime reads are stand-ins that can run an "interrupt" in the middle of
the copy, doing what the firmware's interrupt path does (including moving cm_position_epoch on).

Each case makes a change either during the copy or between two getters of one report, and checks
every value comes from one instant. The source checks that every interrupt path that changes
reported state moves the epoch on, and that every return in the report builders ends the snapshot.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function, region

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / "firmware/tinyg"
DEF = r"[^;{}]*\)(?=\s*(?://[^\n]*\s*)?\{)"

PRELUDE = r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define AXES 6
#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2
#define MM_PER_INCH 25.4
#define GET_TABLE_WORD(x) 3
enum { INCHES, MILLIMETERS };
typedef enum { MOTION_STOP, MOTION_RUN, MOTION_HOLD } cmMotionState;
enum { CYCLE_OFF, CYCLE_MACHINING };
enum { TYPE_FLOAT = 4 };
typedef int stat_t;
#define STAT_OK 0
typedef struct { int units_mode; float work_offset[AXES]; } GCodeState_t;
typedef struct { int index; float value_flt; int precision, valuetype; } nvObj_t;
struct { int motion_state, cycle_state; GCodeState_t gm, *am; struct { float position[AXES], tool_offset[AXES]; } gmx; } cm;
struct { float position[AXES]; GCodeState_t gm; } mr;
#define MODEL (GCodeState_t *)&cm.gm
#define RUNTIME (GCodeState_t *)&mr.gm
#define ACTIVE_MODEL cm.am
extern volatile uint8_t cm_position_epoch;
#define cm_position_changed() (cm_position_epoch++)
static int cm_get_units_mode(const GCodeState_t *gcode_state) { return gcode_state->units_mode; }
static int8_t _get_axis(int index) { return (int8_t)index; }
static const char *const msg_unit[] = { "in", "mm" };
stat_t _get_msg_helper(nvObj_t *nv, const char *const msg_array[], uint8_t value) { (void)msg_array; nv->value_flt = value; return STAT_OK; }
float cm_get_active_coord_offset(uint8_t axis) { return cm.gm.work_offset[axis]; }

/* an "interrupt" that fires once, when the copy reads the runtime of trigger_axis */
static void (*pending)(void);
static int trigger_axis, reads_of_x;
static void tick(uint8_t axis) {
    if (axis == AXIS_X) reads_of_x++;
    if (pending && axis == trigger_axis) { void (*isr)(void) = pending; pending = 0; isr(); }
}
float mp_get_runtime_absolute_position(uint8_t axis) { tick(axis); return mr.position[axis]; }
float mp_get_runtime_work_position(uint8_t axis) { return mr.position[axis] - mr.gm.work_offset[axis]; }
float mp_get_runtime_work_offset(uint8_t axis) { return mr.gm.work_offset[axis]; }
'''

EXERCISE = r'''
static float get(stat_t (*getter)(nvObj_t *), int axis) { nvObj_t nv = {axis, 0, 0, 0}; getter(&nv); return nv.value_flt; }
static void near(float a, float b) { assert(fabsf(a - b) < 1e-5f); }

static void running(float x, float y) {            /* a G1 in progress; the model is already at its end */
    memset(&cm, 0, sizeof(cm)); memset(&mr, 0, sizeof(mr));
    cm.gm.units_mode = mr.gm.units_mode = MILLIMETERS;
    cm.cycle_state = CYCLE_MACHINING; cm_set_motion_state(MOTION_RUN);
    mr.position[AXIS_X] = x; mr.position[AXIS_Y] = y;
    cm.gmx.position[AXIS_X] = 0.1f; cm.gmx.position[AXIS_Y] = 0.1f;
    pending = 0; reads_of_x = 0;
}
static void segment_to(float x, float y) { mr.position[AXIS_X] = x; mr.position[AXIS_Y] = y; }
static void program_end(void) {                     /* the last segment ran, then the queued program end */
    segment_to(0.1f, 0.1f);
    cm_set_motion_state(MOTION_STOP); cm.cycle_state = CYCLE_OFF;
}
/* interrupt paths, as the firmware runs them */
static void isr_exec_segment(void) { segment_to(0.05f, 0.05f); cm_position_changed(); }
static void isr_loader_offsets(void) { mr.gm.work_offset[AXIS_X] = mr.gm.work_offset[AXIS_Y] = 3.0f; cm_position_changed(); }
static void isr_loader_program_end(void) { program_end(); cm_position_changed(); }
static void isr_offsets_unannounced(void) { mr.gm.work_offset[AXIS_X] = mr.gm.work_offset[AXIS_Y] = 3.0f; }

static void report(float *x, float *y, float *ox, float *oy, float *mx, float *my) {
    cm_position_snapshot_begin();
    *x = get(cm_get_pos, AXIS_X); *y = get(cm_get_pos, AXIS_Y);
    *ox = get(cm_get_ofs, AXIS_X); *oy = get(cm_get_ofs, AXIS_Y);
    *mx = get(cm_get_mpo, AXIS_X); *my = get(cm_get_mpo, AXIS_Y);
    cm_position_snapshot_end();
}

int main(void) {
    float x, y, ox, oy, mx, my;

    /* DURING the copy: the loader changes the offsets after X was copied */
    running(5.0f, 5.0f); mr.gm.work_offset[AXIS_X] = mr.gm.work_offset[AXIS_Y] = 1.0f;
    pending = isr_loader_offsets; trigger_axis = AXIS_Y;
    report(&x, &y, &ox, &oy, &mx, &my);
    near(x, 2.0f); near(y, 2.0f); near(ox, 3.0f); near(oy, 3.0f); near(mx, 5.0f); near(my, 5.0f);
    assert(reads_of_x == 2);                                        /* copied once more, no more */

    /* control: the same change without the epoch moving gives the mixed report seen in review */
    running(5.0f, 5.0f); mr.gm.work_offset[AXIS_X] = mr.gm.work_offset[AXIS_Y] = 1.0f;
    pending = isr_offsets_unannounced; trigger_axis = AXIS_Y;
    report(&x, &y, &ox, &oy, &mx, &my);
    near(x, 4.0f); near(y, 2.0f);

    /* DURING the copy: the program ends (the loader runs it), so the model the report reads changes */
    running(0.0f, 0.0f);
    pending = isr_loader_program_end; trigger_axis = AXIS_Y;
    report(&x, &y, &ox, &oy, &mx, &my);
    near(x, 0.1f); near(y, 0.1f); near(mx, 0.1f); near(my, 0.1f);   /* all from the model, idle */

    /* DURING the copy: a segment runs */
    running(0.02f, 0.02f);
    pending = isr_exec_segment; trigger_axis = AXIS_Y;
    report(&x, &y, &ox, &oy, &mx, &my);
    near(x, 0.05f); near(y, 0.05f);

    /* BETWEEN getters: the move ends after posx; posy still from the report's instant */
    running(0.0f, 0.0f);
    cm_position_snapshot_begin();
    x = get(cm_get_pos, AXIS_X); mx = get(cm_get_mpo, AXIS_X);
    program_end();
    y = get(cm_get_pos, AXIS_Y); my = get(cm_get_mpo, AXIS_Y);
    cm_position_snapshot_end();
    assert(x == 0.0f && y == 0.0f && mx == 0.0f && my == 0.0f);   /* on the x=y path, not (0, 0.1) */
    near(get(cm_get_pos, AXIS_Y), 0.1f);                            /* a single read after it: the model */

    /* BETWEEN getters: a block with another coordinate system loads */
    running(5.0f, 5.0f); mr.gm.work_offset[AXIS_X] = mr.gm.work_offset[AXIS_Y] = 1.0f;
    cm_position_snapshot_begin();
    x = get(cm_get_pos, AXIS_X); ox = get(cm_get_ofs, AXIS_X);
    mr.gm.work_offset[AXIS_X] = mr.gm.work_offset[AXIS_Y] = 3.0f; segment_to(6.0f, 6.0f);
    y = get(cm_get_pos, AXIS_Y); oy = get(cm_get_ofs, AXIS_Y); my = get(cm_get_mpo, AXIS_Y);
    cm_position_snapshot_end();
    near(x, 4.0f); near(y, 4.0f); near(ox, 1.0f); near(oy, 1.0f); near(y + oy, my);
    near(get(cm_get_ofs, AXIS_Y), 3.0f);

    /* BETWEEN getters: a G20 block loads; units and the unit report from the same instant */
    running(25.4f, 25.4f);
    cm_position_snapshot_begin();
    x = get(cm_get_pos, AXIS_X);
    float unit_before = get(cm_get_unit, 0);
    mr.gm.units_mode = INCHES;
    y = get(cm_get_pos, AXIS_Y);
    float unit_after = get(cm_get_unit, 0);
    cm_position_snapshot_end();
    near(x, 25.4f); near(y, 25.4f); assert(unit_before == MILLIMETERS && unit_after == MILLIMETERS);
    near(get(cm_get_pos, AXIS_Y), 1.0f); assert(get(cm_get_unit, 0) == INCHES);

    /* nesting: an inner pair does not release the outer report */
    running(0.0f, 0.0f);
    cm_position_snapshot_begin();
    cm_position_snapshot_begin();
    cm_position_snapshot_end();
    program_end();
    assert(get(cm_get_pos, AXIS_Y) == 0.0f);
    cm_position_snapshot_end();
    near(get(cm_get_pos, AXIS_Y), 0.1f);

    /* an unmatched end does not wrap the depth */
    cm_position_snapshot_end();
    running(0.0f, 0.0f);
    cm_position_snapshot_begin();
    program_end();
    assert(get(cm_get_pos, AXIS_X) == 0.0f);
    cm_position_snapshot_end();
    near(get(cm_get_pos, AXIS_X), 0.1f);
    return 0;
}
'''


def _bracketed(body):
    """Every return after the begin ends the snapshot first."""
    after = body[body.index("cm_position_snapshot_begin();"):]
    returns = len(re.findall(r"\breturn\b", after))
    ends = len(re.findall(r"cm_position_snapshot_end\(\);", after))
    return returns >= 1 and returns == ends


class PositionSnapshot(unittest.TestCase):
    def test_one_report_one_instant(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        cmc = (FIRMWARE / "canonical_machine.c").read_text()
        code = PRELUDE
        for signature in (r"void cm_set_motion_state\(", r"float cm_get_work_offset\(",
                          r"float cm_get_absolute_position\(", r"static float _get_work_position_mm\(",
                          r"float cm_get_work_position\("):
            code += function(cmc, signature + DEF) + "\n"
        code += region(cmc, r"static GCodeState_t \*_position_model\(\)", r"stat_t cm_get_ofs\(" + DEF) + "\n"
        code += function(cmc, r"stat_t cm_get_unit\(" + DEF) + "\n"
        code += EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-e166-") as tmp:
            c, exe = Path(tmp) / "snap.c", Path(tmp) / "snap"
            c.write_text(code)
            subprocess.run([cc, "-std=gnu99", "-Wall", "-Werror", "-Wno-unused-function",
                            "-fsanitize=address,undefined", str(c), "-o", str(exe), "-lm"], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_reports_and_groups_bracket_every_return(self):
        report = (FIRMWARE / "report.c").read_text()
        config = (FIRMWARE / "config.c").read_text()
        for source, signature in ((report, r"static stat_t _populate_unfiltered_status_report\("),
                                  (report, r"static bool _populate_filtered_status_report\("),
                                  (config, r"stat_t get_grp\(")):
            body = function(source, signature + DEF)
            self.assertTrue(_bracketed(body), signature)
        m114 = function((FIRMWARE / "canonical_machine.c").read_text(), r"stat_t cm_get_position\(" + DEF)
        self.assertLess(m114.index("cm_position_snapshot_end();"), m114.index("cs.comm_mode"))

    def test_interrupt_writers_move_the_epoch(self):
        stepper = (FIRMWARE / "stepper.c").read_text()
        cmc = (FIRMWARE / "canonical_machine.c").read_text()
        exec_isr = function(stepper, r"ISR\(TIMER_EXEC_ISR_vect\)")
        self.assertLess(exec_isr.index("mp_exec_move()"), exec_isr.index("cm_position_changed();"))
        loader = function(stepper, r"static void _load_move\(" + DEF)
        command = loader[loader.index("mp_runtime_command(st_pre.bf);"):]
        self.assertTrue(re.match(r"mp_runtime_command\(st_pre\.bf\);\s*cm_position_changed\(\);", command))
        self.assertIn("cm_position_changed();", function(cmc, r"void cm_set_motion_state\(" + DEF))


if __name__ == "__main__":
    unittest.main()
