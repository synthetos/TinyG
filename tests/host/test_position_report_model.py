"""E090: position reports never show the end point of motion that has been accepted but not started.

The real _position_model(), cm_get_pos(), cm_get_mpo(), cm_get_ofs() and the position helpers are
extracted from canonical_machine.c. The model is moved to a queued end point while the runtime
(where the machine is) stays at the start; nothing executes, so the interleaving is fixed, not raced.
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
#define AXES 6
#define AXIS_X 0
#define AXIS_Z 2
#define MM_PER_INCH 25.4
#define GET_TABLE_WORD(x) 3
enum { MILLIMETERS, INCHES };
enum { MOTION_STOP, MOTION_RUN, MOTION_HOLD };
enum { CYCLE_OFF, CYCLE_MACHINING };
enum { TYPE_FLOAT = 4 };
typedef int stat_t;
#define STAT_OK 0
typedef struct { int units_mode; float work_offset[AXES]; } GCodeState_t;
typedef struct { int index; float value_flt; int precision, valuetype; } nvObj_t;
struct { int motion_state, cycle_state; GCodeState_t gm, *am; struct { float position[AXES], tool_offset[AXES]; } gmx; } cm;
struct { GCodeState_t gm; } mr;
#define MODEL (GCodeState_t *)&cm.gm
#define RUNTIME (GCodeState_t *)&mr.gm
#define ACTIVE_MODEL cm.am
static float runtime_mm[AXES];
float mp_get_runtime_absolute_position(uint8_t axis) { return runtime_mm[axis]; }
float mp_get_runtime_work_position(uint8_t axis) { return runtime_mm[axis] - mr.gm.work_offset[axis]; }
float cm_get_active_coord_offset(uint8_t axis) { return cm.gm.work_offset[axis]; }
static int8_t _get_axis(int index) { return (int8_t)index; }
'''

EXERCISE = r'''
static float report(stat_t (*get)(nvObj_t *), int axis) { nvObj_t nv = {axis, 0, 0, 0}; get(&nv); return nv.value_flt; }
static void accepted_not_started(void) {   /* model at the queued end point, machine still at the start */
    cm.cycle_state = CYCLE_MACHINING; cm.motion_state = MOTION_STOP; cm.am = MODEL;
    cm.gmx.position[AXIS_X] = 2.0f; runtime_mm[AXIS_X] = 0.0f;
}
int main(void) {
    /* an arc accepted with its first segment refused (planner headroom), or queued with exec deferred */
    accepted_not_started();
    assert(report(cm_get_pos, AXIS_X) == 0.0f && report(cm_get_mpo, AXIS_X) == 0.0f);
    /* a paused program (M0 holds): the runtime, as before */
    cm.motion_state = MOTION_HOLD; cm.am = RUNTIME;
    assert(report(cm_get_pos, AXIS_X) == 0.0f);
    /* running: the runtime */
    cm.motion_state = MOTION_RUN; runtime_mm[AXIS_X] = 0.7f;
    assert(fabsf(report(cm_get_pos, AXIS_X) - 0.7f) < 1e-6f);
    /* idle, no cycle: the model, which may differ from the runtime after G92 or an offset change */
    cm.cycle_state = CYCLE_OFF; cm.motion_state = MOTION_STOP; cm.am = MODEL;
    cm.gmx.position[AXIS_X] = 5.0f; runtime_mm[AXIS_X] = 0.0f;
    assert(report(cm_get_pos, AXIS_X) == 5.0f && report(cm_get_mpo, AXIS_X) == 5.0f);
    /* a queued G20 has not run yet: the runtime position, in the units the unit report shows */
    accepted_not_started(); runtime_mm[AXIS_X] = 25.4f;
    cm.gm.units_mode = INCHES; mr.gm.units_mode = MILLIMETERS;
    assert(fabsf(report(cm_get_pos, AXIS_X) - 1.0f) < 1e-5f);
    cm.gm.units_mode = MILLIMETERS;
    /* the work offset comes from the same model as the position */
    accepted_not_started(); cm.gm.work_offset[AXIS_X] = 9.0f; mr.gm.work_offset[AXIS_X] = 1.0f;
    assert(report(cm_get_ofs, AXIS_X) == 1.0f);
    return 0;
}
'''


class PositionReportModel(unittest.TestCase):
    def test_reports_ignore_accepted_but_unstarted_motion(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        source = (ROOT / "firmware/tinyg/canonical_machine.c").read_text()
        code = PRELUDE
        for signature in (r"float cm_get_work_offset\(", r"float cm_get_absolute_position\(",
                          r"float cm_get_work_position\(", r"static GCodeState_t \*_position_model\(",
                          r"stat_t cm_get_pos\(", r"stat_t cm_get_mpo\(", r"stat_t cm_get_ofs\("):
            code += function(source, signature + r"[^;{}]*\)(?=\s*\{)") + "\n"
        code += EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-e090-") as tmp:
            c, exe = Path(tmp) / "pos.c", Path(tmp) / "pos"
            c.write_text(code)
            subprocess.run([cc, "-std=c99", "-Wall", "-Wno-unused-function", "-fsanitize=address,undefined",
                            str(c), "-o", str(exe), "-lm"], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
