"""Compile the firmware query functions with deterministic model/report fixtures.

No serial port or hardware is accessed. Set TINYG_SOURCE to test another checkout.
"""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

from source_extract import region

ROOT = Path(os.environ.get("TINYG_SOURCE", Path(__file__).resolve().parents[2]))


def function(source, name):
    match = re.search(r"^(?:static )?(?:stat_t|float) " + name + r"\([^;]*?\)\s*\{", source, re.M)
    if not match:
        raise AssertionError(f"firmware does not implement {name}")
    depth = 1
    end = match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


class PositionQueries(unittest.TestCase):
    def test_reports_use_work_position_and_units(self):
        source = (ROOT / "firmware/tinyg/canonical_machine.c").read_text()
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#define AXES 6
#define AXIS_X 0
#define AXIS_Z 2
#define INCHES 1
#define MM_PER_INCH 25.4f
#define NUL 0
#define TEXT_MODE 0
#define STAT_OK 0
#define STAT_JSON_TOO_MANY_PAIRS 112
#define PSTR(x) (x)
#define pgm_read_byte(x) (*(x))
#define strcpy_P strcpy
#define fprintf_P fprintf
typedef int stat_t;
typedef struct { int units_mode; float target[6], work_offset[6]; } GCodeState_t;
#define MOTION_STOP 0
#define CYCLE_OFF 0
struct { int motion_state, cycle_state; GCodeState_t gm, *am; struct {float position[6], tool_offset[6];} gmx; } cm;
struct { GCodeState_t gm; } mr;
struct { int comm_mode; float fw_build, fw_version; } cs;
#define MODEL (&cm.gm)
#define RUNTIME (&mr.gm)
#define ACTIVE_MODEL cm.am
float offsets[6], runtime_pos[6];
float cm_get_active_coord_offset(uint8_t axis) {return offsets[axis];}
float mp_get_runtime_work_position(uint8_t axis) {return runtime_pos[axis];}
static int cm_get_units_mode(const GCodeState_t *gcode_state) {return gcode_state->units_mode;}
float mp_get_runtime_absolute_position(uint8_t axis) {return runtime_pos[axis];}
float mp_get_runtime_work_offset(uint8_t axis) {(void)axis; return 0;}
typedef struct {char token[6], group[4]; int depth, precision; float value_flt;} nvObj_t;
nvObj_t objects[6]; int used, capacity=6;
nvObj_t *nv_add_object(const char *token) {
    if (used == capacity) return NULL;
    nvObj_t *nv=&objects[used++];
    strcpy(nv->token, token); strcpy(nv->group, "pos"); nv->depth=0;
    if (!strcmp(token,"fb")) nv->value_flt=cs.fw_build;
    else if (!strcmp(token,"fv")) nv->value_flt=cs.fw_version;
    return nv;
}
'''
        code += '\n'.join(function(source, name) for name in ('cm_get_work_offset', 'cm_get_absolute_position',
                                                              '_get_work_position_mm', 'cm_get_work_position'))
        code += '\n' + region(source, r"static GCodeState_t \*_position_model\(\)",
                              r"static float _get_report_position\([^;{}]*\)(?=\s*\{)") + '\n'
        code += '\n'.join(function(source, name) for name in ('cm_get_position', 'cm_get_firmware'))
        code += r'''
int main(void) {
    cm.am=MODEL; cs.comm_mode=1; cs.fw_build=449.01f; cs.fw_version=.97f;
    cm.gmx.position[0]=34.4f; offsets[0]=10; cm.gmx.tool_offset[0]=1;
    cm.gmx.position[3]=180; cm.gm.target[0]=999;
    assert(cm_get_position()==0 && used==6);
    assert(fabsf(objects[0].value_flt-23.4f)<.0001f);   /* 34.4 - 10 - 1: G43.1 subtracts (E037) */
    assert(objects[3].value_flt==180);
    for (int i=0;i<6;i++) {
        assert(objects[i].depth==1 && objects[i].precision==4 && objects[i].group[0]==0);
    }
    assert(!strcmp(objects[0].token,"posx") && !strcmp(objects[5].token,"posc"));
    cm.gm.units_mode=INCHES; used=0;
    assert(cm_get_position()==0 && fabsf(objects[0].value_flt-23.4f/25.4f)<.0001f);
    assert(objects[3].value_flt==180);
    GCodeState_t runtime={.units_mode=INCHES}; cm.am=&runtime;
    runtime_pos[0]=50.8f; runtime_pos[3]=270; used=0;
    assert(cm_get_position()==0 && fabsf(objects[0].value_flt-2)<.0001f);
    assert(objects[3].value_flt==270);
    used=0; capacity=2;
    assert(cm_get_position()==STAT_JSON_TOO_MANY_PAIRS && used==2);
    used=0;
    assert(cm_get_firmware()==0 && used==2);
    assert(!strcmp(objects[0].token,"fb") && objects[0].value_flt==cs.fw_build);
    assert(!strcmp(objects[1].token,"fv") && objects[1].value_flt==cs.fw_version);
    used=0; capacity=1;
    assert(cm_get_firmware()==STAT_JSON_TOO_MANY_PAIRS && used==1);
    cs.comm_mode=TEXT_MODE; used=0;
    assert(cm_get_position()==0 && used==0);
    assert(cm_get_firmware()==0 && used==0);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / 'queries.c'
            c.write_text(code)
            exe = Path(tmp) / 'queries'
            subprocess.run(['cc', '-std=c99', '-Wall', '-Werror', '-Wno-unused-function', '-fsanitize=address,undefined','-fstrict-flex-arrays=3',
                            str(c), '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
            self.assertEqual(result.stderr.splitlines()[0],
                             'X:2.0000 Y:0.0000 Z:0.0000 A:270.0000 B:0.0000 C:0.0000')
            self.assertIn('FIRMWARE_NAME:TinyG, FIRMWARE_VERSION:0.97, FIRMWARE_BUILD:449.01', result.stderr)


if __name__ == '__main__':
    unittest.main()
