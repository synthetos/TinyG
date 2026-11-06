"""Spindle and coolant state readouts, named as in g2core: spe, spd, sps (spindle.c), cof, com (canonical_machine.c).

The readouts report the outputs as last driven by the real executors, not the model, which runs ahead of
the outputs: M2/M30 turns the model off when it is parsed, while the outputs stay on until the queued
program end runs. Cases, with the real _exec_spindle_control(), _exec_spindle_speed(), spindle_reset(),
_exec_flood/mist_coolant_control() and cm_coolant_off_immediate():
  - M3 S, M4 S, M5, S while running and while off, M7/M8/M9 and an alarm's immediate off;
  - the model turned off as M30's parse does, outputs still on: the readouts still say on, until the
    program end's spindle_reset() and cm_coolant_off_immediate() run;
  - spd keeps the direction last driven while the spindle is off; sps is 0 while off;
  - a speed read interrupted halfway by the loader's S update returns one of the two speeds, never a mix.
The rows are read-only (set_nul) and never initialized or persisted.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FW = Path(os.environ.get("TINYG_FIRMWARE", ROOT / "firmware/tinyg"))
DEF = r"[^;{}]*\)(?=\s*(?://[^\n]*\s*)?\{)"

PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define __AVR
typedef uint8_t stat_t;
#define STAT_OK 0
#define AXES 6
#define FLAGS_NONE ((bool *)0)
enum { TYPE_INTEGER = 2, TYPE_FLOAT = 4 };
enum { SPINDLE_OFF = 0, SPINDLE_CW, SPINDLE_CCW };
enum { MODEL = 1 };
enum { SPINDLE_BIT = 1, SPINDLE_DIR = 2, MIST_COOLANT_BIT = 4, FLOOD_COOLANT_BIT = 8 };
typedef struct { uint8_t valuetype; union { float value_flt; uint32_t value_int; }; } nvObj_t;
struct { struct { uint8_t spindle_mode, mist_coolant, flood_coolant; float spindle_speed; } gm; int alarm; } cm;
struct { uint8_t enable_polarity, direction_polarity; } spindle_cfg = { 1, 1 };
uint8_t SREG = 0x80;
static void cli(void) { SREG &= ~0x80; }
static uint8_t pins;
static void gpio_set_bit_on(uint8_t b) { pins |= b; }
static void gpio_set_bit_off(uint8_t b) { pins &= ~b; }
static void pwm_set_duty(int ch, float d) { (void)ch; (void)d; }
#define PWM_1 0
static bool cm_outputs_inhibited(void) { return cm.alarm; }
static void cm_set_spindle_mode(int m, uint8_t mode) { (void)m; cm.gm.spindle_mode = mode; }
static void cm_set_spindle_speed_parameter(int m, float s) { (void)m; cm.gm.spindle_speed = s; }
static float cm_get_spindle_pwm(uint8_t mode) {                      /* the real one clamps while running */
    if (mode != SPINDLE_OFF) { if (cm.gm.spindle_speed > 24000) cm.gm.spindle_speed = 24000; }
    return 0.5f;
}
static int inject;
static void loader_s_update(void);
'''


def src(path):
    return (FW / path).read_text(encoding="latin-1")


def fn(path, name, ret=r"(?:static )?(?:void|stat_t)"):
    return function(src(path), r"(?m)^" + ret + r"\s+" + name + r"\(" + DEF)   # not the commented-out prototypes


def code(split_read=False):
    sp, cmc = src("spindle.c"), src("canonical_machine.c")
    c = PRELUDE
    c += sp[sp.index("static struct {"):sp.index("} spindle_out;") + len("} spindle_out;")] + "\n"
    for name in ("_record_speed", "_get_state", "_exec_spindle_control", "_exec_spindle_speed", "spindle_reset"):
        c += fn("spindle.c", name) + "\n"
    for line in re.findall(r"^stat_t spindle_get_sp[edc]\(nvObj_t \*nv\) \{[^\n]*\}$", sp, re.M):
        c += line + "\n"
    sps = fn("spindle.c", "spindle_get_sps")
    if split_read:                                                    # the AVR loads the float a byte at a time
        assert sps.count("float speed = spindle_out.speed;") == 1
        sps = sps.replace("float speed = spindle_out.speed;",
                          "float speed; unsigned char *d = (unsigned char *)&speed, *s = (unsigned char *)&spindle_out.speed;"
                          " d[0] = s[0]; d[1] = s[1]; if (inject) { inject = 0; if (SREG & 0x80) loader_s_update(); else inject = 2; }"
                          " d[2] = s[2]; d[3] = s[3];")
    c += sps + "\n"
    c += "static void _exec_flood_coolant_control(float *value, bool *flags);\n"
    c += re.search(r"static volatile uint8_t coolant_out_mist, coolant_out_flood;\n", cmc).group(0)
    for line in re.findall(r"^stat_t coolant_get_co[fm]\(nvObj_t \*nv\) \{[^\n]*\}$", cmc, re.M):
        c += line + "\n"
    for name in ("_exec_mist_coolant_control", "_exec_flood_coolant_control", "cm_coolant_off_immediate"):
        c += fn("canonical_machine.c", name) + "\n"
    return c


MAIN = r'''
static uint32_t i(stat_t (*f)(nvObj_t *)) { nvObj_t nv = {0}; assert(f(&nv) == STAT_OK && nv.valuetype == TYPE_INTEGER); return nv.value_int; }
static float sps(void) { nvObj_t nv = {0}; assert(spindle_get_sps(&nv) == STAT_OK && nv.valuetype == TYPE_FLOAT); return nv.value_flt; }
static void exec(void (*f)(float *, bool *), float v) { float value[AXES] = { v }; f(value, FLAGS_NONE); }
static void loader_s_update(void) { exec(_exec_spindle_speed, 20000.25f); }
int main(void) {
    spindle_reset();
    assert(i(spindle_get_spe) == 0 && sps() == 0);
    exec(_exec_spindle_speed, 5000);                    /* S while off: kept, not in use */
    assert(i(spindle_get_spe) == 0 && sps() == 0);
    exec(_exec_spindle_control, SPINDLE_CW);           /* M3 */
    assert(i(spindle_get_spe) == 1 && i(spindle_get_spc) == 1 && i(spindle_get_spd) == 0 && sps() == 5000);
    exec(_exec_spindle_speed, 30000);                   /* S while running, clamped */
    assert(sps() == 24000);
    exec(_exec_spindle_control, SPINDLE_CCW);          /* M4 */
    assert(i(spindle_get_spe) == 1 && i(spindle_get_spc) == 2 && i(spindle_get_spd) == 1);
    exec(_exec_spindle_control, SPINDLE_OFF);          /* M5: the direction last driven stays */
    assert(i(spindle_get_spe) == 0 && i(spindle_get_spc) == 0 && i(spindle_get_spd) == 1 && sps() == 0);

    exec(_exec_flood_coolant_control, 1); exec(_exec_mist_coolant_control, 1);   /* M8, M7 */
    assert(i(coolant_get_cof) == 1 && i(coolant_get_com) == 1);
    exec(_exec_flood_coolant_control, 0);              /* M9: both off */
    assert(i(coolant_get_cof) == 0 && i(coolant_get_com) == 0);

    /* M30 parsed behind a move: the model goes off at once, the outputs only when the program end runs */
    exec(_exec_spindle_control, SPINDLE_CW); exec(_exec_flood_coolant_control, 1); exec(_exec_mist_coolant_control, 1);
    cm.gm.spindle_mode = SPINDLE_OFF; cm.gm.mist_coolant = false; cm.gm.flood_coolant = false;   /* cm_program_end() */
    assert(i(spindle_get_spe) == 1 && i(coolant_get_cof) == 1 && i(coolant_get_com) == 1 && sps() > 0);
    spindle_reset(); cm_coolant_off_immediate();        /* _exec_program_end() */
    assert(i(spindle_get_spe) == 0 && i(coolant_get_cof) == 0 && i(coolant_get_com) == 0 && sps() == 0);

    /* an alarm turns the outputs off at once, and a queued M3 that runs after it stays off */
    exec(_exec_spindle_control, SPINDLE_CW); cm.alarm = 1; spindle_reset(); exec(_exec_spindle_control, SPINDLE_CW);
    assert(i(spindle_get_spe) == 0);
    cm.alarm = 0;
    return 0;
}
'''

TORN = r'''
static void exec(void (*f)(float *, bool *), float v) { float value[AXES] = { v }; f(value, FLAGS_NONE); }
static void loader_s_update(void) { exec(_exec_spindle_speed, 20000.25f); }
int main(void) {
    spindle_reset(); exec(_exec_spindle_speed, 10000.5f); exec(_exec_spindle_control, SPINDLE_CW);
    inject = 1;
    nvObj_t nv = {0}; spindle_get_sps(&nv);
    if (inject == 2) { inject = 0; loader_s_update(); }   /* a masked interrupt runs on SREG restore */
    assert(nv.value_flt == 10000.5f || nv.value_flt == 20000.25f);
    return 0;
}
'''


class SpindleCoolantStatus(unittest.TestCase):
    def build_run(self, c, name):
        with tempfile.TemporaryDirectory() as tmp:
            path, exe = Path(tmp) / (name + ".c"), Path(tmp) / name
            path.write_text(c)
            subprocess.run(["cc", "-std=gnu99", "-O1", "-Wall", "-Werror", "-Wno-unused-function", "-Wno-unused-variable",
                            "-fsanitize=address,undefined","-fstrict-flex-arrays=3", str(path), "-o", str(exe)], check=True)
            r = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_readouts_follow_the_outputs(self):
        self.build_run(code() + MAIN, "sp")

    def test_speed_read_is_whole(self):
        self.build_run(code(split_read=True) + TORN, "torn")

    def test_rows_read_only_and_not_persisted(self):
        table = src("config_app.c")
        for key, getter in (("spe", "spindle_get_spe"), ("spc", "spindle_get_spc"), ("spd", "spindle_get_spd"), ("sps", "spindle_get_sps"),
                            ("cof", "coolant_get_cof"), ("com", "coolant_get_com")):
            row = re.search(r'\{\s*""\s*,\s*"' + key + r'"\s*,\s*(_\w+)\s*,[^,]*,[^,]*,\s*(\w+)\s*,\s*(\w+)\s*,', table)
            self.assertIsNotNone(row, key)
            self.assertIn(row.group(1), ("_i0", "_f0"), key)            # no F_INITIALIZE or F_PERSIST
            self.assertEqual((row.group(2), row.group(3)), (getter, "set_nul"), key)


if __name__ == "__main__":
    unittest.main()
