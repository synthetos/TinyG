"""E112: setters check the whole value before narrowing it to a byte, and index-like settings have a range.

The real set_ui8() (config.c), set_gco(), set_ma(), set_baud() (config_app.c) and st_set_md() (stepper.c)
run against a one-row table whose target is a byte between two guard bytes. Out-of-range values must be
refused and leave the target (and its neighbours) untouched; 256 must never act as 0, nor 257 as 1.
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
#include <stdio.h>
#include <string.h>
#define STAT_OK 0
#define STAT_INPUT_VALUE_RANGE_ERROR 109
#define AXES 6
#define MOTORS 4
#define NV_MESSAGE_LEN 64
#define PSTR(s) (s)
#define sprintf_P sprintf
enum { ABSOLUTE_COORDS = 0, G54, G55, G56, G57, G58, G59 };
enum { TYPE_NULL = 0, TYPE_INTEGER = 2 };
typedef int stat_t;
typedef struct { int index, valuetype; uint32_t value_int; } nvObj_t;
static struct { uint8_t lo, target, hi; } cell = {0xA5, 0, 0x5A};
static struct { void *target; } cfgArray[1] = {{&cell.target}};
#define GET_TABLE_WORD(a) cfgArray[nv->index].a
static const char *msg_baud[] = {"0", "9600", "19200", "38400", "57600", "115200", "230400"};
#define GET_TEXT_ITEM(b,a) b[a]
struct { uint8_t usb_baud_rate; bool usb_baud_flag; } xio;
static int deenergized = -1, all_off;
void nv_add_message_P(const char *m) { (void)m; }
void nv_add_message(const char *m) { (void)m; }
void st_deenergize_motors(void) { all_off++; }
static void _deenergize_motor(uint8_t motor) { assert(motor < MOTORS); deenergized = motor; }
'''

EXERCISE = r'''
static stat_t call(stat_t (*set)(nvObj_t *), uint32_t v) { nvObj_t nv = {0, TYPE_INTEGER, v}; return set(&nv); }
static void guards(void) { assert(cell.lo == 0xA5 && cell.hi == 0x5A); }
int main(void) {
    /* gco: G54..G59 only; 7 and 256 (which narrows to 0) refused */
    cell.target = G54;
    assert(call(set_gco, 7) == STAT_INPUT_VALUE_RANGE_ERROR && cell.target == G54);
    assert(call(set_gco, 256) == STAT_INPUT_VALUE_RANGE_ERROR && cell.target == G54);
    assert(call(set_gco, 0) == STAT_INPUT_VALUE_RANGE_ERROR && cell.target == G54);
    assert(call(set_gco, G59) == STAT_OK && cell.target == G59); guards();
    /* ma: an axis index 0..AXES-1 */
    cell.target = 0;
    assert(call(set_ma, AXES) == STAT_INPUT_VALUE_RANGE_ERROR && call(set_ma, 256 + 2) == STAT_INPUT_VALUE_RANGE_ERROR);
    assert(cell.target == 0 && call(set_ma, 5) == STAT_OK && cell.target == 5); guards();
    /* baud: 257 is not 1 (9600) */
    xio.usb_baud_rate = 5; xio.usb_baud_flag = false;
    assert(call(set_baud, 257) == STAT_INPUT_VALUE_RANGE_ERROR && xio.usb_baud_rate == 5 && !xio.usb_baud_flag);
    assert(call(set_baud, 6) == STAT_OK && xio.usb_baud_rate == 6 && xio.usb_baud_flag);
    /* md: 256 is not "all motors off" or motor index 255 */
    assert(call(st_set_md, 256) == STAT_INPUT_VALUE_RANGE_ERROR && deenergized == -1 && !all_off);
    assert(call(st_set_md, 2) == STAT_OK && deenergized == 1);
    assert(call(st_set_md, 0) == STAT_OK && all_off == 1);
    return 0;
}
'''


class SetterRanges(unittest.TestCase):
    def test_setters_reject_out_of_range_before_narrowing(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        fw = ROOT / "firmware/tinyg"
        config, app, stepper = ((fw / f).read_text() for f in ("config.c", "config_app.c", "stepper.c"))
        code = PRELUDE
        for source, signature in ((config, r"stat_t set_ui8\("), (app, r"static stat_t set_gco\("),
                                  (app, r"static stat_t set_ma\("), (app, r"static stat_t set_baud\("),
                                  (stepper, r"stat_t st_set_md\(")):
            code += function(source, signature + r"[^;{}]*\)(?=[^\n{;]*\n?\s*\{)") + "\n"
        code += EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-e112-") as tmp:
            c, exe = Path(tmp) / "set.c", Path(tmp) / "set"
            c.write_text(code)
            subprocess.run([cc, "-std=c99", "-Wall", "-Wno-unused-function", "-Wno-deprecated-declarations",
                            "-fsanitize=address,undefined", str(c), "-o", str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
