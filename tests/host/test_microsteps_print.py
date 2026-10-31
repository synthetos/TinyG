"""E111: the text-mode microsteps report prints the whole uint32_t, so $1mi shows 256, not 0.

The real fmt_0mi, _print_motor_u32() and st_print_mi() are extracted from stepper.c. On the AVR
uint32_t is unsigned long, which is what %lu expects; the host build maps it the same way.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]

PRELUDE = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define uint32_t unsigned long          /* as on the AVR */
#define PROGMEM
#define sprintf_P sprintf
#define NV_MESSAGE_LEN 80
typedef struct { char group[4], token[6]; uint32_t value_int; } nvObj_t;
static char printed[NV_MESSAGE_LEN];
void text_finalize_message(char *msg) { strcpy(printed, msg); }
'''


class MicrostepsPrint(unittest.TestCase):
    def test_text_report_prints_256(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        source = (ROOT / "firmware/tinyg/stepper.c").read_text()
        fmt = re.search(r"static const char fmt_0mi\[\] PROGMEM = [^\n]*", source).group(0)
        code = PRELUDE + fmt + "\n"
        code += function(source, r"static void _print_motor_u32\([^;{}]*\)(?=[^\n{;]*\n?\s*\{)") + "\n"
        code += function(source, r"void st_print_mi\(nvObj_t \*nv\)") + "\n"
        code += r'''
int main(void) {
    nvObj_t nv = {"1", "mi", 256};
    st_print_mi(&nv);
    assert(strstr(printed, " 256 [1,2,4,8]") != NULL);
    nv.value_int = 8; st_print_mi(&nv);
    assert(strstr(printed, " 8 [1,2,4,8]") != NULL);
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix="tinyg-e111-") as tmp:
            c, exe = Path(tmp) / "mi.c", Path(tmp) / "mi"
            c.write_text(code)
            subprocess.run([cc, "-std=c99", "-Wall", "-Werror", "-Wno-deprecated-declarations", "-fsanitize=address,undefined", str(c), "-o", str(exe)],
                           check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
