"""E161: the normaliser drops only whitespace and '%' and never splices digits across other characters.

The real _normalize_gcode_block() is extracted from gcode_parser.c.
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
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define NUL 0
'''

EXERCISE = r'''
static void check(const char *in, const char *want, const char *want_com) {
    char buf[80], none = NUL, *com = &none, *msg = &none; uint8_t del;
    strcpy(buf, in);
    _normalize_gcode_block(buf, &com, &msg, &del);
    if (strcmp(buf, want) || strcmp(com, want_com)) { fprintf(stderr, "%s -> [%s] [%s]\n", in, buf, com); assert(0); }
}
int main(void) {
    check("  g1 x100 Y100 f400", "G1X100Y100F400", "");
    check("g1\tx1 \r", "G1X1", "");
    check("G1 X+1.5", "G1X+1.5", "");
    check("%", "", "");
    check("  % ", "", "");
    check("%(start)", "", "start");
    check("G1 X0.001%9 F100", "G1X0.001%9F100", "");     /* embedded %: kept for rejection */
    check("%G1X1", "%G1X1", "");
    check("G1 X1 (a * # comment)", "G1X1", "a * # comment");
    check("G1 X1 ; note", "G1X1", " note");
    check("/G1 X1", "/G1X1", "");
    check("G1 X0.001*9 F100", "G1X0.001*9F100", "");      /* not X0.0019 */
    check("G1 X10 Y5*45", "G1X10Y5*45", "");
    check("G1 X0.0#01", "G1X0.0#01", "");
    check("G1 X[1]", "G1X[1]", "");
    return 0;
}
'''


class NormalizeBlock(unittest.TestCase):
    def test_no_digit_splicing(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        source = (ROOT / "firmware/tinyg/gcode_parser.c").read_text()
        code = PRELUDE + function(source, r"static void _normalize_gcode_block\([^;{}]*\)(?=\s*\{)") + "\n" + EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-e161-") as tmp:
            c, exe = Path(tmp) / "norm.c", Path(tmp) / "norm"
            c.write_text(code)
            subprocess.run([cc, "-std=gnu99", "-Wall", "-fsanitize=address,undefined", str(c), "-o", str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
