"""E101: the probe completion report is a top-level {"prb":{...}} object with JSON-safe numbers.

The real _probe_report() is extracted from cycle_probing.c and run on ordinary coordinates, NaN and
both infinities: every line must be strict JSON, with the prb group's keys and null for non-finite
values (as the serializer's prep_float makes them).
"""
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]

PRELUDE = r'''
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define AXES 6
#define PSTR(s) (s)
#define sprintf_P sprintf
struct { int probe_state; float probe_results[AXES]; } cm;
struct { char out_buf[512]; } cs;
char *json_relax(char *s) { return s; }
'''

MAIN = r'''
int main(void) {
    float cases[][AXES] = {{1.5f, -2.25f, -1.0f, 0, 0, 90.125f},
                           {NAN, 0, -1.0f, 0, 0, 0},
                           {INFINITY, -INFINITY, 0.001f, 0, 0, 0}};
    for (int c = 0; c < 3; c++) {
        cm.probe_state = c == 0;
        memcpy(cm.probe_results, cases[c], sizeof(cases[c]));
        _probe_report();
    }
    return 0;
}
'''


class ProbeReport(unittest.TestCase):
    def test_report_is_strict_json_with_null_for_non_finite(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        source = (ROOT / "firmware/tinyg/cycle_probing.c").read_text()
        code = PRELUDE + function(source, r"static void _probe_report\(\)(?=\s*\{)") + "\n" + MAIN
        with tempfile.TemporaryDirectory(prefix="tinyg-e101-") as tmp:
            c, exe = Path(tmp) / "prb.c", Path(tmp) / "prb"
            c.write_text(code)
            subprocess.run([cc, "-std=c99", "-Wall", "-Wno-deprecated-declarations", "-fsanitize=address,undefined",
                            str(c), "-o", str(exe)], check=True)
            out = subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout
        reports = [json.loads(line) for line in out.splitlines()]      # strict: bare nan/inf would raise
        self.assertEqual(reports[0], {"prb": {"e": 1, "x": 1.5, "y": -2.25, "z": -1.0, "a": 0.0, "b": 0.0, "c": 90.125}})
        self.assertEqual(reports[1]["prb"]["x"], None)
        self.assertEqual(reports[1]["prb"]["z"], -1.0)
        self.assertEqual((reports[2]["prb"]["x"], reports[2]["prb"]["y"], reports[2]["prb"]["z"]), (None, None, 0.001))


if __name__ == "__main__":
    unittest.main()
