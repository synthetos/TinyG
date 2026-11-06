"""Queue reports print qi/qo over their whole uint16 range, unsigned, without overrunning the buffer.

qr_queue_report_callback() is extracted from firmware/tinyg/report.c and compiled with ASan/UBSan.
qi/qo are uint16_t and accumulate until reported, so they can reach five digits (E091 keeps them
across cycles); the JSON report used to go through a 32-byte buffer with %d.
"""
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def function(source, name):
    match = re.search(r"^stat_t " + name + r"\([^)]*\)[^{;]*\{", source, re.M)   # a trailing comment is allowed
    if not match:
        raise AssertionError(f"firmware does not implement {name}")
    depth, end = 1, match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


PRELUDE = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define PSTR(s) (s)
#define printf_P printf
#define sprintf_P sprintf
#define snprintf_P snprintf
#define TEXT_MODE 0
#define JSON_MODE 1
#define QR_OFF 0
#define QR_SINGLE 1
#define QR_TRIPLE 2
#define STAT_OK 0
#define STAT_NOOP 60
typedef int stat_t;
struct { uint8_t queue_report_verbosity, queue_report_requested, buffers_available;
         uint16_t buffers_added, buffers_removed; } qr;
struct { uint8_t comm_mode; } cs;
static char *json_relax(char *s) { return s; }
static void qr_init_queue_report(void) { qr.queue_report_requested = 0; qr.buffers_added = qr.buffers_removed = 0; }
'''

EXERCISE = r'''
int main(void) {
    static const uint16_t counts[] = {0, 9999, 10000, 32767, 32768, 65535};
    for (unsigned mode = 0; mode < 2; mode++) {
        for (unsigned i = 0; i < sizeof(counts)/sizeof(counts[0]); i++) {
            cs.comm_mode = mode ? JSON_MODE : TEXT_MODE;
            qr.queue_report_verbosity = QR_TRIPLE; qr.queue_report_requested = 1;
            qr.buffers_available = 255; qr.buffers_added = counts[i]; qr.buffers_removed = counts[i];
            if (qr_queue_report_callback() != STAT_OK) return 1;
        }
    }
    return 0;
}
'''


class QueueReportFormat(unittest.TestCase):
    def test_counts_print_unsigned_and_whole(self):
        compiler = shutil.which("cc")
        if not compiler:
            self.skipTest("host C compiler not installed")
        source = (ROOT / "firmware/tinyg/report.c").read_text()
        code = PRELUDE + function(source, "qr_queue_report_callback") + EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-qr-") as tmp:
            c, exe = Path(tmp) / "qr.c", Path(tmp) / "qr"
            c.write_text(code)
            subprocess.run([compiler, "-std=c99", "-Wno-format-security", "-fsanitize=address,undefined","-fstrict-flex-arrays=3",
                            str(c), "-o", str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        expected = [f"qr:255, qi:{n}, qo:{n}" for n in (0, 9999, 10000, 32767, 32768, 65535)]
        expected += [f'{{"qr":255,"qi":{n},"qo":{n}}}' for n in (0, 9999, 10000, 32767, 32768, 65535)]
        self.assertEqual(result.stdout.splitlines(), expected)


if __name__ == "__main__":
    unittest.main()
