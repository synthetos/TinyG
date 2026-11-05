"""E183: a queue report must not lose a buffer removal that lands while it is written.

report.c qr_queue_report_callback() formats qi/qo and then zeroes the counters (qr_init_queue_report).
A buffer freed by the exec in between is counted into buffers_removed and then wiped: it is never
reported, so qi and qo stop balancing. The real queue-report functions are extracted; the exec's free
runs right after the report is formatted, deferred while the main loop has interrupts off.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = Path(os.environ.get('TINYG_FIRMWARE', ROOT / 'firmware/tinyg'))
DEF = r"[^;{}]*\)(?=\s*(?://[^\n]*\s*)?\{)"

QR_PRELUDE = r'''
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#define __AVR                                  /* as tinyg.h defines it */
uint8_t SREG = 0x80;
#define cli() (SREG &= (uint8_t)~128)
#define PSTR(s) (s)
#define printf_P printf
#define TEXT_MODE 0
#define QR_OFF 0
#define QR_SINGLE 1
#define MOTION_MODE_CW_ARC 2
#define MOTION_MODE_CCW_ARC 3
#define MIN_ARC_QR_INTERVAL 200
#define ACTIVE_MODEL 0
#define STAT_OK 0
#define STAT_NOOP 3
typedef int stat_t;
typedef struct { uint8_t queue_report_verbosity, queue_report_requested, buffers_available, prev_available;
                 uint16_t buffers_added, buffers_removed; uint8_t motion_mode; uint32_t init_tick; } qrSingleton_t;
qrSingleton_t qr;
struct { int comm_mode; } cs = {1};
static uint8_t available = 20;
static uint8_t mp_get_planner_buffers_available(void) { return available; }
static uint8_t cm_get_motion_mode(int m) { (void)m; return 1; }
static uint32_t SysTickTimer_getValue(void) { return 1000; }
static const char *json_relax(const char *s) { return s; }
void qr_request_queue_report(int8_t buffers);
static unsigned reported_removed, frees;
static int fire_in_report;
static void exec_frees_a_buffer(void) { frees++; available++; qr_request_queue_report(-1); }
static int snprintf_P(char *buf, size_t n, const char *fmt, ...);
#include <stdarg.h>
static int snprintf_P(char *buf, size_t n, const char *fmt, ...) {
    va_list ap; va_start(ap, fmt); int r = vsnprintf(buf, n, fmt, ap); va_end(ap);
    unsigned qr_, qi, qo;
    if (sscanf(buf, "{\"qr\":%u,\"qi\":%u,\"qo\":%u}", &qr_, &qi, &qo) == 3) reported_removed += qo;
    if (fire_in_report && (SREG & 0x80)) { fire_in_report = 0; exec_frees_a_buffer(); }  /* report formatted, counters not yet cleared */
    return r;
}
static int fputs_stub(const char *s, FILE *f) { (void)s; (void)f; return 0; }
#define fputs fputs_stub
'''

QR_MAIN = r'''
int main(void) {
    qr.queue_report_verbosity = 2;                      /* QR_TRIPLE: qr, qi, qo */
    qr_init_queue_report();
    for (int i = 0; i < 3; i++) exec_frees_a_buffer();  /* three blocks done before the report */
    fire_in_report = 1;
    assert(qr_queue_report_callback() == STAT_OK);      /* a fourth is freed while it is written */
    if (fire_in_report) { fire_in_report = 0; exec_frees_a_buffer(); }   /* masked: runs after */
    while (qr.queue_report_requested) qr_queue_report_callback();
    assert(reported_removed + qr.buffers_removed == frees);   /* stock: 3 reported, 0 left, 4 freed */
    return 0;
}
'''


def _cc():
    return shutil.which('cc')


class QueueReportSnapshot(unittest.TestCase):
    def test_e183_queue_report_loses_a_removal(self):
        if not _cc():
            self.skipTest('host C compiler not installed')
        report = (FIRMWARE / 'report.c').read_text()
        code = QR_PRELUDE
        for sig in (r'void qr_init_queue_report\(', r'void qr_request_queue_report\(',
                    r'stat_t qr_queue_report_callback\('):
            code += function(report, sig + DEF) + '\n'
        code += QR_MAIN
        with tempfile.TemporaryDirectory() as d:
            c, exe = Path(d) / 'qr.c', Path(d) / 'qr'
            c.write_text(code)
            subprocess.run([_cc(), '-std=gnu99', '-Wall', '-Wno-unused-function', '-fsanitize=address,undefined','-fstrict-flex-arrays=3',
                            str(c), '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
