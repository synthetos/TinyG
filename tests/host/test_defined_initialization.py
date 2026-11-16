"""Strict red/green checks for C initialization defects found by the audit.

The tests use actual function bodies. Host sanitizers diagnose C contracts;
they do not assert that an AVR faults on a null data-space access.
"""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

from source_extract import function
import test_linemode_allocator as allocator

FW = Path(__file__).resolve().parents[2] / "firmware/tinyg"
HEADERS = Path(__file__).parent / "planner_startup"


class DefinedInitialization(unittest.TestCase):
    def compile_and_run(self, source, extra=()):
        with tempfile.TemporaryDirectory(dir=os.environ.get("TMPDIR")) as directory:
            path = Path(directory) / "check.c"
            path.write_text(source)
            exe = path.with_suffix("")
            built = subprocess.run([
                "cc", "-std=gnu99", "-O1", "-g", "-D_POSIX_C_SOURCE=200809L",
                "-fsanitize=undefined,address", "-fno-sanitize-recover=all",
                "-I" + str(HEADERS), "-I" + str(FW), *extra,
                str(path), "-lm", "-o", str(exe)
            ], capture_output=True, text=True)
            self.assertEqual(built.returncode, 0, built.stderr)
            ran = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(ran.returncode, 0, ran.stderr)

    @unittest.expectedFailure
    def test_null_parent_initializes_an_unparented_nv_list(self):
        code = r'''
#include <assert.h>
#include <string.h>
#include "tinyg.h"
#include "config.h"
#define NUL 0
nvStr_t nvStr;
nvList_t nvl;
'''
        code += function((FW / "config.c").read_text(), r'nvObj_t \*nv_reset_nv_list\(')
        code += r'''
int main(void) {
    char *parents[]={NULL,"","r"};
    for(unsigned p=0;p<3;p++) {
        memset(&nvl,0xa5,sizeof nvl);
        assert(nv_reset_nv_list(parents[p])==NV_BODY);
        assert(nvl.list[0].pv==NULL && nvl.list[NV_LIST_LEN-1].nx==NULL);
        for(unsigned i=0;i<NV_LIST_LEN;i++) {
            assert(nvl.list[i].nv_num==i);
            if(i) assert(nvl.list[i].pv==&nvl.list[i-1]);
            if(i+1<NV_LIST_LEN) assert(nvl.list[i].nx==&nvl.list[i+1]);
        }
        assert(nvl.list[2].depth==(p==2?2:1));
        assert(NV_BODY->valuetype==(p==2?TYPE_PARENT:TYPE_EMPTY));
    }
}
'''
        self.compile_and_run(code)

    @unittest.expectedFailure
    def test_line_headers_are_linked_without_before_array_pointer(self):
        code = "#define RX_HEADERS 12\n#define RX_BUFFER_POOL_SIZE 1000\n#define RX_BUFFER_MIN_SIZE 80\n"
        code += allocator.PRELUDE.replace("buf_hdr_t buf[RX_HEADERS];", "buf_hdr_t buf[RX_HEADERS]; char sanitizer_tail;")
        code += "static struct {char rx_pool[RX_BUFFER_POOL_SIZE];} bufpool;\n"
        code += function((FW / "xio.c").read_text(), r'static void _init_readline_linemode\((?:void)?\)\s*(?=\{)')
        code += r'''
int main(void) {
    for(unsigned n=0;n<3;n++) {
        memset(&bm,0xa5,sizeof bm);
        _init_readline_linemode();
        for(unsigned i=0;i<RX_HEADERS;i++) {
            assert(bm.buf[i].pv==&bm.buf[(i+RX_HEADERS-1)%RX_HEADERS]);
            assert(bm.buf[i].nx==&bm.buf[(i+1)%RX_HEADERS]);
            assert(bm.buf[i].bufp==bufpool.rx_pool);
            assert(bm.buf[i].state==BUFFER_FREE);
        }
    }
}
'''
        self.compile_and_run(code, ("-O0",))

    @unittest.expectedFailure
    def test_arc_time_call_has_no_uninitialized_input(self):
        source = (FW / "plan_arc.c").read_text()
        call = next(line for line in source.splitlines() if "float segments_for_minimum_time =" in line)
        helper = function(source, r'static float _estimate_arc_time\s*\([^;{}]*\)\s*(?=\{)')
        code = r'''
#include <assert.h>
#include <math.h>
#define INVERSE_TIME_MODE 0
#define MICROSECONDS_PER_MINUTE 60000000.0f
#define MIN_ARC_SEGMENT_USEC 5000.0f
#define max(a,b) ((a)>(b)?(a):(b))
static struct {struct {int feed_rate_mode;float feed_rate;}gm;
    float length,planar_travel,linear_travel;int plane_axis_0,plane_axis_1,linear_axis;}arc;
static struct {struct {float feed_rate;}gm;struct {float feedrate_max;}a[3];}cm;
'''
        code += helper
        code += '\nint main(void){\n'
        code += 'arc.length=10;cm.gm.feed_rate=100;arc.gm.feed_rate_mode=1;\n'
        code += 'for(int a=0;a<3;a++)cm.a[a].feedrate_max=100;\n'
        if "_estimate_arc_time(arc_time)" in call:
            code += "float arc_time;\n"
        code += call + '\nassert(segments_for_minimum_time>0);}\n'
        self.compile_and_run(code, ("-Werror=uninitialized",))
