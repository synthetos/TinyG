"""E191: a queue flush accepted while a line is being read must stop that line.

_dispatch_command() and _dispatch_control() (controller.c) check for a pending host '%' and then
read a line. The RX interrupt can accept '%' (in a feedhold) while readline() is still returning
a line that was buffered before it. The dispatcher then ran that pre-flush line, for example a
G92, which the later flush cannot undo.

The real dispatch functions and flush request helpers are compiled; the reader stand-in accepts a
'%' just before it returns the line, and the parser stand-in counts dispatches. The uninjected
run is the control.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FW = Path(os.environ.get("TINYG_FIRMWARE", ROOT / "firmware/tinyg"))
DEF = r"[^;{}]*\)(?=\s*(?://[^\n]*\s*)?\{)"


def src(path):
    return (FW / path).read_text(encoding="latin-1")


def fn(path, name):
    return function(src(path), r"(?:static )?(?:inline )?(?:void|bool|int|uint8_t|uint32_t|stat_t) " + name + r"\(" + DEF)


def flush_case():
    code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
typedef int stat_t;
typedef uint16_t devflags_t;
enum {STAT_OK,DEV_IS_BOTH=3,DEV_IS_CTRL=1,DEV_IS_OVERFLOW=8};
enum {FEEDHOLD_OFF,FEEDHOLD_HOLD,FLUSH_OFF=0,FLUSH_REQUESTED=1};
#define _FDEV_ERR -1
struct {int hold_state,queue_flush_state;bool queue_flush_input,queue_flush_input_due;} cm;
struct {char *bufp;} cs;
static int inject,dispatched;
void cm_request_planner_flush(void);
'''
    code += fn("canonical_machine.c", "cm_request_planner_flush")
    code += fn("canonical_machine.c", "cm_request_queue_flush")
    code += r'''
static char *readline(devflags_t *flags,uint16_t *size) {
    (void)flags;*size=7;
    /* The line is read; RX receives an honored '%' before this call returns. */
    if(inject) cm_request_queue_flush();
    return "G92 X0";
}
static void _count_line(uint16_t n) {(void)n;}
static void _reject_damaged_line(void) {assert(0);}
static void _reject_overlong_line(void) {assert(0);}
static void _dispatch_kernel(void) {dispatched++;}
'''
    code += fn("controller.c", "_dispatch_command")
    code += fn("controller.c", "_dispatch_control")
    return code + r'''
int main(int argc,char **argv) {
    (void)argv;inject=argc==1;int failed=0;
    for(int control=0;control<2;control++) {
        cm.hold_state=FEEDHOLD_HOLD;cm.queue_flush_state=FLUSH_OFF;
        cm.queue_flush_input=cm.queue_flush_input_due=false;dispatched=0;
        if(control)_dispatch_control();else _dispatch_command();
        if(inject && dispatched) {fputs("E191: pre-flush line dispatched after honored '%'\n",stderr);failed=66;}
        if(!inject)assert(dispatched==1);
    }
    return failed;
}
'''



class FlushAfterRead(unittest.TestCase):
    def test_flush_during_read_stops_the_line(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        with tempfile.TemporaryDirectory(prefix="tinyg-e191-") as d:
            c, exe = Path(d) / "flush.c", Path(d) / "flush"
            c.write_text(flush_case())
            subprocess.run([cc, "-std=gnu99", "-O1", "-g", "-Wall", "-Wno-unused-function", "-Wno-unused-variable",
                            "-fsanitize=address,undefined", str(c), "-o", str(exe)], check=True)
            control = subprocess.run([str(exe), "control"], capture_output=True, text=True)
            self.assertEqual(control.returncode, 0, control.stderr)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
