"""E192: a probe contact seen by the switch interrupt while the probe cycle starts must not be lost.

_probing_start() (cycle_probing.c) samples the probe pin with read_switch() (switch.c), which reads
the pin and then stores the result in sw.s[].state. If the probe touches between the two, the
switch interrupt records the contact and requests a feedhold, which does nothing while motion is
stopped; read_switch() then stores its older, inactive sample over it, and the probe move is queued
with the probe already in contact and nothing to stop it. Starting with the probe tripped is an
error (g2core wiki Gcode-Probes, G38.x errors); a contact just after the sample must stop the move.

The real read_switch(), switch dispatch, feedhold request and _probing_start() are compiled; the
probe touches right after the pin is sampled. The edge interrupt runs at that point if interrupts
are enabled, else when the critical section ends. The uninjected run is the control.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function
from test_switch_lockout import PRELUDE as SWITCH_PRELUDE, EXERCISE as SWITCH_EXERCISE

ROOT = Path(__file__).resolve().parents[2]
FW = Path(os.environ.get("TINYG_FIRMWARE", ROOT / "firmware/tinyg"))
DEF = r"[^;{}]*\)(?=\s*(?://[^\n]*\s*)?\{)"
INTERRUPTS = r'''
#define __AVR 1
uint8_t SREG = 0x80;
static void cli(void) { SREG &= (uint8_t)~0x80; }
'''


def src(path):
    return (FW / path).read_text(encoding="latin-1")


def fn(path, name):
    return function(src(path), r"(?:static )?(?:inline )?(?:void|bool|int|uint8_t|uint32_t|stat_t|swState) " + name + r"\(" + DEF)


def switch_code(probe=False):
    h = src("switch.h")
    clock_h = src("xmega/xmega_rtc.h")
    types = clock_h[clock_h.index("typedef struct rtClock"):clock_h.index("// Other stuff")]
    types += h[h.index("#define SW_LOCKOUT_MS"):h.index("/****************", h.index("switches_t sw;"))]
    pre = SWITCH_PRELUDE + INTERRUPTS + "\nstatic int inject, pending;\n"
    if probe:
        pre = pre.replace("bool hold_fast,", "bool end_hold_requested, hold_fast,")
        pre += "\nstatic struct {bool resume_cancelled;} hold_outputs;\n"
        pre = pre.replace("static void cm_request_feedhold(void) {holds++;}", "void cm_request_feedhold(void);")
    code = pre + types + "\nrtClock_t rtc;\n"
    code += "uint32_t SysTickTimer_getValue(void) { return rtc.sys_ticks; }   /* switch.c reads the clock through it (E190) */\n"
    code += "\n".join(fn("xmega/xmega_rtc.c", n) for n in ("Timeout_isSet", "Timeout_isPast", "Timeout_set", "Timeout_clear"))
    raw = fn("switch.c", "_read_raw_switch")
    if probe:
        code += "\nstatic void after_pin_sample(void);\n"
        raw = raw.replace("return ((bool)raw", "after_pin_sample();\n    return ((bool)raw")
    code += raw + "\n" + fn("switch.c", "reset_switches")
    code += "\n" + fn("switch.c", "_dispatch_switch")
    code += "\n" + fn("switch.c", "switch_rtc_callback")
    return code


def probe_case():
    code = switch_code(probe=True)
    code += "\n" + "static void xio_usb_rtc_callback(void) {}   /* E195: the USB quiet-time tick */\n" + function(src("xmega/xmega_rtc.c"), r"ISR\(RTC_COMP_vect\)")
    code += SWITCH_EXERCISE[:SWITCH_EXERCISE.index("int main(void)")]
    code += r'''
typedef int stat_t;
#define STAT_OK 0
static bool mp_hold_startup(void) {return false;}
#define CYCLE_MACHINING 99
#define FEEDHOLD_HOLD 6
#define PREP_BUFFER_OWNED_BY_LOADER 0
#define BLOCK_TYPE_DWELL 3
#define CONTROLLER_READY 0
static struct {int buffer_state,block_type;} st_pre;
static struct {int controller_state;} cs;
static bool st_runtime_isbusy(void) {return cm.motion_state==MOTION_RUN;}
static bool st_aline_in_flight(void) {return cm.motion_state==MOTION_RUN;}
static int cm_is_alarmed(void) {return 0;}
static void cm_cycle_start(void) {assert(0);} // no dwell is modeled in this probe fixture

'''
    code += fn("canonical_machine.c", "cm_request_feedhold")
    code += r'''
/* the probe touches right after the pin is sampled; the edge interrupt runs then if interrupts are
   enabled, else as soon as the critical section ends (before anything else the main loop does) */
static void after_pin_sample(void) {
    if(inject) {inject=0;pin(0,true);if(SREG&0x80)_dispatch_switch(0);else pending=1;}
}
static void deliver_pending(void) {if(pending&&(SREG&0x80)){pending=0;_dispatch_switch(0);}}
'''
    code += fn("switch.c", "read_switch")
    code += r'''
struct {bool already_tripped;uint8_t probe_switch;float target[6];bool flags[6];} pb;
static unsigned moves;
static stat_t cm_straight_feed(float *target,bool *flags) {
    deliver_pending();
    (void)target;(void)flags;moves++;cm.motion_state=MOTION_RUN;return STAT_OK;
}
static stat_t _probing_finish(void) {return STAT_OK;}
static stat_t _set_pb_func(stat_t (*f)(void)) {(void)f;return STAT_OK;}
static stat_t _probing_error_exit(stat_t s) {assert(0);return s;}
'''
    code += fn("cycle_probing.c", "_probing_start")
    return code + r'''
int main(int argc,char **argv) {
    (void)argv;setup(0,SW_ACTIVE_HI,CYCLE_PROBE,SW_MODE_PROBE);
    cm.motion_state=MOTION_STOP;pb.probe_switch=0;inject=argc==1;
    _probing_start();
    deliver_pending();
    if(argc==1) {
        assert(_read_raw_switch(0));
        if(moves && cm.hold_state==FEEDHOLD_OFF) {fputs("E192: probe move starts past an unhandled contact\n",stderr);return 66;}
    } else assert(moves==1);
    return 0;
}
'''



class ProbeInitialContact(unittest.TestCase):
    def test_contact_during_initial_sample_stops_the_probe_move(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        with tempfile.TemporaryDirectory(prefix="tinyg-e192-") as d:
            c, exe = Path(d) / "probe.c", Path(d) / "probe"
            c.write_text(probe_case())
            subprocess.run([cc, "-std=gnu99", "-O1", "-g", "-Wall", "-Wno-unused-function", "-Wno-unused-variable",
                            "-fsanitize=address,undefined","-fstrict-flex-arrays=3", str(c), "-o", str(exe)], check=True)
            control = subprocess.run([str(exe), "control"], capture_output=True, text=True)
            self.assertEqual(control.returncode, 0, control.stderr)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
