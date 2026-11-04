"""E190: the four-byte system clock is read and written in parts across interrupts.

rtc.sys_ticks (uint32) is written by the RTC compare interrupt (low level) and read by the main
loop (SysTickTimer_getValue, util.c) and by the switch interrupts (medium level) through
Timeout_isPast/Timeout_set (xmega_rtc.c). avr-gcc loads and stores it one byte at a time, low
byte first. Two consequences:
  - main reads the low byte of 65530 (0x0000fffa), the clock advances to 65540 (0x00010004),
    main reads the new upper bytes: 65786, 246 ms early; a motor power deadline at 65600 is
    taken as passed and the motor is turned off;
  - the RTC has stored the low half of 65540 when a switch interrupt preempts it: the switch sees
    time 4, a 50 ms lockout started at 65500 looks long expired, and a homing edge at 40 ms is
    accepted during lockout.

The real functions are compiled; every load of rtc.sys_ticks in the getter and every store in
the RTC handler is split into its bytes, with the other side's interrupt arriving between them.
The interrupt arrives mid-access only while interrupts are enabled (SREG bit 7 and, for the
switch, the medium level); otherwise it runs right after the access completes, as the hardware
would deliver it. Uninjected runs are the controls.
"""
from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function
from test_motor_energize_timeout import PRELUDE as MOTOR_PRELUDE
from test_switch_lockout import PRELUDE as SWITCH_PRELUDE, EXERCISE as SWITCH_EXERCISE

ROOT = Path(__file__).resolve().parents[2]
FW = Path(os.environ.get("TINYG_FIRMWARE", ROOT / "firmware/tinyg"))
DEF = r"[^;{}]*\)(?=\s*(?://[^\n]*\s*)?\{)"


def src(path):
    return (FW / path).read_text(encoding="latin-1")


def fn(path, name):
    return function(src(path), r"(?:static )?(?:inline )?(?:void|bool|int|uint8_t|uint32_t|stat_t|swState) " + name + r"\(" + DEF)


INTERRUPTS = r'''
#ifndef SREG_DEFINED
#define SREG_DEFINED
uint8_t SREG = 0x80;
static void cli(void) { SREG &= (uint8_t)~0x80; }
#endif
'''


def split_loads(code):
    """Every read of rtc.sys_ticks becomes a byte-by-byte load with the interrupt window inside."""
    return re.sub(r"rtc\.sys_ticks(?!\s*=[^=])", "split_load()", code)


def split_stores(code):
    """Every store to rtc.sys_ticks becomes a byte-by-byte store with the interrupt window inside."""
    return re.sub(r"rtc\.sys_ticks\s*=(?!=)\s*([^;]+);", r"split_store(\1);", code)


def main_case():
    pre = MOTOR_PRELUDE.replace("uint32_t SysTickTimer_getValue(void) {return tick;}", "")
    # the motor prelude may already define SREG and cli()
    code = pre + ("" if "uint8_t SREG" in pre else INTERRUPTS) + r'''
#include <stdio.h>
#define ISR(n) void n(void)
struct {uint32_t rtc_ticks,sys_ticks;} rtc;
static int inject, pending;
static void switch_rtc_callback(void) {}
void RTC_COMP_vect(void);
static uint32_t split_load(void) {
    unsigned char *p = (unsigned char *)&rtc.sys_ticks;
    uint32_t value = p[0];
    if (inject) {
        inject = 0;
        if (SREG & 0x80) { RTC_COMP_vect(); } else { pending = 1; }
    }
    for (unsigned i = 1; i < 4; i++) { value |= (uint32_t)p[i] << (8 * i); }
    if (pending && (SREG & 0x80)) { pending = 0; RTC_COMP_vect(); }
    return value;
}
'''
    code += "static void xio_usb_rtc_callback(void) {}   /* E195: the USB quiet-time tick */\n" + function(src("xmega/xmega_rtc.c"), r"ISR\(RTC_COMP_vect\)")
    code += split_loads(fn("util.c", "SysTickTimer_getValue"))
    for n in ("_deenergize_motor", "_energize_motor", "st_motor_power_callback"):
        code += fn("stepper.c", n)
    return code + r'''
int main(int argc, char **argv) {
    (void)argv; inject = (argc == 1);
    rtc.rtc_ticks = 6553; rtc.sys_ticks = 65530;
    for (unsigned m = 0; m < MOTORS; m++) { st_cfg.mot[m].power_mode = MOTOR_ALWAYS_POWERED; }
    st_cfg.mot[0].power_mode = MOTOR_POWERED_ONLY_WHEN_MOVING;
    st_run.mot[0].power_state = MOTOR_POWER_TIMEOUT_COUNTDOWN;
    st_run.mot[0].power_systick = 65500;
    st_run.mot[0].power_duration = 100;
    st_motor_power_callback();
    if (pending) { pending = 0; RTC_COMP_vect(); }     /* a masked tick is delivered afterwards */
    assert(rtc.sys_ticks < 65600);
    if (!_motor_is_enabled(0)) { fputs("E190: motor turned off before its deadline\n", stderr); return 66; }
    return 0;
}
'''


def switch_case():
    h = src("switch.h")
    clock_h = src("xmega/xmega_rtc.h")
    types = clock_h[clock_h.index("typedef struct rtClock"):clock_h.index("// Other stuff")]
    types += h[h.index("#define SW_LOCKOUT_MS"):h.index("/****************", h.index("switches_t sw;"))]
    code = SWITCH_PRELUDE + INTERRUPTS + "\nstatic int inject, pending;\n" + types + "\nrtClock_t rtc;\n"
    code += "uint32_t SysTickTimer_getValue(void);\n"
    code += "\n".join(fn("xmega/xmega_rtc.c", n) for n in ("Timeout_isSet", "Timeout_isPast", "Timeout_set", "Timeout_clear"))
    code += fn("switch.c", "_read_raw_switch") + "\n" + fn("switch.c", "reset_switches")
    code += "\n" + fn("switch.c", "_dispatch_switch") + "\n" + fn("switch.c", "switch_rtc_callback")
    code += "\n" + fn("util.c", "SysTickTimer_getValue")
    code += r'''
/* The RTC stores low byte first; a medium-level switch interrupt can preempt its low-level handler. */
static void split_store(uint32_t value) {
    unsigned char *p = (unsigned char *)&rtc.sys_ticks;
    p[0] = value; p[1] = value >> 8;
    if (inject) {
        inject = 0;
        if ((SREG & 0x80) && (PMIC.CTRL & PMIC_MEDLVLEN_bm)) { _dispatch_switch(0); } else { pending = 1; }
    }
    p[2] = value >> 16; p[3] = value >> 24;
}
'''
    code += "static void xio_usb_rtc_callback(void) {}   /* E195: the USB quiet-time tick */\n" + split_stores(function(src("xmega/xmega_rtc.c"), r"ISR\(RTC_COMP_vect\)"))
    code += SWITCH_EXERCISE[:SWITCH_EXERCISE.index("int main(void)")]
    return code + r'''
int main(int argc, char **argv) {
    (void)argv; setup(0, SW_ACTIVE_HI, CYCLE_HOMING, SW_MODE_HOMING);
    rtc.rtc_ticks = 6553; rtc.sys_ticks = 65530;
    sw.s[0].timeout._start = 65500; sw.s[0].timeout._delay = 50;
    pin(0, true); inject = (argc == 1);
    RTC_COMP_vect();
    if (pending) { pending = 0; _dispatch_switch(0); }  /* a masked edge is delivered afterwards */
    assert(rtc.sys_ticks == 65540);
    if (holds) { fputs("E190: switch lockout expired at 40 ms, before 50 ms\n", stderr); return 66; }
    return 0;
}
'''


class ClockRead(unittest.TestCase):
    def _run(self, name, code):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        with tempfile.TemporaryDirectory(prefix="tinyg-e190-") as d:
            c, exe = Path(d) / (name + ".c"), Path(d) / name
            c.write_text(code)
            subprocess.run([cc, "-std=gnu99", "-O1", "-g", "-Wall", "-Wno-unused-function", "-Wno-unused-variable",
                            "-fsanitize=address,undefined", str(c), "-o", str(exe)], check=True)
            control = subprocess.run([str(exe), "control"], capture_output=True, text=True)
            self.assertEqual(control.returncode, 0, control.stderr)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_main_loop_clock_read_keeps_motor_deadline(self):
        self._run("main", main_case())

    def test_switch_interrupt_sees_a_complete_clock(self):
        self._run("switch", switch_case())


if __name__ == "__main__":
    unittest.main()
