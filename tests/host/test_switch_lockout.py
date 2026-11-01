"""Test real switch dispatch and RTC lockout expiry with simulated port inputs.

Run: python3 tests/host/test_switch_lockout.py
This checks state transitions and register preservation, not interrupt latency or
physical switch timing. No device is opened.
"""
import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


def function(source, name):
    match = re.search(r"(?m)^[\w ]+\b" + name + r"\([^\n]*\)\s*\{", source)
    if not match:
        raise AssertionError(f"missing function {name}")
    return source[match.start():source.index("\n}", match.end()) + 2]


PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define PMIC_MEDLVLEN_bm 2
#define PMIC_HILVLEN_bm 4
#define SW_MIN_BIT_bm 64
#define SW_MAX_BIT_bm 128
#define SR_REQUEST_ASAP 1
#define ISR(name) void name(void)
enum {AXIS_X, AXIS_Y, AXIS_Z, AXIS_A};
enum {CYCLE_OFF, CYCLE_HOMING, CYCLE_PROBE};
enum {FEEDHOLD_OFF, FEEDHOLD_REQUESTED, FEEDHOLD_SYNC};
enum {MOTION_STOP, MOTION_RUN, MOTION_HOLD};
static struct {uint8_t CTRL;} PMIC;
static struct {uint8_t IN;} ports[4];
static struct {typeof(ports[0]) *sw_port[4];} hw;
static struct {int cycle_state, hold_state, motion_state; bool hold_fast, hold_fast_request;} cm;   /* E031 limit fast stop */
static unsigned reports, holds, limits;
static bool polling;
static uint8_t saved_pmic;
static void sr_request_status_report(int request) {
    assert(request==SR_REQUEST_ASAP);
    if(polling) {
        assert(!(PMIC.CTRL&PMIC_MEDLVLEN_bm));
        assert((PMIC.CTRL&PMIC_HILVLEN_bm)==(saved_pmic&PMIC_HILVLEN_bm));
    }
    reports++;
}
static void cm_request_feedhold(void) {holds++;}
static void cm_set_motion_state(int state) {cm.motion_state=state;}
static void controller_assert_limit_condition(uint8_t n) {assert(n>=1&&n<=8);limits++;}
'''

EXERCISE = r'''
static void pin(uint8_t n,bool active) {
    uint8_t mask=(n&1)?SW_MAX_BIT_bm:SW_MIN_BIT_bm;
    bool high=active^(sw.s[n].type==SW_ACTIVE_LO);
    if(high)ports[n/2].IN|=mask;else ports[n/2].IN&=~mask;
}
static void ticks(unsigned n) {
    while(n--) {
        saved_pmic=PMIC.CTRL;polling=true;RTC_COMP_vect();polling=false;
        assert(PMIC.CTRL==saved_pmic);
    }
}
static void setup(unsigned n,unsigned type,unsigned cycle,unsigned mode) {
    memset(&sw,0,sizeof(sw));memset(ports,0,sizeof(ports));
    rtc.rtc_ticks=10;rtc.sys_ticks=100;
    for(unsigned i=0;i<4;i++)hw.sw_port[i]=&ports[i];
    sw.switch_type=type;sw.s[n].mode=mode;
    sw.s[n].type=mode==SW_MODE_PROBE?SW_ACTIVE_LO:type;
    pin(n,false);reset_switches();
    memset(&cm,0,sizeof(cm));cm.cycle_state=cycle;reports=holds=limits=0;PMIC.CTRL=7;
}
int main(void) {
    unsigned cases=0;
    for(unsigned n=0;n<NUM_SWITCHES;n++)for(unsigned type=0;type<2;type++)
    for(unsigned cycle=0;cycle<3;cycle++) {
        setup(n,type,cycle,cycle==CYCLE_PROBE?SW_MODE_PROBE:SW_MODE_HOMING_LIMIT);
        pin(n,true);_dispatch_switch(n);assert(sw.s[n].state==SW_ACTIVE);
        assert(reports==1);
        ticks(2);pin(n,false);_dispatch_switch(n); /* release inside the lockout */
        assert(sw.s[n].state==SW_ACTIVE&&reports==1);
        ticks(3);assert(sw.s[n].state==SW_ACTIVE&&reports==1);
        ticks(1);
        if(sw.s[n].state!=SW_INACTIVE) {
            fputs("E050: switch stayed active after the ignored release and lockout expiry\n",stderr);
            return 2;
        }
        assert(reports==2);
        assert(holds==(cycle==CYCLE_HOMING?2u:cycle==CYCLE_PROBE?1u:0u));
        assert(limits==(cycle==CYCLE_OFF?1u:0u));
        ticks(6);pin(n,true);_dispatch_switch(n);
        assert(reports==3); /* next real closure must trigger again */
        assert(holds==(cycle==CYCLE_HOMING?3u:cycle==CYCLE_PROBE?2u:0u));
        assert(limits==(cycle==CYCLE_OFF?2u:0u));
        ticks(30);assert(reports==3); /* stable input must not retrigger */
        assert(!Timeout_isSet(&sw.s[n].timeout));
        cases++;
    }
    /* Closure during release lockout must also be delivered at expiry. */
    for(unsigned n=0;n<NUM_SWITCHES;n++) {
        setup(n,SW_ACTIVE_HI,CYCLE_OFF,SW_MODE_LIMIT);
        pin(n,true);_dispatch_switch(n);ticks(6);
        pin(n,false);_dispatch_switch(n);ticks(1);
        pin(n,true);_dispatch_switch(n);assert(limits==1);ticks(5);
        assert(limits==2&&sw.s[n].state==SW_ACTIVE);cases++;
    }
    /* Expired inputs are independent; a delayed port ISR must not duplicate an edge. */
    setup(0,SW_ACTIVE_HI,CYCLE_OFF,SW_MODE_LIMIT);
    for(unsigned n=0;n<NUM_SWITCHES;n++) {
        sw.s[n].mode=SW_MODE_LIMIT;sw.s[n].type=SW_ACTIVE_HI;pin(n,false);
    }
    reset_switches();
    for(unsigned n=0;n<NUM_SWITCHES;n++) {pin(n,true);_dispatch_switch(n);pin(n,false);}
    ticks(6);assert(limits==NUM_SWITCHES&&reports==NUM_SWITCHES*2);
    for(unsigned n=0;n<NUM_SWITCHES;n++) {
        assert(sw.s[n].state==SW_INACTIVE);_dispatch_switch(n);
    }
    assert(reports==NUM_SWITCHES*2);cases++;
    /* Preserve all PMIC bits, including high-level stepping, and tick rollover. */
    for(unsigned bits=0;bits<256;bits++) {
        setup(0,SW_ACTIVE_HI,CYCLE_OFF,SW_MODE_LIMIT);
        PMIC.CTRL=bits;pin(0,true);_dispatch_switch(0);pin(0,false);ticks(6);
        assert(sw.s[0].state==SW_INACTIVE&&PMIC.CTRL==bits);cases++;
    }
    setup(0,SW_ACTIVE_HI,CYCLE_OFF,SW_MODE_LIMIT);
    rtc.rtc_ticks=UINT32_MAX/10-2;rtc.sys_ticks=rtc.rtc_ticks*10;
    pin(0,true);_dispatch_switch(0);pin(0,false);ticks(7);
    assert(sw.s[0].state==SW_INACTIVE);cases++;
    setup(0,SW_ACTIVE_HI,CYCLE_OFF,SW_MODE_DISABLED);
    pin(0,true);_dispatch_switch(0);ticks(20);
    assert(reports==0&&limits==0&&sw.s[0].state==SW_DISABLED);cases++;
    printf("E050: %u lockout, polarity, cycle, mask and rollover scenarios passed\n",cases);
    return 0;
}
'''


class SwitchLockoutTests(unittest.TestCase):
    def test_lockout_expiry_recovers_edges(self):
        compiler = shutil.which("cc")
        if not compiler:
            self.skipTest("host C compiler not installed")
        firmware = ROOT / "firmware/tinyg"
        source = (firmware / "switch.c").read_text()
        header = (firmware / "switch.h").read_text()
        clock = (firmware / "xmega/xmega_rtc.c").read_text()
        clock_header = (firmware / "xmega/xmega_rtc.h").read_text()
        types = clock_header[clock_header.index("typedef struct rtClock"):clock_header.index("// Other stuff")]
        switch_types = header[header.index("#define SW_LOCKOUT_MS"):header.index("/****************", header.index("switches_t sw;"))]
        actual = "\n".join(function(clock, name) for name in
                           ("Timeout_isSet", "Timeout_isPast", "Timeout_set", "Timeout_clear"))
        actual += "\n" + function(source, "_read_raw_switch")
        actual += "\n" + function(source, "reset_switches")
        actual += "\n" + function(source, "_dispatch_switch")
        if "void switch_rtc_callback(" in source:
            actual += "\n" + function(source, "switch_rtc_callback")
        start = clock.index("ISR(RTC_COMP_vect)")
        actual += "\n" + clock[start:clock.index("\n}", start) + 2]
        with tempfile.TemporaryDirectory(prefix="tinyg-switch-") as directory:
            path = pathlib.Path(directory)
            clock_access = ("uint8_t SREG = 0x80;\nstatic void cli(void) { SREG &= (uint8_t)~0x80; }\n"
                            "uint32_t SysTickTimer_getValue(void) { return rtc.sys_ticks; }\n")   # E190
            (path / "check.c").write_text(PRELUDE + types + switch_types + "\nrtClock_t rtc;\n" + clock_access +
                                          actual + EXERCISE)
            subprocess.run([compiler, "-std=gnu99", "-O1", "-fsanitize=address,undefined",
                            str(path / "check.c"), "-o", str(path / "check")], check=True)
            result = subprocess.run([str(path / "check")], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            print(result.stdout.strip())


if __name__ == "__main__":
    unittest.main()
