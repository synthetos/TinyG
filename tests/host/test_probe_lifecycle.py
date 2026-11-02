"""Deterministic probe lifecycle checks with actual continuation functions."""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT=Path(__file__).resolve().parents[2]
FIRMWARE=Path(os.environ.get('TINYG_FIRMWARE', ROOT/'firmware/tinyg'))


class ProbeLifecycle(unittest.TestCase):
    def test_transitions(self):
        source=(FIRMWARE/'cycle_probing.c').read_text()
        struct=source[source.index('struct pbProbingSingleton {'):source.index('/**** NOTE:')]
        definitions=[]
        names=['cm_straight_probe','cm_probe_callback','_set_pb_func','_probing_init',
               '_probing_start','_probing_finalize_exit']
        if 'static stat_t _probing_error_exit(' in source:
            names.append('_probing_error_exit')
        for name in names:
            sig=r'(?:static )?(?:stat_t|uint8_t) '+name+r'\([^;{}]*\)\s*\{'
            match=re.search(sig,source); assert match,name
            definitions.append(function(source[match.start():],sig[:-5]))
        code=r'''
#include <stdlib.h>
#include <assert.h>
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "switch.h"
#include "stepper.h"
#include "planner.h"
#include "util.h"
#include <avr/interrupt.h>
cmSingleton_t cm;
stPrepSingleton_t st_pre;
volatile uint8_t SREG;
stat_t status_code;
static bool queued, stepping, alarm_on_feed;
static int feeds, finishes, flushes, alarms, cycle_ends;
static stat_t feed_status;
static stat_t _probing_init(void);
static stat_t _probing_start(void);
static stat_t _probing_finalize_exit(void);
static stat_t _probing_error_exit(stat_t);
static stat_t _probing_finish(void) { finishes++; return STAT_OK; }
#define MINIMUM_PROBE_TRAVEL .254
float cm_get_absolute_position(const GCodeState_t *g, const uint8_t a) { return cm.gmx.position[a]; }
float cm_get_work_position(const GCodeState_t *g, const uint8_t a) { return cm.gmx.position[a]; }
float cm_get_axis_jerk(const uint8_t a) { return cm.a[a].jerk_max; }
void cm_set_axis_jerk(const uint8_t a, const float j) { cm.a[a].jerk_max=j; }
uint8_t cm_get_distance_mode(const GCodeState_t *g) { return g->distance_mode; }
uint8_t cm_get_units_mode(const GCodeState_t *g) { (void)g; return MILLIMETERS; }
stat_t cm_set_distance_mode(const uint8_t mode) { cm.gm.distance_mode=mode; return STAT_OK; }
int8_t find_probe_switch(void) { return 0; }
swState read_switch(const uint8_t s) { return SW_INACTIVE; }
static unsigned feedholds;
void cm_request_feedhold(void) { feedholds++; }   /* E192: a contact after the initial sample holds the move */
stat_t cm_spindle_control(uint8_t mode) { queued=true; return STAT_OK; }
stat_t cm_straight_feed(const float *target, const bool *flags) {
    feeds++;
    for(int a=AXIS_A;a<AXES;a++) assert(!flags[a]); // also catches undersized argument arrays
    if(alarm_on_feed) cm.machine_state=MACHINE_ALARM;
    if(feed_status==STAT_OK) queued=true;
    return feed_status;
}
float mp_get_runtime_absolute_position(uint8_t a) { return .25+a; }
void cm_set_position(const uint8_t a,const float p) { cm.gmx.position[a]=p; cm.gm.target[a]=p; }
void mp_flush_planner(void) { assert(!stepping); queued=false; flushes++; }
void cm_set_motion_mode(GCodeState_t *g,const uint8_t mode) { g->motion_mode=mode; }
void cm_cycle_end(void) { cycle_ends++; cm.machine_state=MACHINE_PROGRAM_STOP; cm.hold_state=FEEDHOLD_OFF; }
stat_t cm_is_alarmed(void) {
    if(cm.machine_state==MACHINE_ALARM) return STAT_COMMAND_REJECTED_BY_ALARM;
    if(cm.machine_state==MACHINE_SHUTDOWN) return STAT_COMMAND_REJECTED_BY_SHUTDOWN;
    if(cm.machine_state==MACHINE_PANIC) return STAT_COMMAND_REJECTED_BY_PANIC;
    return STAT_OK;
}
stat_t cm_alarm_P(const stat_t s,const char *msg) { status_code=s; alarms++; cm.machine_state=MACHINE_ALARM; return s; }
bool mp_runtime_is_idle(void) { assert(!(SREG&128)); return !stepping; }
bool mp_has_runnable_buffer(void) { assert(!(SREG&128)); return queued; }
uint8_t cm_get_runtime_busy(void) { return cm.cycle_state==CYCLE_OFF ? false : stepping; }
'''
        code+=struct+'\n'+'\n'.join(definitions)
        code+=r'''
static void setup(void) {
    memset(&cm,0,sizeof(cm)); memset(&pb,0,sizeof(pb)); memset(&st_pre,0,sizeof(st_pre));
    st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC; SREG=0x95;
    queued=stepping=alarm_on_feed=false; feeds=finishes=flushes=alarms=cycle_ends=0;
    feed_status=STAT_OK; status_code=STAT_OK;
    cm.gm.feed_rate=100; cm.gm.feed_rate_mode=UNITS_PER_MINUTE_MODE;
    cm.gm.distance_mode=INCREMENTAL_MODE; cm.machine_state=MACHINE_PROGRAM_STOP;
    for(int a=0;a<3;a++) { cm.a[a].jerk_max=100+a; cm.a[a].jerk_homing=10+a; }
}
static void begin(void) {
    float target[AXES]={1,0,0,0,0,0}; bool flags[AXES]={true,false,false,false,false,false};
    assert(cm_straight_probe(target,flags)==STAT_OK); // gf.feed_rate is false: modal feed
    assert(cm.probe_state==PROBE_WAITING && cm.cycle_state==CYCLE_OFF);
}
static void init_probe(void) {
    assert(cm_probe_callback()==STAT_EAGAIN); // initialization; E061: probing leaves the spindle alone
    assert(cm.cycle_state==CYCLE_PROBE && cm.gm.distance_mode==ABSOLUTE_MODE);
    assert(!queued && feeds==0);                // nothing queued, so nothing to drain
}
int main(void) {
    setup(); begin();
    queued=true; assert(cm_probe_callback()==STAT_EAGAIN && cm.cycle_state==CYCLE_OFF);
    queued=false; stepping=true; assert(cm_probe_callback()==STAT_EAGAIN && cm.cycle_state==CYCLE_OFF);
    stepping=false; st_pre.buffer_state=PREP_BUFFER_OWNED_BY_LOADER;
    assert(cm_probe_callback()==STAT_EAGAIN && cm.cycle_state==CYCLE_OFF);
    assert(SREG==0x95 && cm.gm.distance_mode==INCREMENTAL_MODE && cm.a[0].jerk_max==100);
    st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;
    queued=true; cm.hold_state=FEEDHOLD_HOLD; // a previous held move must not start probing
    assert(cm_probe_callback()==STAT_EAGAIN && cm.cycle_state==CYCLE_OFF);
    queued=false; cm.hold_state=FEEDHOLD_OFF;
    init_probe();
    assert(cm_probe_callback()==STAT_EAGAIN && feeds==1 && feedholds==0); // inactive probe: no hold
    assert(cm_probe_callback()==STAT_EAGAIN && finishes==0); // not yet loaded
    queued=false; stepping=true;
    assert(cm_probe_callback()==STAT_EAGAIN && finishes==0);
    stepping=false; st_pre.buffer_state=PREP_BUFFER_OWNED_BY_LOADER;
    assert(cm_probe_callback()==STAT_EAGAIN && finishes==0);
    st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;
    queued=true; cm.hold_state=FEEDHOLD_HOLD; // stopped contact leaves a held buffer
    assert(cm_probe_callback()==STAT_OK && finishes==1);
    assert(SREG==0x95);
    // A rejected move must restore configuration, report once, and never retry.
    for(int preset=0;preset<2;preset++) {
        setup(); begin(); init_probe(); feed_status=STAT_MINIMUM_TIME_MOVE;
        if(preset==1) alarm_on_feed=true;
        assert(cm_probe_callback()==STAT_MINIMUM_TIME_MOVE && feeds==1);
        assert(cm.cycle_state==CYCLE_OFF && cm.probe_state==PROBE_FAILED && pb.func==NULL);
        assert(cm.gm.distance_mode==INCREMENTAL_MODE && flushes==1);
        for(int a=0;a<3;a++) assert(cm.a[a].jerk_max==100+a);
        for(int a=0;a<AXES;a++) assert(cm.gm.target[a]==.25+a);
        assert(cm_probe_callback()==STAT_NOOP && feeds==1);
        assert(alarms==(preset==0 ? 1 : 0));
        if(preset==0) assert(status_code==STAT_MINIMUM_TIME_MOVE); // preserve rejection, not generic miss
        if(preset>=1) assert(cycle_ends==0); // do not erase alarm/shutdown/panic
        assert(cm.machine_state==MACHINE_ALARM);
    }
    // Already in SHUTDOWN or PANIC when the move is staged: end the probe without queuing it (E073).
    for(int preset=2;preset<4;preset++) {
        setup(); begin(); init_probe();
        cm.machine_state=(preset==2) ? MACHINE_SHUTDOWN : MACHINE_PANIC;
        assert(cm_probe_callback()==STAT_OK && feeds==0);
        assert(cm.cycle_state==CYCLE_OFF && cm.probe_state==PROBE_FAILED);
        assert(cm.gm.distance_mode==INCREMENTAL_MODE);
        for(int a=0;a<3;a++) assert(cm.a[a].jerk_max==100+a);
        assert(alarms==0 && cycle_ends==0);
        assert(cm.machine_state==((preset==2) ? MACHINE_SHUTDOWN : MACHINE_PANIC));
    }
    setup(); cm.gm.feed_rate=0;
    float target[AXES]={1}; bool flags[AXES]={true};
    assert(cm_straight_probe(target,flags)==STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
    setup(); cm.gm.feed_rate_mode=INVERSE_TIME_MODE;
    assert(cm_straight_probe(target,flags)==STAT_GCODE_INVERSE_TIME_MODE_CANNOT_BE_USED);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp); (tmp/'avr').mkdir()
            (tmp/'avr/pgmspace.h').write_text('#define PROGMEM\n#define PSTR(s) (s)\n')
            (tmp/'avr/interrupt.h').write_text('#include <stdint.h>\nextern volatile uint8_t SREG;\n#define cli() (SREG &= (uint8_t)~128)\n')
            c=tmp/'probe.c'; c.write_text(code); exe=tmp/'probe'
            subprocess.run(['cc','-std=gnu99','-O1','-g','-fsanitize=address,undefined',
                            '-D_POSIX_C_SOURCE=200809L','-Wno-macro-redefined','-I'+str(tmp),
                            '-I'+str(FIRMWARE),str(c),'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)


if __name__ == '__main__':
    unittest.main()
