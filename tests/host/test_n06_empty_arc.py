"""N06: a sub-resolution full circle must not leave a machining cycle with no work.

NIST RS274NGC v3 3.5.3 permits center-format full circles. TinyG's chordal
approximation may produce a zero-length chord; that is not an endless cycle.
Runs the real arc calculator, callback, planner, line and trapezoid code.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]
FW=ROOT/'firmware/tinyg'
FIX=Path(__file__).parent/'planner_startup'

class EmptyArc(unittest.TestCase):
    def test_tiny_circle_and_queued_controls(self):
        pre=(FIX/'check.c').read_text().split('static void exec_pending')[0]
        pre=pre.replace('void cm_cycle_end(void) {}', 'void cm_cycle_end(void) { cm.cycle_state=CYCLE_OFF; cm.machine_state=MACHINE_PROGRAM_STOP; }')
        pre += '''
#include "util.h"
void cm_cycle_start(void) { cm.cycle_state=CYCLE_MACHINING; cm.machine_state=MACHINE_CYCLE; }
stat_t cm_alarm_P(stat_t s,const char *msg) { cm.machine_state=MACHINE_ALARM; return s; }
void cm_request_feedhold(void) {}
arc_t arc;
'''
        source=(FW/'plan_arc.c').read_text()
        for sig in [r'static void _compute_arc_offsets_from_radius\(\)\s*',r'static float _estimate_arc_time \(float arc_time\)\s*',r'static stat_t _compute_arc\(const bool radius_f\)\s*',r'stat_t cm_arc_callback\(\)\s*']:
            pre+=function(source,sig + r"(?=\{)")
        pre += r'''
static void tiny(void) {
    memset(&arc,0,sizeof(arc));
    cm.gm.feed_rate=100; cm.chordal_tolerance=.01;
    arc.plane_axis_0=0; arc.plane_axis_1=1; arc.linear_axis=2;
    arc.gm.motion_mode=MOTION_MODE_CW_ARC;
    arc.gm.feed_rate_mode=UNITS_PER_MINUTE_MODE; arc.gm.feed_rate=100;
    arc.offset[0]=.0001f;
    assert(_compute_arc(false)==STAT_OK);
    assert(arc.segment_count==1);
    arc.run_state=BLOCK_RUNNING;
    cm_cycle_start();
}
int main(void) {
    setup(); tiny();
    assert(cm_arc_callback()==STAT_OK);
    assert(arc.run_state==BLOCK_IDLE);
    assert(mb.buffers_available==PLANNER_BUFFER_POOL_SIZE);
    assert(cm.cycle_state==CYCLE_OFF);
    // A discarded chord must not end a preceding queued move's cycle.
    setup(); tiny(); move(1);
    arc.gm.target[0]=arc.position[0]=1;
    arc.center_0+=1;
    assert(cm_arc_callback()==STAT_OK);
    assert(mp_has_runnable_buffer());
    assert(cm.cycle_state==CYCLE_MACHINING);
    // A previous block may have ended the cycle while this arc was being generated.
    setup(); tiny(); arc.center_0+=1;
    cm.cycle_state=CYCLE_OFF; cm.machine_state=MACHINE_PROGRAM_STOP;
    assert(cm_arc_callback()==STAT_OK);
    assert(mp_has_runnable_buffer());
    assert(cm.cycle_state==CYCLE_MACHINING);
}
'''
        with tempfile.TemporaryDirectory() as d:
            c=Path(d)/'arc.c'; c.write_text(pre); exe=Path(d)/'arc'
            subprocess.run(['cc','-std=gnu99','-O1','-g','-funsigned-char','-fsanitize=address,undefined','-fstrict-flex-arrays=3',
                            '-D_POSIX_C_SOURCE=200809L','-include','tgmath.h','-I'+str(FIX),'-I'+str(FW),str(c),
                            *[str(FW/f) for f in ['planner.c','plan_line.c','plan_zoid.c','util.c']],'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
