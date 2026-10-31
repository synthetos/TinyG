"""Exercise the actual arc calculator without motion hardware."""
from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = Path(os.environ.get('TINYG_FIRMWARE', ROOT / 'firmware/tinyg'))


class FullCircle(unittest.TestCase):
    def test_arc_geometry(self):
        source = (FIRMWARE / 'plan_arc.c').read_text()
        signatures = [r'static void _compute_arc_offsets_from_radius\(\)\s*\{',
                      r'static float _estimate_arc_time \(float arc_time\)\s*\{',
                      r'static stat_t _compute_arc\(const bool radius_f\)\s*\{']
        # Include the opening brace in the exact signature only to distinguish the
        # definitions from the forward declarations, then let the extractor count it.
        functions = []
        for signature in signatures:
            match = re.search(signature, source)
            assert match, signature
            functions.append(function(source[match.start():], signature[:-5]))
        code = r'''
#include <stdlib.h>
#include <assert.h>
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "planner.h"
#include "plan_arc.h"
#include "util.h"
cmSingleton_t cm;
arc_t arc;
static int alarms;
stat_t cm_alarm_P(const stat_t status, const char *message) { alarms++; return status; }
'''
        code += '\n'.join(functions)
        code += r'''
static void setup(int plane, int ccw) {
    memset(&cm, 0, sizeof(cm)); memset(&arc, 0, sizeof(arc)); alarms=0;
    arc.plane_axis_0 = plane == 2 ? AXIS_Y : AXIS_X;
    arc.plane_axis_1 = plane == 0 ? AXIS_Y : AXIS_Z;
    arc.linear_axis = plane == 0 ? AXIS_Z : plane == 1 ? AXIS_Y : AXIS_X;
    arc.gm.motion_mode = ccw ? MOTION_MODE_CCW_ARC : MOTION_MODE_CW_ARC;
    arc.gm.feed_rate_mode = UNITS_PER_MINUTE_MODE;
    cm.gm.feed_rate = 120;
    cm.chordal_tolerance = .01;
    for(int a=0; a<AXES; a++) cm.a[a].feedrate_max=1000;
    arc.offset[arc.plane_axis_0] = -1;
}
static void full(int plane, int ccw, int implicit, int turns, float noise, float rise) {
    setup(plane,ccw);
    arc.position[arc.plane_axis_0] = 2;
    arc.gm.target[arc.plane_axis_0] = 2 + noise;
    arc.gm.target[arc.plane_axis_1] = noise;
    arc.gm.target[arc.linear_axis] = rise;
    arc.full_circle = implicit;
    arc.rotations = turns;
    assert(_compute_arc(false) == STAT_OK);
    assert(alarms==0);
    float angle = (ccw ? -1 : 1) * 2*M_PI * (implicit ? turns : 1+turns);
    assert(fabs(arc.angular_travel-angle)<.00001);
    assert(fabs(arc.linear_travel-rise)<.00001);
    assert(fabs(arc.length-hypotf(angle,rise))<.0001);
    assert(arc.segment_count>1);
    assert(isfinite(arc.segment_theta));
    assert(fabs(arc.center_0-1)<.00001);
    assert(fabs(arc.center_1)<.00001);
}
int main(void) {
    for(int plane=0;plane<3;plane++) for(int ccw=0;ccw<2;ccw++) {
        for(int turns=0;turns<3;turns++) {
            full(plane,ccw,0,turns,0,0);
            full(plane,ccw,0,turns,0,.2);
            full(plane,ccw,0,turns,.000002,0);
            full(plane,ccw,0,turns,-.000002,0);
        }
        full(plane,ccw,1,1,0,0); // omitted in-plane endpoints retain their existing convention
        full(plane,ccw,1,2,0,.2);
        // Normal quarter arcs retain their direction and length.
        setup(plane,ccw);
        arc.gm.target[arc.plane_axis_0]=-1;
        arc.gm.target[arc.plane_axis_1]=ccw ? 1 : -1;
        assert(_compute_arc(false)==STAT_OK);
        assert(fabs(arc.angular_travel-(ccw ? -M_PI/2 : M_PI/2))<.00001);
        // Small angular travel on a large radius must not become a full circle.
        setup(plane,ccw);
        arc.offset[arc.plane_axis_0]=-10000;
        arc.gm.target[arc.plane_axis_1]=ccw ? .01 : -.01;
        assert(_compute_arc(false)==STAT_ARC_ENDPOINT_IS_STARTING_POINT);
        assert(fabs(arc.angular_travel)<.00001); // retain the existing tiny-arc rejection
        setup(plane,ccw);
        arc.offset[arc.plane_axis_0]=-10000;
        arc.gm.target[arc.plane_axis_1]=ccw ? .2 : -.2;
        assert(_compute_arc(false)==STAT_OK);
        assert(fabs(arc.angular_travel)<.0001);
        assert(arc.length>.19 && arc.length<.21);
        // Radius arcs and impossible centers still use their original validation.
        setup(plane,ccw);
        arc.radius=1;
        arc.gm.target[arc.plane_axis_0]=-1;
        arc.gm.target[arc.plane_axis_1]=ccw ? 1 : -1;
        assert(_compute_arc(true)==STAT_OK);
        assert(fabs(arc.angular_travel-(ccw ? -M_PI/2 : M_PI/2))<.00001);
        setup(plane,ccw);
        arc.gm.target[arc.plane_axis_0]=10;
        assert(_compute_arc(false)==STAT_ARC_HAS_IMPOSSIBLE_CENTER_POINT);
        assert(alarms==1);
    }
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            (tmp / 'avr').mkdir()
            (tmp / 'avr/pgmspace.h').write_text('#define PROGMEM\n#define PSTR(s) (s)\n')
            c = tmp / 'arcs.c'
            c.write_text(code)
            exe = tmp / 'arcs'
            subprocess.run(['cc', '-std=gnu99', '-O1', '-g', '-fsanitize=address,undefined',
                            '-D_POSIX_C_SOURCE=200809L', '-Wno-macro-redefined', '-I'+str(tmp), '-I'+str(FIRMWARE),
                            str(c), '-lm', '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
