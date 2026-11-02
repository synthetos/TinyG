"""Exercise actual move-time and feed lifecycle code without motion hardware."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]
FW=ROOT/'firmware/tinyg'

class InverseTime(unittest.TestCase):
    def test_model_and_arc_segment_time_survive_planning(self):
        line=(FW/'plan_line.c').read_text()
        machine=(FW/'canonical_machine.c').read_text()
        parser=(FW/'gcode_parser.c').read_text()
        prefix=parser[parser.index('stat_t status = STAT_OK;',parser.index('static stat_t _execute_gcode_block()\n{')):]
        prefix=prefix[:prefix.index('EXEC_FUNC(cm_feed_rate_override_factor')]
        feed_guard=function(machine,r'stat_t cm_straight_feed\(')
        feed_guard=feed_guard[feed_guard.index('{')+1:feed_guard.index('cm_set_model_target')]
        code=r'''
#include <assert.h>
#include <math.h>
#include <string.h>
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "util.h"
#include "feedlimit.h"
cmSingleton_t cm;
stat_t status_code;
void cm_set_model_linenum(uint32_t n) {}
#define EXEC_FUNC(f,v) if(cm.gf.v) {status=f(cm.gn.v);}
'''
        code+=function((FW/'util.c').read_text(),r'float max4\(')+'\n'
        code+=function((FW/'util.c').read_text(),r'float max3\(')+'\n'
        for src,sig in [(line,r'static (?:void|bool) _calc_move_times\([^;]*?\)\s*(?://[^\n]*)?\s*(?=\{)'),(machine,r'void cm_finalize_move\('),(machine,r'stat_t cm_set_feed_rate\('),(machine,r'stat_t cm_set_feed_rate_mode\(')]:code+=function(src,sig)+'\n'
        code+='static stat_t begin_block(void) {\n'+prefix+'return status;}\n'
        code+='static bool requested_axes[AXES]={true};\nstatic stat_t check_feed(void) {\nconst bool *flags=requested_axes;\n'+feed_guard+'return STAT_OK;}\n'
        code+=r'''
int main(void) {
    memset(&cm,0,sizeof(cm));
    cm.gm.units_mode=MILLIMETERS;
    for(int i=0;i<AXES;i++){cm.a[i].velocity_max=600;cm.a[i].feedrate_max=600;}
    float length[AXES]={1,0,0,0,0,0}, square[AXES]={1,0,0,0,0,0};
    cm.gn.motion_mode=MOTION_MODE_STRAIGHT_FEED;
    cm.gf.feed_rate_mode=true;cm.gn.feed_rate_mode=INVERSE_TIME_MODE;
    cm.gf.feed_rate=true;cm.gn.feed_rate=60;
    assert(begin_block()==STAT_OK);
    cm.gm.motion_mode=MOTION_MODE_STRAIGHT_FEED;
    assert(check_feed()==STAT_OK);
    _calc_move_times(&cm.gm,length,square);
    assert(cm.gm.feed_rate_mode==INVERSE_TIME_MODE);
    assert(fabs(cm.gm.move_time-1.0/60)<1e-7);
    cm_finalize_move();assert(cm.gm.feed_rate==0);
    cm.gf.feed_rate=false;cm.gf.feed_rate_mode=false;
    assert(begin_block()==STAT_OK);assert(check_feed()==STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
    // A standalone F cannot supply a subsequent inverse-time motion block.
    cm.gf.feed_rate=true;cm.gn.feed_rate=60;assert(begin_block()==STAT_OK);
    cm.gf.feed_rate=false;assert(begin_block()==STAT_OK);assert(check_feed()!=STAT_OK);
    // Each arc segment keeps its own inverse-time fraction; finalizing the
    // model cannot clear the already copied arc state.
    for(int mode=MOTION_MODE_CW_ARC;mode<=MOTION_MODE_CCW_ARC;mode++) {
        cm.gm.feed_rate_mode=INVERSE_TIME_MODE;cm.gm.feed_rate=1.0/60;cm.gm.motion_mode=mode;
        GCodeState_t arc=cm.gm;arc.feed_rate/=10;
        cm_finalize_move();assert(cm.gm.feed_rate==0);
        length[0]=.01;square[0]=.0001;
        for(int i=0;i<10;i++) {_calc_move_times(&arc,length,square);assert(arc.feed_rate_mode==INVERSE_TIME_MODE);assert(fabs(arc.move_time-1.0/600)<1e-8);}
    }
    // Axis-free modal/program commands dispatch through the current G1 mode.
    // They must not require F after a G93 move cleared its per-block feed.
    memset(requested_axes,0,sizeof(requested_axes));
    for(int mode=INVERSE_TIME_MODE;mode<=UNITS_PER_MINUTE_MODE;mode++) {
        cm.gm.feed_rate_mode=mode;cm.gm.feed_rate=0;
        assert(check_feed()==STAT_OK);
    }
    requested_axes[0]=true;
    // G94 feed remains modal, while a zero inverse feed is rejected.
    cm.gf.feed_rate_mode=true;cm.gn.feed_rate_mode=UNITS_PER_MINUTE_MODE;
    cm.gf.feed_rate=true;cm.gn.feed_rate=120;assert(begin_block()==STAT_OK);
    cm.gf.feed_rate_mode=false;cm.gf.feed_rate=false;assert(begin_block()==STAT_OK);
    cm.gm.motion_mode=MOTION_MODE_STRAIGHT_FEED;cm_finalize_move();assert(cm.gm.feed_rate==120);
    cm.gm.feed_rate_mode=INVERSE_TIME_MODE;
    assert(cm_set_feed_rate(0)==STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
    assert(cm_set_feed_rate(1e8)==STAT_OK);assert(check_feed()==STAT_OK);
    assert(cm_set_feed_rate(1e-40f)==STAT_GCODE_FEEDRATE_NOT_SPECIFIED);
    cm.gn.motion_mode=MOTION_MODE_STRAIGHT_TRAVERSE;cm.gf.feed_rate=true;cm.gn.feed_rate=0;
    assert(begin_block()==STAT_OK);assert(cm.gm.feed_rate==0);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as td:
            p=Path(td);(p/'avr').mkdir();(p/'avr/pgmspace.h').write_text('#define PROGMEM\n#define PSTR(s) (s)\n')
            (p/'check.c').write_text(code)
            subprocess.run(['cc','-std=gnu99','-O1','-g','-fsanitize=address,undefined','-D_POSIX_C_SOURCE=200809L','-Wno-macro-redefined','-I'+str(p),'-I'+str(FW),str(p/'check.c'),'-o',str(p/'check')],check=True)
            result=subprocess.run([str(p/'check')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
