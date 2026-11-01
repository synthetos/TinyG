"""Check G10 offsets and queued display snapshots using the actual C functions."""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = Path(os.environ.get('TINYG_FIRMWARE', ROOT / 'firmware/tinyg'))


class CoordinateOffsets(unittest.TestCase):
    def test_model_and_runtime(self):
        source = (FIRMWARE / 'canonical_machine.c').read_text()
        names = ['cm_get_active_coord_offset', 'cm_set_work_offsets', 'cm_get_work_position',
                 'cm_set_coord_offsets']
        if 'static float _get_work_position_mm(' in source:
            names.insert(2, '_get_work_position_mm')
        if 'static void _exec_g10_offset(' in source:
            names.append('_exec_g10_offset')
        definitions = []
        for name in names:
            sig = r'(?:static )?(?:float|void|stat_t) ' + name + r'\([^;{}]*\)\s*\{'
            start = re.search(sig, source)
            assert start, name
            definitions.append(function(source[start.start():], sig[:-5]))
        code = r'''
#include <stdlib.h>
#include <assert.h>
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "planner.h"
#include "util.h"
cmSingleton_t cm;
bool FLAGS_NONE[AXES];
static void _exec_g10_offset(float *, bool *);
static float runtime_offset[AXES];
static GCodeState_t runtime;
static struct { void(*fn)(float[],bool[]); float value[AXES]; bool flags[AXES]; } queued[8];
static int count;
void mp_queue_command(void(*fn)(float[],bool[]), float *value, bool *flags) {
    assert(count<8); queued[count].fn=fn;
    memcpy(queued[count].value,value,sizeof(queued[count].value));
    memcpy(queued[count].flags,flags,sizeof(queued[count].flags)); count++;
}
void mp_set_runtime_work_offset(float offset[]) { memcpy(runtime_offset,offset,sizeof(runtime_offset)); }
float mp_get_runtime_work_position(const uint8_t axis) { return cm.gmx.position[axis]-runtime_offset[axis]; }
'''
        code += '\n'.join(definitions)
        code += r'''
static void setup(int units, int origin) {
    memset(&cm,0,sizeof(cm)); memset(&runtime,0,sizeof(runtime)); count=0;
    memset(queued,0,sizeof(queued)); memset(runtime_offset,0,sizeof(runtime_offset));
    cm.gm.coord_system=G55; cm.gm.units_mode=units; runtime.units_mode=units;
    cm.gf.L_word=true; cm.gf.parameter=true; cm.gmx.origin_offset_enable=origin;
    for(int a=0;a<AXES;a++) {
        cm.gmx.position[a]=7.5+a;
        cm.offset[G55][a]=-2-a;
        cm.offset[G54][a]=20+a;
        cm.gmx.origin_offset[a]=3.25+a;
        cm.gmx.tool_offset[a]=1.75+a;
    }
    cm_set_work_offsets();
}
static void close_to(float a,float b) { assert(fabs(a-b)<.0001); }
int main(void) {
    float value[AXES]={0}; bool flags[AXES]={0};
    for(int units=0;units<2;units++) for(int origin=0;origin<2;origin++)
    for(int axis=0;axis<AXES;axis++) for(int inactive=0;inactive<2;inactive++) {
        setup(units,origin); memset(flags,0,sizeof(flags)); flags[axis]=true;
        value[axis]=1.25;
        float previous=cm_get_work_position(MODEL,axis);
        float untouched=cm.offset[G55][(axis+1)%AXES];
        assert(cm_set_coord_offsets(inactive ? G54 : 0,20,value,flags)==STAT_OK);
        assert(cm.deferred_write_flag);
        close_to(cm.offset[G55][(axis+1)%AXES],untouched);
        if(inactive) {
            assert(count==0);
            close_to(cm_get_work_position(MODEL,axis),previous);
            cm.gm.coord_system=G54; cm_set_work_offsets();
            close_to(cm_get_work_position(MODEL,axis),value[axis]);
        } else {
            assert(count==1);
            close_to(cm_get_work_position(MODEL,axis),value[axis]);
            queued[0].fn(queued[0].value,queued[0].flags);
            close_to(cm_get_work_position(&runtime,axis),value[axis]);
        }
        // L2 remains an absolute coordinate-offset assignment; ABC is always degrees.
        setup(units,origin);
        assert(cm_set_coord_offsets(G55,2,value,flags)==STAT_OK);
        close_to(cm.offset[G55][axis],(units==INCHES && axis<AXIS_A) ? 31.75 : 1.25);
    }
    // Later model edits must not overwrite earlier queued display snapshots.
    setup(MILLIMETERS,true); memset(flags,0,sizeof(flags)); flags[AXIS_X]=true;
    value[AXIS_X]=2;
    assert(cm_set_coord_offsets(0,20,value,flags)==STAT_OK);
    value[AXIS_X]=4;
    assert(cm_set_coord_offsets(0,20,value,flags)==STAT_OK);
    assert(count==2);
    queued[0].fn(queued[0].value,queued[0].flags);
    close_to(cm_get_work_position(&runtime,AXIS_X),2);
    close_to(cm_get_work_position(MODEL,AXIS_X),4);
    queued[1].fn(queued[1].value,queued[1].flags);
    close_to(cm_get_work_position(&runtime,AXIS_X),4);
    // Validation and no-axis commands do not mutate offsets or enqueue work.
    setup(MILLIMETERS,true);
    float before[sizeof(cm.offset)/sizeof(float)]; memcpy(before,cm.offset,sizeof(before));
    assert(cm_set_coord_offsets(COORD_SYSTEM_MAX+1,20,value,flags)==STAT_P_WORD_IS_INVALID);
    float invalid[]={-1, .5, 256, NAN, INFINITY};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++)
        assert(cm_set_coord_offsets(invalid[i],20,value,flags)==STAT_P_WORD_IS_INVALID);
    cm.gf.parameter=false;
    assert(cm_set_coord_offsets(0,20,value,flags)==STAT_P_WORD_IS_INVALID);
    cm.gf.parameter=true;
    cm.gf.L_word=false;
    assert(cm_set_coord_offsets(0,20,value,flags)==STAT_L_WORD_IS_MISSING);
    cm.gf.L_word=true;
    assert(cm_set_coord_offsets(0,3,value,flags)==STAT_L_WORD_IS_INVALID);
    memset(flags,0,sizeof(flags));
    assert(cm_set_coord_offsets(0,20,value,flags)==STAT_OK);
    assert(!memcmp(before,cm.offset,sizeof(before)) && count==0 && !cm.deferred_write_flag);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            (tmp/'avr').mkdir()
            (tmp/'avr/pgmspace.h').write_text('#define PROGMEM\n#define PSTR(s) (s)\n')
            c=tmp/'offsets.c'; c.write_text(code)
            exe=tmp/'offsets'
            subprocess.run(['cc','-std=gnu99','-O1','-g','-fsanitize=address,undefined',
                            '-D_POSIX_C_SOURCE=200809L','-Wno-macro-redefined',
                            '-I'+str(tmp),'-I'+str(FIRMWARE),str(c),'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)


if __name__ == '__main__':
    unittest.main()
