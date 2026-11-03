"""N06: axis rate caps must permit a finite move time before they are stored.

TinyG-Configuration-for-Firmware-Version-0.97: xVM/xFR are rates, xAM=0 disables
an axis. Zero is not a substitute for disabling an axis.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT = Path(__file__).resolve().parents[2] / 'firmware/tinyg'

class RateLimits(unittest.TestCase):
    def test_actual_table_setters(self):
        app = (ROOT/'config_app.c').read_text()
        cm = (ROOT/'canonical_machine.c').read_text()
        rows = []
        for axis in 'xyzabc':
            for suffix in ('vm', 'fr'):
                row = re.search(r'\{ "'+axis+r'","'+axis+suffix+r'",[^\n]+', app).group()
                rows.append(row.split(',')[6].strip())
        code = r'''
#include <assert.h>
#include <math.h>
#include <float.h>
#include <stdint.h>
#define STAT_OK 0
#define STAT_INPUT_LESS_THAN_MIN_VALUE 108
#define STAT_INPUT_VALUE_RANGE_ERROR 110
#define TYPE_FLOAT 4
#define MODEL 0
#define INCHES 1
#define MM_PER_INCH 25.4f
typedef int stat_t;
typedef struct {float value_flt; int index,precision,valuetype;} nvObj_t;
static float stored;
static int units;
#define target (&stored)
#define precision 0
#define GET_TABLE_WORD(x) (x)
static int cm_get_units_mode(int which) { return units; }
static int _get_axis_type(int index) { return index / 2 >= 3; }
'''.replace('#define precision 0', '#define table_precision 0').replace('#define GET_TABLE_WORD(x) (x)', '#define GET_TABLE_WORD(x) table_##x\n#define table_target (&stored)')
        code += function((ROOT/'config.c').read_text(), r'stat_t set_flt\(nvObj_t \*nv\)')
        code += function(app, r'stat_t set_flu\(nvObj_t \*nv\)')
        if 'stat_t cm_set_axis_rate(' in cm:
            code += function(cm, r'stat_t cm_set_axis_rate\(nvObj_t \*nv\)')
        code += '\nstatic stat_t (*setters[])(nvObj_t*) = {' + ','.join(rows) + '};\n'
        code += r'''
int main(void) {
    float bad[] = {0,-0.0f,-1,NAN,INFINITY,-INFINITY};
    for(int axis=0;axis<12;axis++) for(units=0;units<2;units++) {
        for(int b=0;b<6;b++) {
            stored=123; nvObj_t nv={.value_flt=bad[b],.index=axis};
            assert(setters[axis](&nv)!=STAT_OK); assert(stored==123);
        }
        nvObj_t nv={.value_flt=100,.index=axis};
        assert(setters[axis](&nv)==STAT_OK);
        assert(fabsf(stored-(units && axis<6 ? 2540 : 100))<.001f);
        if(units && axis<6) {
            stored=123; nv.value_flt=FLT_MAX;
            assert(setters[axis](&nv)==STAT_INPUT_VALUE_RANGE_ERROR); assert(stored==123);
        }
    }
}
'''
        with tempfile.TemporaryDirectory() as d:
            c=Path(d)/'rate.c'; c.write_text(code); exe=Path(d)/'rate'
            subprocess.run(['cc','-std=c99','-fsanitize=address,undefined',str(c),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
