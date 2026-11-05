"""Run the actual transient jerk setter and reciprocal-cache update without hardware."""
from pathlib import Path
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]


class TransientJerk(unittest.TestCase):
    def test_units_bounds_and_unchanged_axes(self):
        source = (ROOT / 'firmware/tinyg/canonical_machine.c').read_text()
        code = r'''
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <float.h>
#define AXES 6
#define AXIS_X 0
#define AXIS_Z 2
#define INCHES 1
#define MM_PER_INCH 25.4f
#define JERK_MULTIPLIER 1000000.f
#define STAT_OK 0
#define STAT_INPUT_VALUE_RANGE_ERROR 110
#define max(a,b) ((a)>(b)?(a):(b))
typedef int stat_t;
struct {struct {int units_mode;} gm; struct {float jerk_max,recip_jerk;} a[AXES];} cm;
'''
        code += function(source, r'void cm_set_axis_jerk\(const uint8_t axis, const float jerk\)')
        code += function(source, r'stat_t cm_set_jerk\(const float target\[\], const bool flags\[\]\)')
        code += r'''
int main(void) {
    float values[AXES]={1000,0,500,1000,2000,-1};
    bool flags[AXES]={true,true,false,true,true,true};
    cm_set_axis_jerk(2,77);
    assert(cm_set_jerk(values,flags)==STAT_OK);
    assert(fabsf(cm.a[0].jerk_max-216)<.001f && cm.a[1].jerk_max==1);
    assert(cm.a[2].jerk_max==77 && cm.a[5].jerk_max==1);
    assert(fabsf(cm.a[0].recip_jerk-1.f/216000000)<1e-14f);
    cm.gm.units_mode=INCHES;
    assert(cm_set_jerk(values,flags)==STAT_OK);
    assert(fabsf(cm.a[0].jerk_max-5486.4f)<.01f);
    assert(fabsf(cm.a[3].jerk_max-216)<.001f && fabsf(cm.a[4].jerk_max-432)<.001f);
    assert(cm.a[2].jerk_max==77);
    char before[sizeof(cm.a)]; memcpy(before,cm.a,sizeof(cm.a));
    float invalid[]={NAN,INFINITY,-INFINITY,FLT_MAX};
    for(int i=0;i<4;i++) {
        values[0]=2000; values[5]=invalid[i];
        assert(cm_set_jerk(values,flags)==STAT_INPUT_VALUE_RANGE_ERROR);
        assert(!memcmp(before,cm.a,sizeof(cm.a)));
    }
    // Invalid values on unmentioned axes are irrelevant.
    memset(flags,0,sizeof(flags));
    assert(cm_set_jerk(values,flags)==STAT_OK);
    assert(!memcmp(before,cm.a,sizeof(cm.a)));
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / 'jerk.c'
            c.write_text(code)
            exe = Path(tmp) / 'jerk'
            subprocess.run(['cc', '-std=c99', '-Wall', '-Werror', '-fsanitize=address,undefined','-fstrict-flex-arrays=3',
                            str(c), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
