"""N12c: bound G93 duration before it reaches the uint32 segment counter.

NIST RS274NGC v3 section 3.5.19: G93 F means a move duration of 1/F minutes.
A finite F can still require more segments than the implementation can count.
"""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]/'firmware/tinyg'
class InverseRange(unittest.TestCase):
 def test_feed_range_before_storage(self):
  code=r'''
#include <assert.h>
#include <math.h>
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "planner.h"
#include "util.h"
cmSingleton_t cm;
#ifndef MAX_INVERSE_TIME_MINUTES
#define MAX_INVERSE_TIME_MINUTES (2147483520.0f * NOM_SEGMENT_TIME)
#endif
'''
  code+=function((ROOT/'canonical_machine.c').read_text(),r'stat_t cm_set_feed_rate\(')
  code+=r'''
int main(void){
 cm.gm.feed_rate_mode=INVERSE_TIME_MODE;cm.gm.feed_rate=12;
 assert(cm_set_feed_rate(.0000001f)==STAT_INPUT_VALUE_RANGE_ERROR);assert(cm.gm.feed_rate==12);
 assert(cm_set_feed_rate(1/(MAX_INVERSE_TIME_MINUTES*1.01f))==STAT_INPUT_VALUE_RANGE_ERROR);
 assert(cm_set_feed_rate(1/(MAX_INVERSE_TIME_MINUTES*.99f))==STAT_OK);
 float segments=ceilf((cm.gm.feed_rate*60*1000000)/NOM_SEGMENT_USEC);
 assert(segments<2147483648.0f && segments>0);
 assert(cm_set_feed_rate(60)==STAT_OK);assert(fabsf(cm.gm.feed_rate-1/60.0f)<1e-7f);
 cm.gm.feed_rate_mode=UNITS_PER_MINUTE_MODE;assert(cm_set_feed_rate(100)==STAT_OK);
}
'''
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'inverse.c';p.write_text(code);exe=p.with_suffix('')
   fixture=Path(__file__).parent/'planner_startup'
   subprocess.run(['cc','-std=gnu99','-D_POSIX_C_SOURCE=200809L','-I'+str(fixture),'-I'+str(ROOT),str(p),'-lm','-o',str(exe)],check=True)
   r=subprocess.run([str(exe)],capture_output=True,text=True);self.assertEqual(r.returncode,0,r.stderr)
if __name__=='__main__':unittest.main()
