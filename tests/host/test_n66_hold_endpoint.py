"""N66: a resumed block must use its planned endpoint despite an inherited fractional runtime offset."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]


class HoldEndpoint(unittest.TestCase):
    def test_final_waypoint_ignores_inherited_runtime_roundoff(self):
        cc = shutil.which('cc')
        if not cc:
            self.skipTest('host C compiler required')
        text = function((ROOT / 'firmware/tinyg/plan_exec.c').read_text(), r'stat_t mp_exec_aline\(mpBuf_t \*bf\)')
        # Execute the shipped waypoint initialization, including any final-endpoint correction.
        snippet = text[text.index('// generate the waypoints'):text.index('// Feedhold Processing')]
        snippet = snippet[:snippet.rfind('}')]
        code = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#define AXES 6
#define SECTION_HEAD 0
#define SECTION_BODY 1
#define SECTION_TAIL 2
#define fp_ZERO(x) (fabsf(x)<0.00001f)
#define copy_vector(d,s) memcpy((d),(s),sizeof(float)*AXES)
static struct {float position[AXES],unit[AXES],target[AXES],waypoint[3][AXES];
               float head_length,body_length,tail_length;} mr;
static void initialize(void) {
''' + snippet + r'''
}
int main(void) {
 for(int last=0;last<3;last++) {
  memset(&mr,0,sizeof(mr));
  /* Return leg of a long rapid after holds; model target is exactly zero,
     runtime carries a sub-millimetre integration error (~1.21 X steps at1280/mm). */
  mr.position[0]=-203.120941162109375f; mr.unit[0]=1;
  if(last==0)mr.head_length=203.12f;
  if(last==1)mr.body_length=203.12f;
  if(last==2)mr.tail_length=203.12f;
  initialize();
  assert(mr.waypoint[last][0]==mr.target[0]);
 }
}
'''
        with tempfile.TemporaryDirectory(prefix='tinyg-n66-') as tmp:
            src, exe = Path(tmp)/'check.c', Path(tmp)/'check'
            src.write_text(code)
            subprocess.run([cc,'-std=c99','-Wall','-Werror','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(src),'-lm','-o',str(exe)],check=True)
            result=subprocess.run([str(exe)],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
