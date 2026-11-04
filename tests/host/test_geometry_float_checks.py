"""Binary32 geometry predicates agree with the arithmetic checks, including signed zero."""
from pathlib import Path
import subprocess, tempfile, unittest
from source_extract import function
FW = Path(__file__).resolve().parents[2] / 'firmware/tinyg'
class GeometryChecks(unittest.TestCase):
 def test_boundaries_and_bit_patterns(self):
  src=(FW/'util.h').read_text()
  code='#include <stdint.h>\n#include <stdbool.h>\n#include <math.h>\n#include <assert.h>\n#define __AVR 1\n'
  for sig in ['uint32_t fp_bits','bool fp_finite','bool fp_positive_finite','bool fp_nonnegative_finite','bool fp_bounded','bool fp_nonnegative_finite_at','bool fp_positive_finite_at','bool fp_bounded_at']:
   code+=function(src,r'static inline '+sig+r'\(')
  code+=r'''
static void check(uint32_t bits) {
 union {uint32_t u;float f;} x={.u=bits};float f=x.f;
 assert(fp_finite(f)==!!isfinite(f));
 assert(fp_positive_finite_at(&f)==(isfinite(f)&&f>0));
 assert(fp_nonnegative_finite_at(&f)==(isfinite(f)&&f>=0));
 assert(fp_bounded_at(&f,1e18f)==(isfinite(f)&&fabsf(f)<=1e18f));
}
int main(void) {
 uint32_t edges[]={0,0x80000000,1,0x80000001,0x007fffff,0x00800000,
 0x7f7fffff,0xff7fffff,0x7f800000,0xff800000,0x7fc00000,0xffc00000};
 for(unsigned i=0;i<sizeof edges/sizeof *edges;i++)check(edges[i]);
 union {float f;uint32_t u;} bound={.f=1e18f};
 for(int i=-2;i<=2;i++){check(bound.u+i);check((bound.u+i)|0x80000000);}
 uint32_t state=0x381a5291;
 for(unsigned i=0;i<200000;i++){state^=state<<13;state^=state>>17;state^=state<<5;check(state);}
}
'''
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'check.c';p.write_text(code)
   subprocess.run(['cc','-O2','-fsanitize=undefined,address',str(p),'-o',str(p.with_suffix(''))],check=True)
   subprocess.run([str(p.with_suffix(''))],check=True)
