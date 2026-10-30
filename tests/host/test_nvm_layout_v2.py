"""Layout 2 accepts its own marker and rejects every pre-layout-2 marker."""
from pathlib import Path
import re, subprocess, tempfile, unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]
class NvmLayoutV2(unittest.TestCase):
 def test_marker_compatibility(self):
  source=(ROOT/'firmware/tinyg/config.c').read_text()
  defines='\n'.join(line for line in source.splitlines() if line.startswith('#define CONFIG_NVM_LAYOUT'))
  code=r'''
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <math.h>
typedef union {uint32_t value_int;float value_flt;} nvObj_t;
#define fp_EQ(a,b) (fabs((a)-(b))<.00001f)
'''+defines+'\n'+function(source,r'static bool _nvm_layout_is_current\([^;{}]*\)(?=\s*\{)')+r'''
int main(void) {
 assert(CONFIG_NVM_LAYOUT==2);
 nvObj_t nv;
 float old[]={449.01f,449.99f,10000,10001,10003,0,NAN,INFINITY,-INFINITY};
 for(unsigned i=0;i<sizeof(old)/sizeof(*old);i++) {nv.value_flt=old[i];assert(!_nvm_layout_is_current(&nv));}
 nv.value_int=UINT32_MAX;assert(!_nvm_layout_is_current(&nv));
 nv.value_flt=10002;assert(_nvm_layout_is_current(&nv));
}
'''
  with tempfile.TemporaryDirectory() as d:
   c=Path(d)/'check.c';exe=Path(d)/'check';c.write_text(code)
   subprocess.run(['cc','-Wall','-Werror','-fsanitize=address,undefined',str(c),'-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
