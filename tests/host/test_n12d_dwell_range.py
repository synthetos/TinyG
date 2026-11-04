"""N12d: reject dwells outside the uint32 microsecond path before queueing.

NIST RS274NGC v3 3.5.4 and the TinyG Gcode-Support wiki define G4 P in seconds.
The implementation uses a uint32 microsecond conversion: 4294 seconds is safe,
4295 is not. Zero remains legal; negative P keeps its existing specific status.
"""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]/'firmware/tinyg'
class DwellRange(unittest.TestCase):
 def test_dwell_range_before_queue(self):
  code=r'''
#include <assert.h>
#include <math.h>
#include <string.h>
typedef int stat_t;
#define STAT_OK 0
#define STAT_P_WORD_IS_NEGATIVE 155
#define STAT_INPUT_VALUE_RANGE_ERROR 110
#define PSTR(s) s
static int queued;static float value;static const char *message;
static int mp_dwell(float s){queued++;value=s;return 0;}
static void nv_add_message_P(const char*s){message=s;}
'''
  code+=function((ROOT/'canonical_machine.c').read_text(),r'stat_t cm_dwell\(')
  code+=r'''
int main(void){
 float bad[]={4295,1e20f,INFINITY,NAN};
 for(int n=0;n<4;n++){queued=0;message=0;assert(cm_dwell(bad[n])==110);assert(!queued);assert(message&&strstr(message,"4294"));}
 assert(cm_dwell(-1)==STAT_P_WORD_IS_NEGATIVE && !queued);
 assert(cm_dwell(0)==0 && value==0);assert(cm_dwell(4294)==0 && value==4294);
 assert(value*1000000.0f < 4294967296.0f);
}
'''
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'dwell.c';p.write_text(code);exe=p.with_suffix('')
   subprocess.run(['cc','-std=c99',str(p),'-lm','-o',str(exe)],check=True)
   r=subprocess.run([str(exe)],capture_output=True,text=True);self.assertEqual(r.returncode,0,r.stderr)
if __name__=='__main__':unittest.main()
