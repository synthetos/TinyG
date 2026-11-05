"""N33: reconfiguration must retain a pending edge while any motion is owned."""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
import test_probe_polarity as fixture
ROOT=Path(__file__).resolve().parents[2]
class SwitchBusy(unittest.TestCase):
 def test_setters_preserve_pending_edge(self):
  source=(ROOT/'firmware/tinyg/switch.c').read_text()
  code=fixture.PRE.replace('return true ^ !sw.s[i].type;', 'assert(SREG==0);return true ^ !sw.s[i].type;')+r'''
#define SW_MODE_MAX_VALUE 4
#define ritorno(x) do {stat_t status=(x);if(status)return status;}while(0)
static uint8_t target;
static stat_t set_ui8(nvObj_t*n){target=n->value_int;return 0;}
static stat_t set_01(nvObj_t*n){if(n->value_int>1)return STAT_INPUT_VALUE_RANGE_ERROR;return set_ui8(n);}
'''
  for name in ['reset_switches','sw_set_st','sw_set_sw']:
   code+=function(source,r'(?:void|stat_t) '+name+r'\([^;{}]*\)(?=[^;{}]*\{)')+'\n'
  code+=r'''
int main(void){
 nvObj_t n={0};sw.configured=true;st_pre.buffer_state=1;
 stat_t (*setters[])(nvObj_t*)={sw_set_st,sw_set_sw};
 for(unsigned i=0;i<2;i++)for(unsigned reason=0;reason<4;reason++){
  queued=reason==0;busy=reason==1;st_pre.buffer_state=reason==2?0:1;cm.cycle_state=reason==3?1:0;
  sw.s[0].mode=2;sw.s[0].state=0;sw.s[0].edge=0;sw.s[0].timeout=123;target=1;
  assert(setters[i](&n)==106);
  assert(sw.s[0].state==0 && sw.s[0].timeout==123 && target==1 && SREG==128);
 }
 queued=busy=false;st_pre.buffer_state=1;cm.cycle_state=0;
 for(unsigned i=0;i<2;i++){
  assert(setters[i](&n)==0 && target==0 && sw.s[0].timeout==0 && SREG==128);
  SREG=0;assert(setters[i](&n)==0 && SREG==0);SREG=128;
 }
 reset_switches();assert(SREG==128);
 sw.configured=false;queued=busy=true;cm.cycle_state=1;
 assert(sw_set_st(&n)==0 && sw_set_sw(&n)==0);
 n.value_int=5;assert(sw_set_sw(&n)==STAT_INPUT_VALUE_RANGE_ERROR);
 n.value_int=2;assert(sw_set_st(&n)==STAT_INPUT_VALUE_RANGE_ERROR);
}
'''
  with tempfile.TemporaryDirectory() as d:
   src=Path(d)/'switch.c';src.write_text(code);exe=src.with_suffix('')
   subprocess.run(['cc','-std=c99','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(src),'-o',str(exe)],check=True)
   r=subprocess.run([str(exe)],capture_output=True,text=True);self.assertEqual(r.returncode,0,r.stderr)
if __name__=='__main__':unittest.main()
