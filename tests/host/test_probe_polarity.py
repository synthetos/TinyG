"""Probe polarity initialization and validation, independent of switch timing."""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]
PRE=r'''
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#define NUM_SWITCHES 8
#define SW_MODE_DISABLED 0
#define SW_MODE_PROBE 4
#define SW_DISABLED -1
#define SW_EDGE_NONE -1
#define SW_ACTIVE_LO 0
#define SW_LOCKOUT_MS 50
#define CYCLE_OFF 0
#define PREP_BUFFER_OWNED_BY_LOADER 0
#define STAT_OK 0
#define STAT_INPUT_VALUE_RANGE_ERROR 146
#define STAT_COMMAND_NOT_ACCEPTED 106
#define __AVR 1
uint8_t SREG=128;
#define cli() (SREG=0)
typedef int stat_t;
typedef struct {uint32_t value_int;} nvObj_t;
struct {int cycle_state;} cm;
struct {int buffer_state;} st_pre;
struct {uint8_t switch_type,probe_type;struct {int mode,type,state,edge,lockout_ms,timeout;} s[8];} sw;
static bool queued,busy;
bool mp_has_runnable_buffer(void) {return queued;}
bool mp_runtime_is_idle(void) {return !busy;}
void Timeout_clear(int *p) {*p=0;}
static bool _read_raw_switch(uint8_t i) {return true ^ !sw.s[i].type;}
void reset_switches(void);
void switch_init(void) {reset_switches();}
'''
MAIN=r'''
int main(void) {
 st_pre.buffer_state=1;sw.switch_type=1;
 for(int i=0;i<8;i++)sw.s[i].mode=1;
 sw.s[0].mode=SW_MODE_PROBE;sw.s[7].mode=SW_MODE_DISABLED;
 nvObj_t n;
 for(unsigned p=0;p<2;p++) {
  n.value_int=p;assert(sw_set_prbp(&n)==0 && SREG==128);
  assert(sw.s[0].type==(int)p && sw.s[0].state==(int)p);
  assert(sw.s[1].type==1 && sw.s[1].state==1 && sw.s[7].state==SW_DISABLED);
 }
 for(unsigned v=2;v<4;v++) {n.value_int=v;assert(sw_set_prbp(&n)==146 && sw.probe_type==1);}
 n.value_int=UINT32_MAX;assert(sw_set_prbp(&n)==146);
 n.value_int=0;
 queued=true;assert(sw_set_prbp(&n)==106 && SREG==128);queued=false;
 busy=true;assert(sw_set_prbp(&n)==106);busy=false;
 st_pre.buffer_state=0;assert(sw_set_prbp(&n)==106);st_pre.buffer_state=1;
 cm.cycle_state=1;assert(sw_set_prbp(&n)==106);cm.cycle_state=0;
 SREG=0;assert(sw_set_prbp(&n)==0 && SREG==0);
 return 0;
}
'''
class ProbePolarity(unittest.TestCase):
 def test_reset_and_setter(self):
  s=(ROOT/'firmware/tinyg/switch.c').read_text();code=PRE
  for sig in [r'void reset_switches\([^;{}]*\)(?=\s*\{)',r'stat_t sw_set_prbp\([^;{}]*\)(?=\s*\{)']:
   code+=function(s,sig)+'\n'
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'check.c';exe=Path(d)/'check';p.write_text(code+MAIN)
   subprocess.run(['cc','-fsanitize=address,undefined',str(p),'-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
