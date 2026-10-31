"""Invalid global switch types must not reinitialize inputs or reach persistence."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT=Path(__file__).resolve().parents[2]

class SwitchTypeValidation(unittest.TestCase):
    def test_rejected_type_does_not_reinitialize(self):
        config=(ROOT/'firmware/tinyg/config.c').read_text()
        source=(ROOT/'firmware/tinyg/switch.c').read_text()
        code=r'''
#include <assert.h>
#include <stdint.h>
#define STAT_OK 0
#define STAT_INPUT_VALUE_RANGE_ERROR 109
#define ritorno(x) do {int s=(x);if(s)return s;}while(0)
typedef int stat_t;
typedef struct {uint32_t value_int;} nvObj_t;
uint8_t switch_type;unsigned init_calls;
#define GET_TABLE_WORD(field) (&switch_type)
void switch_init(void){init_calls++;}
'''
        for name in ('set_ui8','set_01'):
            code+=function(config,r'stat_t '+name+r'\(')
        code+=function(source,r'stat_t sw_set_st\(')
        code+=r'''
int main(void){
 nvObj_t nv;
 for(unsigned initial=0;initial<2;initial++) {
  uint32_t invalid[]={2,255,256,UINT32_MAX};
  for(unsigned i=0;i<4;i++) {
   switch_type=initial;init_calls=0;nv.value_int=invalid[i];
   assert(sw_set_st(&nv)==STAT_INPUT_VALUE_RANGE_ERROR);
   assert(switch_type==initial && init_calls==0);
  }
 }
 for(unsigned value=0;value<2;value++) {
  init_calls=0;nv.value_int=value;
  assert(sw_set_st(&nv)==STAT_OK);
  assert(switch_type==value && init_calls==1);
 }
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'switch.c';exe=Path(tmp)/'switch';p.write_text(code)
            subprocess.run(['cc','-std=c99','-Wall','-Werror','-fsanitize=address,undefined',str(p),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
