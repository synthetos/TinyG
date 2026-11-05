"""Bounded ADC polling and protocol formatting; no analog accuracy claim."""
from pathlib import Path
import subprocess, tempfile, unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]
class AdcReport(unittest.TestCase):
 def test_timeout_and_response(self):
  s=(ROOT/'firmware/tinyg/adc_report.c').read_text()
  names=['_adc_wait_ready','hw_get_adc3','adc_report']
  code=r'''#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define STAT_OK 0
#define STAT_COMMAND_NOT_ACCEPTED 106
#define STAT_JSON_TOO_MANY_PAIRS 112
#define TYPE_FLOAT 5
#define TEXT_MODE 0
#define ADC_CH_CHIF_bm 1
#define ADC_POLL_LIMIT 1024
#define PSTR(s) (s)
#define strcpy_P strcpy
#define fprintf_P fprintf
#define NUL 0
#define ritorno(x) do {int r=(x);if(r)return r;} while(0)
typedef int stat_t;
typedef struct {int valuetype,precision,depth; float value_flt; char group[4];} nvObj_t;
struct {int comm_mode;} cs;
static int16_t sample; static int result,used,capacity=1;
static nvObj_t object;
stat_t adc_read_raw(int16_t *out) {if(result)return result;*out=sample;return 0;}
nvObj_t *nv_add_float(const char *token,float v) {
 assert(!strcmp(token,"adc3")); if(used==capacity)return NULL;
 used++; object.value_flt=v; strcpy(object.group,"sys"); return &object;
}
'''
  for n in names:
   code+=function(s,r'(?:static )?stat_t '+n+r'\([^;{}]*\)(?=\s*\{)')+'\n'
  code+=r'''
int main(void) {
 volatile uint8_t flags=0;
 assert(_adc_wait_ready(&flags)==106);
 flags=1;assert(_adc_wait_ready(&flags)==0);
 int16_t values[]={-2048,-1,0,1,2047};
 for(int i=0;i<5;i++) {
  sample=values[i];nvObj_t nv={0};assert(hw_get_adc3(&nv)==0);
  assert(nv.valuetype==TYPE_FLOAT && nv.precision==0 && nv.value_flt==sample);
  cs.comm_mode=1;used=0;assert(adc_report()==0 && used==1);
  assert(object.value_flt==sample && object.depth==1 && object.precision==0 && object.group[0]==0);
 }
 capacity=0;used=0;assert(adc_report()==112);
 result=106;assert(adc_report()==106 && used==0);
 nvObj_t nv={.value_flt=123};assert(hw_get_adc3(&nv)==106 && nv.value_flt==123);
 result=0;cs.comm_mode=TEXT_MODE;sample=-3;assert(adc_report()==0 && used==0);
}
'''
  with tempfile.TemporaryDirectory() as d:
   c=Path(d)/'check.c';exe=Path(d)/'check';c.write_text(code)
   subprocess.run(['cc','-Wall','-Werror','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(c),'-o',str(exe)],check=True)
   r=subprocess.run([str(exe)],capture_output=True,text=True,timeout=5,check=True)
   self.assertEqual(r.stderr,'ADC3:-3\n')
