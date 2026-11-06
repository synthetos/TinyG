"""Signed status-row sentinel is a domain value, not generic unsigned overflow."""
from pathlib import Path
import subprocess
import tempfile
from source_extract import function

FW=Path(__file__).resolve().parents[2]/'firmware/tinyg'


def test_real_status_row_setter_preserves_unused_marker():
    code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define STAT_OK 0
#define STAT_INPUT_VALUE_RANGE_ERROR 110
#define TYPE_INTEGER 3
#define TYPE_SIGNED 5
#define NO_MATCH -1
typedef int stat_t;
typedef int16_t index_t;
typedef struct {uint32_t value_int;int index,valuetype;} nvObj_t;
static uint16_t storage[32];
#define GET_TABLE_WORD(field) (&storage[nv->index+1])
static unsigned nv_index_max(void) {return 100;}
static bool cfg_is_reportable(index_t i) {return i==3 || i==7;}
'''
    c=(FW/'config.c').read_text();r=(FW/'report.c').read_text()
    for name in ['get_u16','set_u16','set_ui8']:
        code+=function(c,r'stat_t '+name+r'\(nvObj_t \*nv\)')+'\n'
    for sig in [r'static bool _valid_status_report_index\(uint32_t index\)',
                r'stat_t sr_get_se\(nvObj_t \*nv\)',r'stat_t sr_set_se\(nvObj_t \*nv\)']:
        code+=function(r,sig)+'\n'
    code+=r'''
int main(void) {
 storage[0]=0xA55A;storage[31]=0x5AA5;
 for(int row=0;row<30;row++) {
  nvObj_t n={.index=row,.valuetype=TYPE_SIGNED,.value_int=(uint32_t)-1};
  storage[row+1]=7;
  assert(sr_set_se(&n)==STAT_OK);
  assert(storage[row+1]==UINT16_MAX);
  // Preserve signed echo and the 32-bit sentinel passed to persistence.
  assert(n.value_int==UINT32_MAX && n.valuetype==TYPE_SIGNED);
  assert(sr_get_se(&n)==STAT_OK && n.value_int==UINT32_MAX && n.valuetype==TYPE_SIGNED);
  n.value_int=3;assert(sr_set_se(&n)==STAT_OK && storage[row+1]==3);
  assert(sr_get_se(&n)==STAT_OK && n.value_int==3);
  uint32_t rejected[]={UINT32_MAX-1,UINT32_MAX-2,65535,65536,0x80000000UL,2,100};
  for(unsigned j=0;j<sizeof(rejected)/sizeof(*rejected);j++) {
   n.value_int=rejected[j];assert(sr_set_se(&n)==110 && storage[row+1]==3);
   assert(n.value_int==rejected[j]);
  }
 }
 assert(storage[0]==0xA55A && storage[31]==0x5AA5);
 nvObj_t n={.index=0,.value_int=UINT32_MAX};storage[1]=0x1234;
 assert(set_u16(&n)==110 && storage[1]==0x1234);
 assert(set_ui8(&n)==110 && storage[1]==0x1234);
 n.value_int=65536;assert(set_u16(&n)==110 && storage[1]==0x1234);
 n.value_int=256;assert(set_ui8(&n)==110 && storage[1]==0x1234);
}
'''
    with tempfile.TemporaryDirectory() as d:
        p=Path(d);(p/'check.c').write_text(code)
        subprocess.run(['cc','-std=c99','-Wall','-Werror','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(p/'check.c'),'-o',str(p/'check')],check=True)
        subprocess.run([str(p/'check')],check=True)
