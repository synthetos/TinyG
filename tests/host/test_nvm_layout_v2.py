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

 def test_reserved_slots_are_unsupported_and_never_persisted(self):
  source=(ROOT/'firmware/tinyg/config.c').read_text()
  app=(ROOT/'firmware/tinyg/config_app.c').read_text()
  rows=[]
  defaults={'SPINDLE_PAUSE_ON_HOLD':'1','SPINDLE_DWELL_TIME':'1.0','COOLANT_PAUSE_ON_HOLD':'0'}
  for token in ('spph','spde','coph'):
   row=next(line for line in app.splitlines() if '"'+token+'"' in line)
   self.assertIn('_reserved_hold_setting,_reserved_hold_setting',row)
   flag='_f0' if token=='spde' else '_i0'
   self.assertIn(flag,row)
   value=re.search(r',([^,}]+)\s*}',row).group(1).strip()
   rows.append('{"'+token+'", '+('TYPE_FLOAT' if token=='spde' else 'TYPE_INTEGER')+', '+defaults[value]+'}')
  code=r'''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#define TYPE_INTEGER 2
#define TYPE_FLOAT 5
#define TYPE_EMPTY -1
#define STAT_FUNCTION_IS_STUBBED 17
#define CYCLE_OFF 0
#define MILLIMETERS 1
#define STAT_OK 0
#define STAT_COMMAND_NOT_ACCEPTED 106
#define STAT_INTERNAL_RANGE_ERROR 255
#define STAT_PARAMETER_IS_READ_ONLY 107
#define F_INITIALIZE 1
#define F_PERSIST 2
#define TOKEN_LEN 5
#define PSTR(s) (s)
#define SR_DEFAULTS ""
#define strncpy_P strncpy
#define GET_TABLE_FLOAT(f) cfgArray[nv->index].f
typedef int stat_t;
typedef int index_t;
static index_t config_load_failure;
typedef struct {int index,valuetype; char token[6]; union {float value_flt;uint32_t value_int;};} nvObj_t;
struct {int cycle_state;} cm;
struct {char token[6];int type;float default_value;} cfgArray[]={
 {"fb",TYPE_FLOAT,449.01},
'''+',\n'.join(rows)+r'''
};
static uint32_t records[4];
bool nv_index_is_single(int i) {return i<4;}
bool nv_index_lt_groups(int i) {return i<4;}
bool cfg_has_flag(int i,int f) {return i==0 && f==F_PERSIST;}
int cfg_is_type(int i) {return cfgArray[i].type;}
void cm_set_units_mode(int m) {}
void rpt_exception_P(int s,const char *m) {assert(0);}
void sr_init_status_report_P(const char *s) {}
void rpt_print_initializing_message(void) {}
stat_t write_persistent_value(nvObj_t *nv) {records[nv->index]=nv->value_int;return 0;}
void _write_nvm_layout_marker(nvObj_t *nv) {nv->index=0;nv->value_flt=10002;write_persistent_value(nv);}
'''
  code+='stat_t nv_set(nvObj_t *nv) {assert(nv->index==0);return STAT_PARAMETER_IS_READ_ONLY; }\n'
  for name in ('_record_config_load_failure','_load_config_default'):
   code+=function(source,r'static (?:void|stat_t) '+name+r'\([^;{}]*\)(?=\s*\{)')+'\n'
  code+=function(source,r'stat_t nv_persist\([^;{}]*\)[^\n]*\n(?=\{)')+'\n'
  code+=function(source,r'static void _set_defa\([^;{}]*\)(?=\s*\{)')+'\n'
  code+=function(app,r'static stat_t _reserved_hold_setting\([^;{}]*\)(?=\s*\{)')+'\n'
  code+=r'''
int main(void) {
 for(int i=0;i<4;i++) records[i]=0xdeadbeef;
 nvObj_t nv={0}; _set_defa(&nv);
 for(int i=1;i<4;i++) {
  assert(records[i]==0xdeadbeef);nv.index=i;nv.value_int=42;
  assert(nv_persist(&nv)==0 && records[i]==0xdeadbeef);
  assert(_reserved_hold_setting(&nv)==STAT_FUNCTION_IS_STUBBED && nv.valuetype==TYPE_EMPTY);
 }
}
'''
  with tempfile.TemporaryDirectory() as d:
   c=Path(d)/'check.c';exe=Path(d)/'check';c.write_text(code)
   subprocess.run(['cc','-Wall','-Werror','-fsanitize=address,undefined',str(c),'-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
