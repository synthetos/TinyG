"""Real config_init and singles predicate with AVR-preprocessed table append orders.

EEPROM and row setters are modeled; separate setter/default tests cover validation.
Permutations test eligibility, not interchangeability of persisted EEPROM indexes.
"""
from pathlib import Path
import itertools
import json
import re
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]
FW=ROOT/'firmware/tinyg'

class AppendedConfigBoot(unittest.TestCase):
 def test_boot_and_migration_every_append_order(self):
  app=(FW/'config_app.c').read_text();config=(FW/'config.c').read_text()
  fixtures=json.loads((Path(__file__).parent/'fixtures/appended_config_rows.json').read_text())
  def token(line):
   m=re.match(r'\s*\{\s*"[^"]*"\s*,\s*"([^"]*)"',line)
   return m.group(1) if m else None
  v2={'spep','spdp','spph','spde','coph','prbp','fstk','adc3'}
  blocks={'v2':[line for line in app.splitlines() if token(line) in v2],
          'legacy':fixtures['legacy'],'optional_stop':fixtures['optional_stop']}
  self.assertEqual(len(blocks['v2']),8)
  clean='\n'.join(line for line in app.splitlines() if token(line) not in v2|{'os','net','ma','fd'})
  for order in itertools.permutations(blocks):
   with self.subTest(order=order):
    added='\n'.join(row for block in order for row in blocks[block])+'\n'
    combined=clean.replace('// Group lookups - must follow',added+'// Group lookups - must follow')
    pre=subprocess.check_output(['avr-gcc','-mmcu=atxmega192a3','-DF_CPU=32000000UL','-I',str(FW),'-E','-P','-x','c','-'],input=combined,text=True)
    table=pre.split('const cfgItem_t cfgArray[]',1)[1].split('\n};',1)[0]
    rows=[row.split(',') for row in re.findall(r'\{(\s*"[^{}]+)\}',table)]
    first_group=next(i for i,r in enumerate(rows) if r[1].strip()=='"sys"')
    enum='\n'.join(re.findall(r'typedef enum\s*\{[^{}]*\}\s*\w+\s*;',pre))
    code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#define __AVR 1
#define NUL 0
#define NO_MATCH -1
#define TOKEN_LEN 5
#define STAT_OK 0
#define STAT_CONFIG_ASSERTION_FAILURE 90
#define STAT_COMMAND_NOT_ACCEPTED 106
#define STAT_INTERNAL_RANGE_ERROR 21
#define F_INITIALIZE 0x10
#define F_PERSIST 0x20
#define PSTR(s) (s)
#define strcpy_P strcpy
#define strncpy_P strncpy
#define SR_DEFAULTS "defaults"
#define fp_EQ(a,b) (fabs((a)-(b))<.00001)
#define GET_TABLE_FLOAT(f) cfgArray[nv->index].f
typedef int stat_t;
typedef int16_t index_t;
typedef union {uint32_t u;float f;} value_t;
typedef struct {index_t index;int valuetype;char token[6];union {uint32_t value_int;float value_flt;};} nvObj_t;
struct {int cycle_state;} cm;
'''+enum+'\n'
    code+='typedef struct {const char *token;unsigned flags;float default_value;} cfgItem_t;\nstatic const cfgItem_t cfgArray[]={\n'
    code+='\n'.join('{'+r[1]+','+r[2]+','+r[-1]+'},' for r in rows)+'\n};\n'
    code+=f'#define FIRST_GROUP {first_group}\n'
    code+=r'''
#define COUNT (sizeof(cfgArray)/sizeof(*cfgArray))
value_t eeprom[COUNT],model[COUNT];unsigned writes[COUNT],sets[COUNT];
static nvObj_t object;
static index_t config_load_failure;
static unsigned sr_loads;
static void _set_defa(nvObj_t *nv);
stat_t nv_persist(nvObj_t *nv);
nvObj_t *nv_reset_nv_list(char c){memset(&object,0,sizeof(object));return &object;}
void config_init_assertions(void){}
void cm_set_units_mode(int m){assert(m==MILLIMETERS);}
void rpt_print_loading_configs_message(void){}
void rpt_print_initializing_message(void){}
void rpt_exception_P(int s,const char *m){assert(0);}
stat_t cm_panic_P(int s,const char *m){assert(0);return s;}
void sr_init_status_report_P(const char *s){sr_loads++;}
bool cfg_has_flag(int i,int f){return cfgArray[i].flags&f;}
int cfg_is_type(int i){return cfgArray[i].flags&7;}
index_t nv_index_max(void){return COUNT;}
bool nv_index_lt_groups(int i){return i<FIRST_GROUP;}
index_t nv_get_index(const char *g,const char *t){for(int i=0;i<COUNT;i++)if(!strcmp(t,cfgArray[i].token))return i;return NO_MATCH;}
stat_t read_persistent_value(nvObj_t *nv){nv->value_int=eeprom[nv->index].u;return 0;}
stat_t nv_set(nvObj_t *nv){sets[nv->index]++;model[nv->index].u=nv->value_int;return 0;}
stat_t write_persistent_value(nvObj_t *nv){
 if(nv->index==0){for(int i=nv_get_index("","spep");i<FIRST_GROUP;i++)
   if(cfg_has_flag(i,F_INITIALIZE)&&cfg_has_flag(i,F_PERSIST))assert(writes[i]==1);}
 writes[nv->index]++;eeprom[nv->index].u=nv->value_int;return 0;
}
'''
    code+=function(pre,r'(?:_Bool|bool) nv_index_is_single\([^;{}]*\)(?=\s*\{)')+'\n'
    code+='\n'.join(line for line in config.splitlines() if line.startswith('#define CONFIG_NVM_LAYOUT'))+'\n'
    for name in ('_record_config_load_failure','_load_config_default','_nvm_layout_is_current','_nvm_layout_is_legacy','_write_nvm_layout_marker'):
     code+=function(config,r'(?:static )?(?:bool|void|stat_t) '+name+r'\([^;{}]*\)(?=\s*\{)')+'\n'
    code+=function(config,r'stat_t nv_persist\([^;{}]*\)[^\n]*\n(?=\{)')+'\n'
    code+=function(config,r'static void _set_defa\([^;{}]*\)(?=\s*\{)')+'\n'
    code+=function(config,r'void config_init\([^;{}]*\)(?=\s*\{)')+'\n'
    code+=r'''
value_t def(int i){value_t v;if(cfg_is_type(i)==TYPE_FLOAT)v.f=cfgArray[i].default_value;else v.u=(uint32_t)cfgArray[i].default_value;return v;}
void reset(float marker){
 memset(writes,0,sizeof(writes));memset(sets,0,sizeof(sets));memset(model,0,sizeof(model));sr_loads=0;
 for(unsigned i=0;i<COUNT;i++)eeprom[i].u=0xdeadbeef;
 eeprom[0].f=marker;eeprom[nv_get_index("","1sa")].f=.9f;
}
int main(void){
 int boundary=nv_get_index("","spep"),sa=nv_get_index("","1sa"),os=nv_get_index("","os");
 assert(nv_index_is_single(os));assert(!cfg_has_flag(os,F_INITIALIZE|F_PERSIST));
 for(int i=0;i<COUNT;i++)assert(nv_index_is_single(i)==(i<FIRST_GROUP));
 const char *new_tokens[]={"spep","spdp","prbp"};
 for(unsigned t=0;t<3;t++)assert(cfg_has_flag(nv_get_index("",new_tokens[t]),F_INITIALIZE|F_PERSIST));
 float legacy[]={10001,449.01f};
 for(int m=0;m<2;m++){
  reset(legacy[m]);config_init();
  assert(model[sa].f==.9f && eeprom[sa].f==.9f && !writes[sa]);
  for(int i=1;i<COUNT;i++){
   bool init=cfg_has_flag(i,F_INITIALIZE),fresh=i>=boundary && i<FIRST_GROUP;
   assert(sets[i]==(unsigned)init);
   if(init)assert(model[i].u==(fresh?def(i).u:eeprom[i].u));
   assert(writes[i]==(unsigned)(init&&fresh&&cfg_has_flag(i,F_PERSIST)));
   if(!init)assert(eeprom[i].u==0xdeadbeef);
  }
  assert(eeprom[0].f==10002 && writes[0]==1 && sr_loads==1);
  // After migration, non-default appended values must survive another boot.
  for(unsigned t=0;t<3;t++){int i=nv_get_index("",new_tokens[t]);eeprom[i].u=!def(i).u;}
  memset(writes,0,sizeof(writes));memset(sets,0,sizeof(sets));memset(model,0,sizeof(model));
  config_init();
  for(int i=0;i<COUNT;i++){assert(writes[i]==0);if(cfg_has_flag(i,F_INITIALIZE))assert(sets[i]==1 && model[i].u==eeprom[i].u);}
 }
}
'''
    with tempfile.TemporaryDirectory() as tmp:
     c=Path(tmp)/'boot.c';exe=Path(tmp)/'boot';c.write_text(code)
     subprocess.run(['cc','-std=c99','-Wall','-Werror','-fsanitize=address,undefined',str(c),'-o',str(exe)],check=True)
     subprocess.run([str(exe)],check=True)
