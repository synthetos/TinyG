"""Run real config initialization against a synthetic EEPROM and table boundary."""
from pathlib import Path
import subprocess, tempfile, unittest
from source_extract import function
from config_fallback_source import fallback_source, REFUSING_SETTER
ROOT=Path(__file__).resolve().parents[2]
class NvmMigration(unittest.TestCase):
 def test_append_only_upgrade_preserves_old_values(self):
  source=(ROOT/'firmware/tinyg/config.c').read_text()
  code=r'''
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#define __AVR 1
#define TYPE_INTEGER 2
#define TYPE_FLOAT 5
#define CYCLE_OFF 0
#define MILLIMETERS 1
#define STAT_OK 0
#define STAT_COMMAND_NOT_ACCEPTED 106
#define STAT_CONFIG_ASSERTION_FAILURE 90
#define STAT_SETTINGS_RESTORED_TO_DEFAULTS 118
#include <stdio.h>
#define STAT_INTERNAL_RANGE_ERROR 21
#define F_INITIALIZE 1
#define F_PERSIST 2
#define TOKEN_LEN 5
#define NUL 0
#define NO_MATCH -1
#define sprintf_P(buf, ...) snprintf(buf, 32, __VA_ARGS__)
#define PSTR(s) (s)
#define SR_DEFAULTS "defaults"
#define strcpy_P strcpy
#define strncpy_P strncpy
#define fp_EQ(a,b) (fabs((a)-(b))<.00001)
#define GET_TABLE_FLOAT(f) cfgArray[nv->index].f
typedef int stat_t;
typedef int index_t;
typedef union {uint32_t u;float f;} value_t;
typedef struct {int index,valuetype;char token[6];union {uint32_t value_int;float value_flt;};} nvObj_t;
struct {int cycle_state;} cm;
struct {char token[6];int type,flags;float default_value;} cfgArray[]={
 {"fb",TYPE_FLOAT,2,449.01},
 {"1sa",TYPE_FLOAT,3,1.8},
 {"g54x",TYPE_FLOAT,3,0},
 {"st",TYPE_INTEGER,3,1},
 {"se00",TYPE_INTEGER,2,0},
 {"spep",TYPE_INTEGER,3,1},
 {"spdp",TYPE_INTEGER,3,0},
 {"spph",TYPE_INTEGER,0,1},
 {"spde",TYPE_FLOAT,0,1},
 {"coph",TYPE_INTEGER,0,0},
 {"prbp",TYPE_INTEGER,3,0},
 {"fstk",TYPE_INTEGER,0,0},
 {"adc3",TYPE_FLOAT,0,0}
};
#define COUNT (sizeof(cfgArray)/sizeof(*cfgArray))
static value_t eeprom[COUNT], model[COUNT];
static unsigned writes[COUNT], loaded_sr, defaulted_sr, boundary_lookups;
static nvObj_t object;
static bool reject_old, reject_new, tx;
static unsigned reports;
static char reported[6];
static void _set_defa(nvObj_t *nv);
stat_t nv_persist(nvObj_t *nv);
nvObj_t *nv_reset_nv_list(char c) {memset(&object,0,sizeof(object));return &object;}
void config_init_assertions(void) {}
static bool config_loaded;  /* config.c (N60): set at the end of config_init() */
static bool cm_persistence_busy(void){return cm.cycle_state!=0;}
void cm_set_units_mode(int m) {assert(m==MILLIMETERS);}
void rpt_print_loading_configs_message(void) {}
void rpt_print_initializing_message(void) {}
void rpt_exception_P(int s,const char *m) {assert(0);}
stat_t cm_panic_P(int s,const char *m) {assert(0);return s;}
void sr_init_status_report_P(const char *s) {if(*s)defaulted_sr++;else loaded_sr++;}
bool nv_index_is_single(int i) {return i>=0 && i<COUNT;}
bool nv_index_lt_groups(int i) {return nv_index_is_single(i);}
index_t nv_index_max(void) {return COUNT;}
index_t nv_get_index(const char *g,const char *t) {
 assert(!strcmp(t,"spep"));boundary_lookups++;
 for(int i=0;i<COUNT;i++)if(!strcmp(cfgArray[i].token,t))return i;
 return NO_MATCH;
}
bool cfg_has_flag(int i,int f) {return cfgArray[i].flags & f;}
int cfg_is_type(int i) {return cfgArray[i].type;}
stat_t read_persistent_value(nvObj_t *nv) {nv->value_int=eeprom[nv->index].u;return 0;}
stat_t write_persistent_value(nvObj_t *nv) {eeprom[nv->index].u=nv->value_int;writes[nv->index]++;return 0;}
stat_t nv_set(nvObj_t *nv) {
 if(reject_old && nv->index==1 && nv->value_flt==.9f)return 106;
 if(reject_new && nv->index==5)return 106;
 model[nv->index].u=nv->value_int;return 0;
}
static int reported_status;
void rpt_exception(int status,const char *token){assert(tx);reported_status=status;reports++;strcpy(reported,token);}
'''
  code+='\n'.join(x for x in source.splitlines() if x.startswith('#define CONFIG_NVM_LAYOUT'))+'\n'
  code+=REFUSING_SETTER+fallback_source(source)  # N61
  for name in ('_nvm_layout_is_current','_nvm_layout_is_legacy','_write_nvm_layout_marker'):
   if name+'(' in source:
    code+=function(source,r'(?:static )?(?:bool|void|stat_t) '+name+r'\([^;{}]*\)(?=\s*\{)')+'\n'
  code+=function(source,r'stat_t nv_persist\([^;{}]*\)[^\n]*\n(?=\{)')+'\n'
  code+=function(source,r'static void _set_defa\([^;{}]*\)(?=\s*\{)')+'\n'
  code+='bool _nvm_layout_is_master(nvObj_t *nv){(void)nv;return false;}  /* N92: these boards never hold a master build */\nvoid _migrate_master(nvObj_t *nv,index_t done){(void)nv;(void)done;}\nindex_t _master_checkpoint(index_t *slot,uint8_t *seq){(void)slot;(void)seq;return 0;}\nvoid _clear_master_checkpoints(void){}\n'
  code+=function(source,r'void config_init\([^;{}]*\)(?=\s*\{)')+'\n'
  code+=r'''
void prepare(float marker) {
 memset(writes,0,sizeof(writes));memset(model,0,sizeof(model));
 loaded_sr=defaulted_sr=boundary_lookups=reports=0;config_restored.count=0;config_restored.last=NO_MATCH;config_failed.count=0;config_failed.last=NO_MATCH;reject_old=reject_new=tx=false;
 for(int i=0;i<COUNT;i++)eeprom[i].u=0xdeadbeef;
 eeprom[0].f=marker;eeprom[1].f=.9f;eeprom[2].f=12.5f;eeprom[3].u=0;eeprom[4].u=123;
}
int main(void) {
 float legacy[]={10001,449.01f};
 for(int i=0;i<2;i++) {
  prepare(legacy[i]);config_init();
  assert(model[1].f==.9f && model[2].f==12.5f && model[3].u==0);
  assert(eeprom[1].f==.9f && eeprom[2].f==12.5f && eeprom[3].u==0 && eeprom[4].u==123);
  assert(model[5].u==1 && model[6].u==0 && model[10].u==0);
  assert(eeprom[5].u==1 && eeprom[6].u==0 && eeprom[10].u==0);
  assert(eeprom[0].f==10003 && writes[0]==1 && boundary_lookups==1);
  for(int j=1;j<COUNT;j++)assert(writes[j]==(j==5 || j==6 || j==10));
  for(int j=7;j<=9;j++)assert(eeprom[j].u==0xdeadbeef);
  assert(loaded_sr==1 && defaulted_sr==0);
  eeprom[5].u=0;eeprom[10].u=1;memset(writes,0,sizeof(writes));config_init();
  assert(model[1].f==.9f && model[5].u==0 && model[10].u==1);
  for(int j=0;j<COUNT;j++)assert(writes[j]==0);
 }
 // Legacy and current layouts recover an invalid old value only in RAM.
 float markers[]={449.01f,10001,10003};
 for(int i=0;i<3;i++){
  prepare(markers[i]);reject_old=true;config_init();
  assert(model[1].f==1.8f && eeprom[1].f==.9f && writes[1]==0);
  assert(reports==0);tx=true;config_report_load_failure();
  assert(reports==1 && reported_status==118 && !strcmp(reported,"1sa"));config_report_load_failure();assert(reports==1);  /* restored */
 }
 // A rejected new default leaves its EEPROM slot untouched and migration retryable.
 for(int i=0;i<2;i++){
  prepare(legacy[i]);reject_new=true;config_init();
  assert(writes[5]==0 && eeprom[5].u==0xdeadbeef);
  assert(eeprom[0].f==legacy[i] && writes[0]==0 && model[1].f==.9f);
  assert(reports==0);tx=true;config_report_load_failure();
  assert(reports==1 && reported_status==STAT_CONFIG_ASSERTION_FAILURE && !strcmp(reported,"spep"));  /* N61: refused default: not restored */
  reject_new=false;config_init();
  assert(eeprom[0].f==10003 && writes[0]==1 && model[5].u==1);
 }
 float invalid[]={449.99f,10000,10004,0,NAN,INFINITY};
 for(int i=0;i<6;i++){
  prepare(invalid[i]);config_init();
  assert(model[1].f==1.8f && model[2].f==0 && model[3].u==1);
  assert(eeprom[0].f==10003 && defaulted_sr==1 && loaded_sr==0);
  for(int j=7;j<=9;j++)assert(writes[j]==0 && eeprom[j].u==0xdeadbeef);
 }
}
'''
  with tempfile.TemporaryDirectory() as d:
   c=Path(d)/'check.c';exe=Path(d)/'check';c.write_text(code)
   subprocess.run(['cc','-Wall','-Werror','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(c),'-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
