"""Rejected saved settings fall back to their defaults without panic and are reported once after TX starts.

N61: a default that nv_set() refuses is applied through the setter itself, so a gate in nv_set() can never leave a
setting stale; the report names the restored settings with status 118 "Settings restored to defaults"."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function
from config_fallback_source import fallback_source

ROOT = Path(__file__).resolve().parents[2]


class ConfigLoadRecovery(unittest.TestCase):
    def test_recovery_and_deferred_report(self):
        source = (ROOT / 'firmware/tinyg/config.c').read_text()
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define __AVR 1
#define NUL 0
#define PSTR(s) (s)
#define strncpy_P strncpy
#define strcpy_P strcpy
#define NO_MATCH 65535
#define TOKEN_LEN 5
#define F_INITIALIZE 1
#define TYPE_FLOAT 1
#define TYPE_INTEGER 2
#define STAT_OK 0
#define STAT_COMMAND_NOT_ACCEPTED 106
#define STAT_CONFIG_ASSERTION_FAILURE 88
#define STAT_SETTINGS_RESTORED_TO_DEFAULTS 118
#define CONFIG_PLACEHOLDER 0
#define sprintf_P(buf, ...) snprintf(buf, 32, __VA_ARGS__)
#include <stdio.h>
#define MILLIMETERS 0
#define CYCLE_OFF 0
#define MAGICNUM 123
#define BAD_MAGIC(v) ((v)!=MAGICNUM)
#define SR_DEFAULTS "defaults"
#define GET_TABLE_FLOAT(field) (1.8f)
typedef int stat_t;
typedef uint16_t index_t;
typedef struct {int index; char token[6]; float value_flt; uint32_t value_int; int valuetype;} nvObj_t;
typedef stat_t (*fptrCmd)(nvObj_t *nv);
static float ram[3];
static int direct[3];
stat_t get_flt(nvObj_t *nv){return 0;} stat_t get_ui8(nvObj_t *nv){return 0;} stat_t get_u16(nvObj_t *nv){return 0;}
stat_t get_u32(nvObj_t *nv){return 0;} stat_t get_int(nvObj_t *nv){return 0;}
static stat_t setter(nvObj_t *nv){direct[nv->index]++;ram[nv->index]=nv->valuetype==TYPE_FLOAT?nv->value_flt:nv->value_int;return 0;}
struct {char token[6]; fptrCmd get; fptrCmd set; void *target;} cfgArray[]={{"fb",get_flt,setter,0},{"1sa",get_flt,setter,0},{"1mi",get_ui8,setter,0}};
#define GET_TABLE_WORD(a) cfgArray[nv->index].a
struct {int magic_start,magic_end;} cfg,nvl,nvStr;
struct {int cycle_state;} cm;
static nvObj_t object;
static bool current, reject, reject_default, tx;
static int attempts[3],persisted[3],reports;
static char reported[64];
static int setters,markers,panics;
nvObj_t *nv_reset_nv_list(int n){memset(&object,0,sizeof(object));return &object;}
void cm_set_units_mode(int n){}
void read_persistent_value(nvObj_t *nv){nv->value_flt=9.0f;nv->value_int=8;}
bool _nvm_layout_is_current(nvObj_t *nv){return current;}
bool _nvm_layout_is_legacy(nvObj_t *nv){return false;}
index_t nv_index_max(void){return 3;}
index_t nv_get_index(const char *g,const char *t){assert(0);return NO_MATCH;}
bool nv_index_is_single(int index){return index<3;}
bool cfg_has_flag(int index,int flag){return index>0;}
int cfg_is_type(int index){return index==1?TYPE_FLOAT:TYPE_INTEGER;}
stat_t nv_set(nvObj_t *nv){
 setters++;attempts[nv->index]++;
 if(reject_default && nv->index==2)return 106;
 if(nv->index==1 && ((reject && nv->value_flt==9) || (reject_default && nv->value_flt==1.8f))) return 106;
 ram[nv->index]=nv->valuetype==TYPE_FLOAT?nv->value_flt:nv->value_int;return 0;
}
void nv_persist(nvObj_t *nv){persisted[nv->index]++;}
void rpt_print_loading_configs_message(void){}
void rpt_print_initializing_message(void){}
void sr_init_status_report_P(const char *s){}
void _write_nvm_layout_marker(nvObj_t *nv){markers++;}
void rpt_exception_P(int status,const char *s){}
void rpt_exception(int status,const char *s){assert(tx);assert(status==STAT_SETTINGS_RESTORED_TO_DEFAULTS);reports++;strncpy(reported,s,sizeof(reported)-1);}
stat_t cm_panic_P(int status,const char *s){panics++;return status;}
static void _set_defa(nvObj_t *nv);
void config_init_assertions(void);
static bool config_loaded;  /* config.c (N60): set at the end of config_init() */
static bool cm_persistence_busy(void){return cm.cycle_state!=0;}
'''
        code += fallback_source(source)                     # N61: the real report lists and fallback
        for name in ('config_init', '_set_defa', 'config_init_assertions', 'config_test_assertions'):
            code += function(source, r'(?:static )?(?:void|stat_t) ' + name + r'\([^;{}]*\)(?=\s*\{)') + '\n'
        code += r'''
int main(void){
 for(int layout=0;layout<2;layout++) for(int failure=0;failure<3;failure++){
  current=layout;reject=failure>0;reject_default=failure==2;tx=false;
  setters=markers=panics=reports=0;memset(attempts,0,sizeof(attempts));memset(persisted,0,sizeof(persisted));memset(direct,0,sizeof(direct));ram[1]=7;
  config_init();
  bool warned=layout ? failure>0 : false;      /* N61: fresh defaults that needed the setter are applied, not reported */
  assert(config_test_assertions()==STAT_OK && panics==0 && reports==0);
  assert(ram[1]==(layout && !failure?9:1.8f));             /* N61: never left at the stale 7 */
  assert(direct[1]==(failure==2));                          /* the refused default went through the setter */
  assert(attempts[1]==(layout && failure?2:1));
  assert(persisted[1]==(!layout));                          /* fresh EEPROM saves the default, even a forced one (N61) */
  tx=true;config_report_load_failure();
  assert(reports==warned);
  if(warned)assert(strcmp(reported,failure==2?"1sa,1mi":"1sa")==0);
  config_report_load_failure();assert(reports==warned && panics==0);
 }
}

'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / 'check.c').write_text(code)
            subprocess.run(['cc', '-Wall', '-Werror', '-fsanitize=address,undefined','-fstrict-flex-arrays=3', str(path/'check.c'), '-o', str(path/'check')], check=True)
            subprocess.run([str(path/'check')], check=True)
