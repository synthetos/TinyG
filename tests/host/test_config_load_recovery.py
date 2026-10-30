"""Rejected saved settings fall back without panic and report once after TX starts."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function

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
#define TOKEN_LEN 5
#define F_INITIALIZE 1
#define TYPE_FLOAT 1
#define TYPE_INTEGER 2
#define STAT_OK 0
#define STAT_COMMAND_NOT_ACCEPTED 106
#define STAT_CONFIG_ASSERTION_FAILURE 88
#define MILLIMETERS 0
#define CYCLE_OFF 0
#define MAGICNUM 123
#define BAD_MAGIC(v) ((v)!=MAGICNUM)
#define SR_DEFAULTS "defaults"
#define GET_TABLE_FLOAT(field) (1.8f)
typedef int stat_t;
typedef uint16_t index_t;
typedef struct {int index; char token[6]; float value_flt; uint32_t value_int; int valuetype;} nvObj_t;
struct {char token[6];} cfgArray[]={{"fb"},{"1sa"},{"1mi"}};
struct {int magic_start,magic_end;} cfg,nvl,nvStr;
struct {int cycle_state;} cm;
static nvObj_t object;
static bool current, reject, reject_default, tx;
static float ram[3];
static int attempts[3],reports;
static char reported[6];
static int setters,markers,panics;
nvObj_t *nv_reset_nv_list(int n){memset(&object,0,sizeof(object));return &object;}
void cm_set_units_mode(int n){}
void read_persistent_value(nvObj_t *nv){nv->value_flt=9.0f;nv->value_int=8;}
bool _nvm_layout_is_current(nvObj_t *nv){return current;}
bool nv_index_is_single(int index){return index<3;}
bool cfg_has_flag(int index,int flag){return index>0;}
int cfg_is_type(int index){return index==1?TYPE_FLOAT:TYPE_INTEGER;}
stat_t nv_set(nvObj_t *nv){
 setters++;attempts[nv->index]++;
 if(nv->index==1 && ((reject && nv->value_flt==9) || (reject_default && nv->value_flt==1.8f))) return 106;
 ram[nv->index]=nv->valuetype==TYPE_FLOAT?nv->value_flt:nv->value_int;return 0;
}
void nv_persist(nvObj_t *nv){}
void rpt_print_loading_configs_message(void){}
void rpt_print_initializing_message(void){}
void sr_init_status_report_P(const char *s){}
void _write_nvm_layout_marker(nvObj_t *nv){markers++;}
void rpt_exception_P(int status,const char *s){}
void rpt_exception(int status,const char *s){assert(tx);assert(status==STAT_CONFIG_ASSERTION_FAILURE);reports++;strcpy(reported,s);}
stat_t cm_panic_P(int status,const char *s){panics++;return status;}
static void _set_defa(nvObj_t *nv);
void config_init_assertions(void);
'''
        if 'static index_t config_load_failure;' in source:
            code += 'static index_t config_load_failure;\n'
            for name in ('_record_config_load_failure','_load_config_default','config_report_load_failure'):
                code += function(source, r'(?:static )?(?:void|stat_t) '+name+r'\([^;{}]*\)(?=\s*\{)')+'\n'
        else:
            code += 'void config_report_load_failure(void){}\n'
        for name in ('config_init', '_set_defa', 'config_init_assertions', 'config_test_assertions'):
            code += function(source, r'(?:static )?(?:void|stat_t) ' + name + r'\([^;{}]*\)(?=\s*\{)') + '\n'
        code += r'''
int main(void){
 for(int layout=0;layout<2;layout++) for(int failure=0;failure<3;failure++){
  current=layout;reject=failure>0;reject_default=failure==2;tx=false;
  setters=markers=panics=reports=0;memset(attempts,0,sizeof(attempts));ram[1]=7;
  config_init();
  bool warned=layout ? failure>0 : failure==2;
  assert(config_test_assertions()==STAT_OK && panics==0 && reports==0);
  assert(ram[1]==(failure==2?7:(layout && !failure?9:1.8f)));
  assert(attempts[1]==(layout && failure?2:1));
  tx=true;config_report_load_failure();
  assert(reports==warned);
  if(warned)assert(strcmp(reported,"1sa")==0);
  config_report_load_failure();assert(reports==warned && panics==0);
 }
}

'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / 'check.c').write_text(code)
            subprocess.run(['cc', '-Wall', '-Werror', '-fsanitize=address,undefined', str(path/'check.c'), '-o', str(path/'check')], check=True)
            subprocess.run([str(path/'check')], check=True)
