"""Activate the reserved layout-2 cells once, retaining every earlier index and saved setting."""
from pathlib import Path
import json
import re
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "firmware/tinyg"


class HoldSettingsActivation(unittest.TestCase):
    def test_existing_single_value_indexes_do_not_move(self):
        source = (FW / "config_app.c").read_text()
        singles = source.split("const cfgItem_t cfgArray[]", 1)[1].split("// Group lookups", 1)[0]
        tokens = re.findall(r'\{\s*"[^"]*"\s*,\s*"([^"]+)"', singles)
        old = json.loads((Path(__file__).parent / "layout2_before_e060.json").read_text())
        self.assertEqual(tokens[:len(old)], old)
        self.assertEqual(tokens[len(old)], "_hld")

    def test_boot_initializes_only_unactivated_hold_slots(self):
        source = (FW / "config.c").read_text()
        defines = "\n".join(line for line in source.splitlines()
                            if line.startswith(("#define CONFIG_NVM_LAYOUT", "#define CONFIG_HOLD_SETTINGS_STAMP")))
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#define __AVR
#define STAT_OK 0
#define STAT_CONFIG_ASSERTION_FAILURE 90
#define NO_MATCH -1
#define NUL 0
#define TOKEN_LEN 5
#define MILLIMETERS 1
#define F_INITIALIZE 1
#define TYPE_FLOAT 5
#define TYPE_INTEGER 2
#define PSTR(x) (x)
#define strncpy_P strncpy
#define strcpy_P strcpy
#define strcmp_P strcmp
#define fp_EQ(a,b) (fabs((a)-(b))<.00001f)
typedef int stat_t;
typedef int index_t;
typedef struct {index_t index;int valuetype;char token[6];union{float value_flt;uint32_t value_int;};} nvObj_t;
static nvObj_t object;
static struct {const char*token;float value;int type;} cfgArray[]={
 {"fb",0,TYPE_FLOAT},{"spep",1,TYPE_INTEGER},{"spph",1,TYPE_INTEGER},
 {"spde",1,TYPE_FLOAT},{"coph",0,TYPE_INTEGER},{"_hld",0,TYPE_INTEGER}};
static uint32_t stored[6],ram[6];
static int writes,last_written,refuse_index=-1;
static bool config_loaded;
static struct {int count;} config_failed;
static uint32_t bits(float f){union{float f;uint32_t u;}v={.f=f};return v.u;}
static nvObj_t* nv_reset_nv_list(int ignored){memset(&object,0,sizeof(object));return &object;}
static index_t nv_get_index(const char*g,const char*t){for(int i=0;i<6;i++)if(!strcmp(cfgArray[i].token,t))return i;return NO_MATCH;}
static bool nv_index_is_single(index_t i){return i<6;}
static index_t nv_index_max(void){return 6;}
static bool cfg_has_flag(index_t i,int flag){return i>0&&i<5;}
static int cfg_is_type(index_t i){return cfgArray[i].type;}
static stat_t read_persistent_value(nvObj_t*n){n->value_int=stored[n->index];return 0;}
static stat_t write_persistent_value(nvObj_t*n){
 if(n->index==refuse_index)return 106;
 stored[n->index]=n->value_int;writes++;last_written=n->index;return 0;
}
static stat_t nv_persist(nvObj_t*n){return write_persistent_value(n);}
static stat_t nv_set(nvObj_t*n){ram[n->index]=n->value_int;return 0;}
static stat_t _load_config_default(nvObj_t*n){
 n->value_flt=cfgArray[n->index].value;
 if(cfg_is_type(n->index)!=TYPE_FLOAT)n->value_int=(uint32_t)n->value_flt;
 return nv_set(n);
}
static void _load_config_fallback(nvObj_t*n){assert(0);}
static void config_init_assertions(void){}
static void cm_set_units_mode(int mode){}
static void rpt_print_loading_configs_message(void){}
static void sr_init_status_report_P(const char*p){}
static stat_t cm_panic_P(int s,const char*p){assert(0);return s;}
static void _set_defa(nvObj_t*n){
 for(n->index=1;n->index<5;n->index++){_load_config_default(n);nv_persist(n);}
 n->index=0;n->value_flt=10002;write_persistent_value(n);
}
''' + defines + "\n"
        for name in ("_nvm_layout_is_current", "_nvm_layout_is_legacy", "_write_nvm_layout_marker",
                     "_hold_settings_need_defaults", "_is_hold_setting", "_write_hold_settings_marker", "config_init"):
            code += function(source, r'(?:static )?(?:bool|void) ' + name + r'\([^;{}]*\)(?=\s*\{)') + "\n"
        code += r'''
static void boot(void){writes=0;config_loaded=false;memset(ram,0,sizeof(ram));config_init();assert(config_loaded);}
int main(void){
 // Plausible bytes in formerly unused slots are not user choices; initialize all three.
 stored[0]=bits(10002);stored[1]=0;stored[2]=0;stored[3]=bits(3.5f);stored[4]=1;stored[5]=UINT32_MAX;
 boot();
 assert(ram[1]==0&&stored[1]==0);  // an unrelated saved setting survives
 assert(ram[2]==1&&ram[3]==bits(1)&&ram[4]==0);
 assert(stored[2]==1&&stored[3]==bits(1)&&stored[4]==0);
 assert(stored[0]==bits(10002)&&stored[5]==CONFIG_HOLD_SETTINGS_STAMP&&last_written==5);
 // Later user choices load unchanged, without writes or a repeated activation.
 stored[2]=0;stored[3]=bits(2.5f);stored[4]=1;
 boot();assert(!writes&&ram[2]==0&&ram[3]==bits(2.5f)&&ram[4]==1&&ram[1]==0);
 // Partial persistence must not stamp activation complete; the next boot retries.
 stored[5]=UINT32_MAX;refuse_index=3;
 boot();assert(stored[5]==UINT32_MAX);
 refuse_index=-1;boot();assert(stored[5]==CONFIG_HOLD_SETTINGS_STAMP&&stored[3]==bits(1));
 // A fresh or layout-1 board also initializes the reserved defaults.
 stored[0]=UINT32_MAX;stored[5]=UINT32_MAX;boot();
 assert(stored[0]==bits(10002)&&stored[5]==CONFIG_HOLD_SETTINGS_STAMP);
 stored[0]=bits(10001);stored[5]=UINT32_MAX;stored[2]=0;stored[4]=1;boot();
 assert(stored[0]==bits(10002)&&stored[2]==1&&stored[4]==0&&stored[5]==CONFIG_HOLD_SETTINGS_STAMP);
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c, exe = Path(tmp)/"activation.c", Path(tmp)/"activation"
            c.write_text(code)
            subprocess.run(["cc", "-std=c99", "-fsanitize=address,undefined", "-fstrict-flex-arrays=3",
                            str(c), "-o", str(exe), "-lm"], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
