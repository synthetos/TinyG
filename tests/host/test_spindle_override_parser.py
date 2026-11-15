"""Run the actual JSON/text setters through RAM-only config persistence dispatch."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function
from test_override_factor_parser import source, h


class SpindleParser(unittest.TestCase):
    def test_requests_ranges_permissions_and_no_eeprom(self):
        fw = h.FW
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp);code=source(out)
            code=code.replace('!strcmp(full,names[i]) || !strcmp(t,names[i])','!strcmp(full,names[i]) || (!*g && !strcmp(t,names[i]))')
            code=code.replace('"rt", "rtmfo"};','"rt", "rtmfo", "sso", "ssoe", "sse", "scp", "rtsso", "rtsse"};')
            code=code.replace('TYPE_NULL, TYPE_FLOAT};','TYPE_NULL, TYPE_FLOAT, TYPE_FLOAT, TYPE_INTEGER, TYPE_INTEGER, TYPE_INTEGER, TYPE_FLOAT, TYPE_INTEGER};')
            code=code.replace('static stat_t spindle_get_sso(nvObj_t*);','stat_t spindle_get_sso(nvObj_t*);')
            code=code.replace('static stat_t spindle_get_sso(nvObj_t *n){n->value_flt=1;return STAT_OK;}','')
            prototypes=''.join('stat_t '+n+'(nvObj_t*);' for n in ('spindle_get_sso','spindle_set_sso','spindle_set_ssoe','spindle_get_ssoe','spindle_get_sse','spindle_get_scp','rt_set_sso','rt_set_sse'))
            code=code.replace('static fptrCmd table_get', prototypes+'\nstatic fptrCmd table_get')
            code=code.replace('return i==11?ov_get_mfo:capture_get;', 'return (i==12||i==16)?spindle_get_sso:i==13?spindle_get_ssoe:(i==14||i==17)?spindle_get_sse:i==15?spindle_get_scp:i==11?ov_get_mfo:capture_get;')
            code=code.replace('return i==10?rt_set_grp:i==11?tracked_set:', 'return i==12?spindle_set_sso:i==13?spindle_set_ssoe:i==16?rt_set_sso:i==17?rt_set_sse:(i==14||i==15)?set_nul:i==10?rt_set_grp:i==11?tracked_set:')
            persist=function((fw/'config.c').read_text(),r'stat_t nv_persist\(')
            code=code.replace('stat_t nv_persist(nvObj_t *n) { (void)n;return STAT_OK; }',
                'static unsigned writes;bool nv_index_lt_groups(index_t i){return nv_index_is_single(i);}\nstat_t write_persistent_value(nvObj_t*n){writes++;return 0;}\n'+persist)
            code+='''
#include <errno.h>
#define memcpy_P memcpy
#define strcpy_P strcpy
#define TEXT_RESPONSE 0
#define JSON_RESPONSE 0
const cfgItem_t cfgArray[18]={{0}};
static uint8_t SREG=128;static void cli(void){SREG=0;}
static uint16_t spindle_factor=1000,spindle_applied=1000;
static uint8_t spindle_enables=7;
static bool spindle_snapshot_valid,spindle_dirty,spindle_capped;
#define SSO_OPERATOR 1
#define SSO_MASTER 2
#define SSO_PROGRAM 4
void sr_run_text_status_report(void){}
void help_general(nvObj_t*n){}
void nv_print_list(stat_t s,uint8_t a,uint8_t b){}
'''
            sp=(fw/'spindle.c').read_text();rt=(fw/'runtime.c').read_text();text=(fw/'text_parser.c').read_text()
            for name in ('_get_state','_get_factor','spindle_get_sso','spindle_set_sso','spindle_get_ssoe','spindle_set_ssoe','spindle_get_sse','spindle_get_scp'):
                code+=function(sp,r'(?:static )?stat_t '+name+r'\([^;{}]*\)\s*(?=\{)')+'\n'
            for name in ('rt_set_sso','rt_set_sse'):
                code+=function(rt,r'stat_t '+name+r'\(')+'\n'
            for name in ('_text_parser_kernal','text_parser'):
                code+=function(text,r'(?m)^(?:static )?stat_t '+name+r'\([^;{}]*\)\s*(?=\{)')+'\n'
            code+=r'''
static void json(const char*s,int expected){char line[100];strcpy(line,s);json_parser(line);assert(last_status==expected);}
static void txt(const char*s,int expected){char line[100];strcpy(line,s);assert(text_parser(line)==expected);}
int main(void){
 reset();cm.machine_state=MACHINE_READY;
 json("{\"sso\":0.05}",0);assert(spindle_factor==50);
 json("{\"sso\":5e-2}",0);assert(spindle_factor==50);
 json("{\"sso\":2.0}",0);assert(spindle_factor==2000);
 json("{\"rt\":{\"sso\":0.5}}",0);assert(spindle_factor==500);
 txt("$sso=1.5",0);assert(spindle_factor==1500);txt("$sso",0);
 txt("$ssoe=0",0);assert(!(spindle_enables&1));json("{\"ssoe\":1}",0);assert(spindle_enables==7);
 json("{\"rt\":{\"sse\":0}}",0);assert(!(spindle_enables&1));
 json("{\"sso\":0.049}",110);json("{\"sso\":2.001}",110);assert(spindle_factor==1500);
 json("{\"ssoe\":2}",110);json("{\"sse\":1}",104);json("{\"scp\":1}",104);
 cm.cycle_state=CYCLE_MACHINING;json("{\"sso\":0.5,\"ssoe\":1}",0);assert(spindle_factor==500);
 json("{\"rt\":{\"sso\":1.5,\"sse\":2}}",110);assert(spindle_factor==500);
 cm.machine_state=MACHINE_ALARM;json("{\"sso\":null,\"sse\":null}",0);json("{\"sso\":1}",204);
 txt("$sso",0);txt("$sso=1",204);assert(spindle_factor==500);
 assert(writes==0 && SREG==128);
}
'''
            c=out/'spindle.c';exe=out/'spindle';c.write_text(code)
            subprocess.run(['cc','-std=gnu99','-funsigned-char','-O1','-I'+str(fw),'-I'+str(out),str(c),str(fw/'override.c'),'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
