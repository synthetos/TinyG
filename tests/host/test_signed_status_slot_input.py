"""Status slots preserve their signed -1 sentinel without widening unsigned settings."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]

class SignedStatusSlots(unittest.TestCase):
    def test_signed_marshalling_and_slot_validation(self):
        parser = (ROOT / 'firmware/tinyg/json_parser.c').read_text()
        util = (ROOT / 'firmware/tinyg/util.c').read_text()
        report = (ROOT / 'firmware/tinyg/report.c').read_text()
        table = (ROOT / 'firmware/tinyg/config_app.c').read_text()
        for slot in range(30):
            self.assertRegex(table, r'"se%02d",\s*_sip\b' % slot)
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#define NUL 0
#define TOKEN_LEN 5
#define GROUP_LEN 3
#define NO_MATCH -1
#define STAT_OK 0
#define STAT_UNRECOGNIZED_NAME 100
#define STAT_INPUT_EXCEEDS_MAX_LENGTH 107
#define STAT_BAD_NUMBER_FORMAT 108
#define STAT_INPUT_VALUE_RANGE_ERROR 109
#define STAT_JSON_TOO_MANY_PAIRS 110
#define STAT_JSON_SYNTAX_ERROR 111
#define STAT_VALUE_TYPE_ERROR 116
#define STAT_FLOAT_IS_NAN 117
#define STAT_FLOAT_IS_INFINITE 118
#define PSTR(s) (s)
#define strcmp_P strcmp
#define NV_STATUS_REPORT_LEN 30
#define ritorno(x) do {int s=(x);if(s)return s;}while(0)
#define JTOK_FLAG 0x80
#define JTOK_DSHIFT 4
#define JTOK_DMASK 7
#define JTOK_PARENT 4
#define JTOK_TMASK 3
typedef int stat_t;
typedef int index_t;
typedef int valueType;
enum {TYPE_SKIP=-2,TYPE_EMPTY=-1,TYPE_NULL,TYPE_BOOLEAN,TYPE_INTEGER,TYPE_STRING,
TYPE_FLOAT,TYPE_SIGNED,TYPE_DATA,TYPE_TXTCON,TYPE_ARRAY,TYPE_PARENT};
typedef struct nv {int index,valuetype,depth;char token[6],group[4];char *str;
 union {uint32_t value_int;float value_flt;};struct nv *nx;} nvObj_t;
int row_type, writes;
void nv_reset_nv(nvObj_t *n){n->valuetype=TYPE_EMPTY;n->value_int=1234567;n->str=NULL;}
int nv_get_index(char *g,char *t){(void)g;(void)t;return 1;}
nvObj_t *nv_next(nvObj_t *n){return n->nx;}
nvObj_t *nv_prev(nvObj_t *n){(void)n;return NULL;}
bool nv_group_is_prefixed(char *t){(void)t;return false;}
bool nv_group_is_typesafe(char *t){(void)t;return true;}
int cfg_is_type(int i){(void)i;return row_type;}
char *str_unescape(char *s){return s;}
int nv_copy_string(nvObj_t *n,char *s){n->str=s;return 0;}
int nv_index_max(void){return 300;}
bool cfg_is_reportable(int i){return i>=0&&i<300;}
int set_u16(nvObj_t *n){(void)n;writes++;return 0;}
'''
        for source, pattern in [(util,r'stat_t str2float\('),(util,r'stat_t str2long\('),
            (parser,r'static char \*_next_token\('),
            (parser,r'static stat_t _marshall_nv_pair\([^;{}]*\)(?=\s*\{)'),
            (report,r'static bool _valid_status_report_index\('),(report,r'stat_t sr_set_se\(')]:
            code += function(source,pattern)
        code += r'''
int parse(char *value,int type,nvObj_t *n){
 char input[100],group[4]="";snprintf(input,sizeof(input),"se29%c%s%c",0x98,value,0x92);
 char *p=input;nvObj_t *np=n;memset(n,0,sizeof(*n));row_type=type;
 return _marshall_nv_pair(&np,&p,group,0,true,false);
}
int main(void){
 nvObj_t n;
 char *disabled[]={"-1","-1.0","-10e-1"};
 for(unsigned i=0;i<3;i++) {assert(parse(disabled[i],TYPE_SIGNED,&n)==0);assert(n.valuetype==TYPE_SIGNED);assert(n.value_int==UINT32_MAX);assert(sr_set_se(&n)==0);}
 assert(parse("42.0",TYPE_SIGNED,&n)==0 && n.value_int==42);assert(sr_set_se(&n)==0);
 assert(parse("-2",TYPE_SIGNED,&n)==0);int before=writes;assert(sr_set_se(&n)==109 && writes==before);
 assert(parse("2147483647",TYPE_SIGNED,&n)==0 && n.value_int==INT32_MAX);
 assert(parse("-2147483648",TYPE_SIGNED,&n)==0 && n.value_int==0x80000000u);
 char *bad[]={"2147483648","-2147483649","-1.5","0x10"};
 for(unsigned i=0;i<4;i++)assert(parse(bad[i],TYPE_SIGNED,&n)!=0);
 assert(parse("-1",TYPE_INTEGER,&n)!=0);
 assert(parse("4294967295",TYPE_INTEGER,&n)==0 && n.value_int==UINT32_MAX);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'slots.c';exe=Path(tmp)/'slots';p.write_text(code)
            subprocess.run(['cc','-std=c99','-Wall','-Werror','-fsanitize=address,undefined',str(p),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
