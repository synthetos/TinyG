"""Real marshalling and parser execution must not expose uninitialized NV values."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function, RUNTIME_STUBS

ROOT=Path(__file__).resolve().parents[2]

class RejectedSetEcho(unittest.TestCase):
    def test_rejected_values_are_null_and_never_execute(self):
        source=(ROOT/'firmware/tinyg/json_parser.c').read_text()
        util=(ROOT/'firmware/tinyg/util.c').read_text()
        code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
#define STAT_NESTED_TXT_CONTAINER 119
#define JSON_INPUT_STRING_MAX 254
#define JSON_PAIRS_MAX 32
#define ritorno(x) do {int s=(x);if(s)return s;}while(0)
typedef int stat_t;
typedef int valueType;
enum {TYPE_SKIP=-2,TYPE_EMPTY=-1,TYPE_NULL,TYPE_BOOLEAN,TYPE_INTEGER,TYPE_STRING,
TYPE_FLOAT,TYPE_SIGNED,TYPE_DATA,TYPE_TXTCON,TYPE_ARRAY,TYPE_PARENT};
#define JTOK_FLAG 0x80
#define JTOK_DSHIFT 4
#define JTOK_DMASK 7
#define JTOK_PARENT 4
#define JTOK_TMASK 3
#define JTOK_DEPTH 0x70
#define JTOK_KEY 0x08
struct {bool motion_sync_allowed,tid_present;} js;
typedef struct nv {int index,valuetype,depth;char token[6],group[4];char *str;
 union {uint32_t value_int;float value_flt;};struct nv *pv,*nx;} nvObj_t;
nvObj_t list[8];int executed;
void nv_reset_nv(nvObj_t *n){n->valuetype=TYPE_EMPTY;n->value_int=UINT32_MAX;n->str=NULL;}
int nv_get_index(char *g,char *t){(void)g;if(!strcmp(t,"xvm"))return 0;if(!strcmp(t,"1mi")||!strcmp(t,"mi"))return 1;if(!strcmp(t,"1"))return 2;return NO_MATCH;}
nvObj_t *nv_next(nvObj_t *n){return n->nx;}
nvObj_t *nv_prev(nvObj_t *n){return n>list?n-1:n;}
#define PSTR(s) (s)
#define strcmp_P strcmp
#define NV_STATUS_REPORT_LEN 30
nvObj_t *nv_next_empty(nvObj_t *n){while(n && n->valuetype!=TYPE_EMPTY)n=n->nx;return n;}
bool nv_group_is_prefixed(char *t){return !strcmp(t,"1");}
bool nv_group_is_typesafe(char *t){(void)t;return true;}
int cfg_is_type(int i){return i==0 ? TYPE_FLOAT : TYPE_INTEGER;}
char *str_unescape(char *s){return s;}
int nv_copy_string(nvObj_t *n,char *s){n->str=s;return 0;}
int _tokenize_json_string(char *s,int size,int pairs){(void)s;(void)size;(void)pairs;return 0;}
int nv_get(nvObj_t *n){(void)n;executed++;return 0;}
typedef stat_t (*fptrCmd)(nvObj_t *);
int set_grp(nvObj_t *n){(void)n;return 0;}
int rt_set_grp(nvObj_t *n){(void)n;return 0;}
#ifndef GET_TABLE_WORD
#define GET_TABLE_WORD(a) 0
#endif
int nv_set(nvObj_t *n){(void)n;executed++;return 0;}
int nv_persist(nvObj_t *n){(void)n;return 0;}
int cm_is_alarmed(void){return 0;}
int controller_dispatch_txt_container(nvObj_t *n,char *s){(void)n;(void)s;return 0;}
char input[256],*wr;
void reset(void){memset(list,0,sizeof(list));for(int i=0;i<8;i++) {nv_reset_nv(&list[i]);list[i].nx=i<7?&list[i+1]:NULL;}wr=input;memset(input,0,sizeof(input));executed=0;}
void key(char *s,int depth,bool parent){while(*s)*wr++=*s++;*wr++=JTOK_FLAG|(depth<<4)|8|(parent?4:0);}
void val(char *s,int depth,int type){while(*s)*wr++=*s++;*wr++=JTOK_FLAG|(depth<<4)|type;}
'''
        code+=function(util,r'stat_t str2float\(')+function(util,r'stat_t str2long\(')
        code+=function(source,r'static char \*_next_token\(')
        code+=function(source,r'static stat_t _marshall_nv_pair\([^;{}]*\)(?=\s*\{)')
        code+=function(source,r'static bool _motion_sync_envelope\([^;{}]*\)(?=\s*\{)')
        code+=RUNTIME_STUBS+function(source,r'static stat_t _json_parser_kernal\([^;{}]*\)(?=\s*\{)')
        code+=r'''
int parse(void){nvObj_t *p=list;return _json_parser_kernal(&p,input);}
int main(void){
 for(int grouped=0;grouped<2;grouped++) {
  reset();if(grouped)key("1",1,true);
  key(grouped?"mi":"1mi",grouped+1,false);val("8.5",grouped+1,TYPE_INTEGER);
  assert(parse()==STAT_VALUE_TYPE_ERROR);assert(executed==0);
  assert(list[grouped].valuetype==TYPE_NULL);
 }
 reset();key("xvm",1,false);val("\"100\"",1,TYPE_STRING);
 assert(parse()==STAT_VALUE_TYPE_ERROR);assert(!executed);assert(list[0].valuetype==TYPE_NULL);
 reset();key("xvm",1,false);val("1e999",1,TYPE_INTEGER);
 assert(parse()==STAT_FLOAT_IS_INFINITE);assert(!executed);assert(list[0].valuetype==TYPE_NULL);
 reset();key("1mi",1,false);val("8",1,TYPE_INTEGER);
 assert(parse()==0 && executed==1);assert(list[0].value_int==8);assert(list[0].valuetype==TYPE_INTEGER);
 reset();key("1mi",1,false);val("n",1,TYPE_NULL);
 assert(parse()==0 && executed==1);assert(list[0].valuetype==TYPE_NULL);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'echo.c';exe=Path(tmp)/'echo';p.write_text(code)
            subprocess.run(['cc','-std=c99','-Wall','-Werror','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(p),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
