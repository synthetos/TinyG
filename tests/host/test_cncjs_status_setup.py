"""Exercise real NV marshalling with CNCjs capability and SR requests under sanitizers."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT = Path(__file__).resolve().parents[2]

PRELUDE = r''' 
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#define NUL 0
#define TOKEN_LEN 5
#define GROUP_LEN 3
#define NO_MATCH -1
#define STAT_OK 0
#define STAT_UNRECOGNIZED_NAME 100
#define STAT_INPUT_EXCEEDS_MAX_LENGTH 107
#define STAT_JSON_TOO_MANY_PAIRS 110
#define STAT_JSON_SYNTAX_ERROR 111
#define STAT_VALUE_TYPE_ERROR 116
#define STAT_INPUT_VALUE_RANGE_ERROR 109
#define STAT_BAD_NUMBER_FORMAT 108
#define PSTR(s) (s)
#define strcmp_P strcmp
#define NV_STATUS_REPORT_LEN 30
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
typedef struct nv {int index,valuetype,depth;char token[6],group[4];char *str;
 uint32_t value_int;float value_flt;struct nv *pv,*nx;} nvObj_t;
static nvObj_t list[38];
static const char *known[]={"sr","stat","line","vel","feed","unit","coor","momo","plan","path","dist","admo","frmo","tool","posx","posy","posz","posa","posb","posc","mpox","mpoy","mpoz","mpoa","mpob","mpoc","sys"};
void nv_reset_nv(nvObj_t *n){n->valuetype=TYPE_EMPTY;n->value_int=0;}
int nv_get_index(char *g,char *t){if(*g)return NO_MATCH;for(unsigned i=0;i<sizeof(known)/sizeof(*known);i++)if(!strcmp(t,known[i]))return i;return NO_MATCH;}
nvObj_t *nv_next(nvObj_t *n){return n->nx;}
nvObj_t *nv_prev(nvObj_t *n){return n->pv;}
bool nv_group_is_prefixed(char *t){return strcmp(t,"sr")&&strcmp(t,"sys");}
bool nv_group_is_typesafe(char *t){return strcmp(t,"sr");}
int cfg_is_type(int i){(void)i;return TYPE_INTEGER;}
int str2float(char *s,float *f){*f=strtof(s,NULL);return 0;}
int str2long(char *s,uint32_t *v){*v=strtoul(s,NULL,10);return 0;}
char *str_unescape(char *s){return s;}
int nv_copy_string(nvObj_t *n,char *s){n->str=s;return 0;}
static char input[512];static char *wr;
static void reset(void){memset(list,0,sizeof(list));for(int i=0;i<38;i++){
list[i].valuetype=TYPE_EMPTY;list[i].nx=i<37?&list[i+1]:NULL;list[i].pv=i?&list[i-1]:NULL;}wr=input;memset(input,0,sizeof(input));}
static void key(char *s,int depth,bool parent){while(*s)*wr++=*s++;*wr++=JTOK_FLAG|(depth<<4)|8|(parent?4:0);}
static void val(char *s,int depth,int type){while(*s)*wr++=*s++;*wr++=JTOK_FLAG|(depth<<4)|type;}
'''

EXERCISE = r'''
int main(void){
 reset();key("spc",1,false);val("n",1,TYPE_NULL);
 assert(parse()==100);assert(list[0].valuetype==TYPE_NULL);assert(!strcmp(list[0].token,"spc"));
 reset();key("spc",1,false);val("t",1,TYPE_BOOLEAN);
 assert(parse()==100);assert(list[0].valuetype==TYPE_EMPTY);
 reset();key("sr",1,true);
 key("tofx",2,false);val("t",2,TYPE_BOOLEAN);
 for(int i=1;i<26;i++){key((char*)known[i],2,false);val("t",2,TYPE_BOOLEAN);}
 key("cof",2,false);val("f",2,TYPE_BOOLEAN);
 assert(parse()==0);
 for(int i=1;i<26;i++){assert(list[i].index==i);assert(list[i].valuetype==TYPE_BOOLEAN);assert(list[i].value_int==1);}
 assert(list[26].valuetype==TYPE_EMPTY);
 reset();key("sr",1,true);key("zz",2,false);val("42",2,TYPE_INTEGER);assert(parse()==100);
 reset();key("sr",1,true);key("zz",2,true);key("stat",3,false);val("t",3,TYPE_BOOLEAN);assert(parse()==100);
 reset();key("sys",1,true);key("zz",2,false);val("t",2,TYPE_BOOLEAN);assert(parse()==100);
 reset();key("sr",1,true);for(int i=0;i<31;i++){key("stat",2,false);val("t",2,TYPE_BOOLEAN);}assert(parse()==110);
 puts("capability null, compact SR filtering, strict commands and 30-field limit passed");
}
'''

class CncjsStatusSetup(unittest.TestCase):
    def test_capabilities_and_report_selection(self):
        source=(ROOT/'firmware/tinyg/json_parser.c').read_text()
        marshal=function(source,r'static stat_t _marshall_nv_pair\([^;{}]*\)(?=\s*\{)')
        args=', false' if 'bool ignore_unknown' in marshal else ''
        code=PRELUDE+function(source,r'static char \*_next_token\(')+marshal
        code+='\nstatic int parse(void){nvObj_t *p=list;char *s=input;char g[4]="";return _marshall_nv_pair(&p,&s,g,0,true'+args+');}\n'+EXERCISE
        with tempfile.TemporaryDirectory() as d:
            path=Path(d);(path/'check.c').write_text(code)
            subprocess.run(['cc','-std=c99','-fsanitize=address,undefined',str(path/'check.c'),'-o',str(path/'check')],check=True)
            subprocess.run([str(path/'check')],check=True)

    def test_receive_limit_fits_connect_request_without_wrapping(self):
        h=(ROOT/'firmware/tinyg/xio.h').read_text()
        size=int(re.search(r'^#define RX_BUFFER_MIN_SIZE\s+(\d+)',h,re.M).group(1))
        self.assertGreaterEqual(size,247)
        self.assertLessEqual(size,255)  # the current line reader counts bytes in uint8_t
