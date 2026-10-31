"""A failed recursive txt parse must still serialize a valid JSON response body."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]


class NestedTxtFailure(unittest.TestCase):
    def test_recursive_failure_and_success_serialize(self):
        source = (ROOT / 'firmware/tinyg/json_parser.c').read_text()
        code = r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#define NUL 0
#define GROUP_LEN 3
#define JSON_INPUT_STRING_MAX 254
#define JSON_PAIRS_MAX 36
#define STAT_OK 0
#define STAT_NESTED_TXT_CONTAINER 113
#define PSTR(x) (x)
#define sprintf_P sprintf
#define ritorno(x) do { int status=(x); if(status) return status; } while(0)
typedef int stat_t;
enum { TYPE_SKIP=-2,TYPE_EMPTY=-1,TYPE_NULL=0,TYPE_BOOLEAN,TYPE_INTEGER,TYPE_STRING,
       TYPE_FLOAT,TYPE_SIGNED,TYPE_DATA,TYPE_TXTCON,TYPE_ARRAY,TYPE_PARENT };
#define GET_TABLE_WORD(a) 0
#define NO_MATCH (-1)
typedef struct nvObject {int index,valuetype,depth,precision; char token[8];char *str;
 uint32_t value_int;float value_flt;struct nvObject *nx;} nvObj_t;
nvObj_t root,child,end;
struct {char out_buf[256];} cs;
struct {bool motion_sync_allowed,tid_present;} js;
#define JTOK_FLAG 0x80
#define JTOK_DEPTH 0x70
#define JTOK_DSHIFT 4
#define JTOK_KEY 0x08
#define JTOK_PARENT 0x04
char inner[32];
int _tokenize_json_string(char *s,int size,int pairs){(void)size;(void)pairs;return !strcmp(s,"{fv:@}")?111:0;}
int _marshall_nv_pair(nvObj_t **p,char **s,char *group,int depth,bool safe,...){
 (void)group;(void)depth;(void)safe;
 if(!strcmp(*s,"outer")){(*p)->valuetype=TYPE_TXTCON;(*p)->str=inner;strcpy((*p)->token,"txt");}
 else{(*p)->valuetype=TYPE_FLOAT;(*p)->value_flt=.97;strcpy((*p)->token,"fv");}
 *s="";return 0;
}
int controller_dispatch_txt_container(nvObj_t *n,char *s){(void)n;(void)s;assert(0);return 0;}
int nv_get(nvObj_t *n){(void)n;return 0;}
int nv_set(nvObj_t *n){(void)n;return 0;}
int nv_persist(nvObj_t *n){(void)n;return 0;}
int cm_is_alarmed(void){return 0;}
nvObj_t *nv_next_empty(nvObj_t *n){(void)n;return NULL;}
nvObj_t *nv_relink_nv_list(void){root.nx=child.valuetype==TYPE_EMPTY?&end:&child;return &root;}
nvObj_t *nv_next(nvObj_t *n){return n->nx==&end?NULL:n->nx;}
char *strcat_literal_P(char *d,const char *s){strcpy(d,s);return d+strlen(s);}
char *strcat_integer(char *d,uint32_t n){return d+sprintf(d,"%u",n);}
char *strcat_signed(char *d,uint32_t n){return d+sprintf(d,"%d",(int32_t)n);}
char *strcat_string(char *d,const char *s,size_t n){(void)n;return d+sprintf(d,"\"%s\"",s);}
char *strcat_float(char *d,float n,int precision){(void)precision;return d+sprintf(d,"%.2f",n);}
bool prep_float(nvObj_t *n){(void)n;return true;}
void json_relax(char *s){(void)s;}
'''
        code += function(source, r'static bool _motion_sync_envelope\([^;{}]*\)(?=\s*\{)')
        code += function(source, r'static stat_t _json_parser_kernal\([^;{}]*\)(?=\s*\{)')
        code += function(source, r'static char \*_serialize_string\([^;{}]*\)(?=\s*\{)')
        code += function(source, r'int16_t json_serialize\([^;{}]*\)(?=\s*\{)')
        code += r'''
int main(void){
 for(int bad=1;bad>=0;bad--){
  memset(&root,0,sizeof(root));memset(&child,0,sizeof(child));memset(&end,0,sizeof(end));
  root.valuetype=TYPE_PARENT;strcpy(root.token,"r");child.depth=1;child.nx=&end;
  strcpy(inner,bad?"{fv:@}":"{fv:null}");
  nvObj_t *p=&child;char outer[]="outer";
  assert(_json_parser_kernal(&p,outer)==(bad?111:0));
  assert(json_serialize(&root,cs.out_buf,sizeof(cs.out_buf))>0);
  fputs(cs.out_buf,stdout);
 }
}
'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / 'check.c').write_text(code)
            subprocess.run(['cc', '-std=c99', '-Wall', '-Werror', '-Wno-deprecated-declarations', '-Wno-format',
                            '-fsanitize=address,undefined',
                            str(path / 'check.c'), '-o', str(path / 'check')], check=True)
            output = subprocess.check_output([str(path / 'check')], text=True)
        bodies = [json.loads(line) for line in output.splitlines()]
        self.assertEqual(bodies, [{'r': {}}, {'r': {'fv': .97}}])
