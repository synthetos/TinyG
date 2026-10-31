"""E102: the real JSON serializer stays inside its buffer and answers an oversized reply.

json_serialize(), strcat_string(), json_relax() and json_print_response() are extracted from the
firmware and run under ASan with the output buffer allocated at its exact size, so any write past
it fails. Two string values are swept across every split of the buffer (the key of the second one
can land right at out_max), with quotes, backslashes and control bytes in keys and values.
"""
import json
import pathlib
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = pathlib.Path(__file__).resolve().parents[2]

PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define NUL 0
#define PSTR(s) (s)
#define sprintf_P sprintf
#define STAT_OK 0
#define STAT_BUFFER_FULL 13
#define STAT_NO_DISPLAY 19
#define STAT_JSON_SYNTAX_ERROR 111
#define MACHINE_INITIALIZING 0
#define RX_MODE_CHAR 0
#define NV_FOOTER_LEN 20
enum {JV_SILENT, JV_FOOTER, JV_MESSAGES, JV_CONFIGS, JV_LINENUM, JV_VERBOSE};
enum {JSON_SYNTAX_RELAXED, JSON_SYNTAX_STRICT};
enum {TYPE_SKIP=-2,TYPE_EMPTY=-1,TYPE_NULL,TYPE_BOOLEAN,TYPE_INTEGER,TYPE_STRING,
      TYPE_FLOAT,TYPE_SIGNED,TYPE_DATA,TYPE_TXTCON,TYPE_ARRAY,TYPE_PARENT};
enum {NV_TYPE_REPORT,NV_TYPE_GCODE,NV_TYPE_LINENUM,NV_TYPE_CONFIG,NV_TYPE_MESSAGE};
typedef int nvType;
typedef struct nv {int valuetype,depth,precision;char token[6];char *str;uint32_t value_int;float value_flt;
                   struct nv *nx;} nvObj_t;
#define NV_MAX 6
nvObj_t list[NV_MAX];                   /* head, body..., footer */
#define NV_HEAD (&list[0])
#define NV_BODY (&list[1])
#define NV_FOOT (&list[NV_MAX-1])
struct {int json_verbosity,json_syntax;bool tid_present,echo_json_gcode_block,echo_json_linenum,
        echo_json_configs,echo_json_messages;uint32_t tid;} js;
struct {char out_buf[512],*bufp,*saved_buf;unsigned linelen;} cs;
struct {int rx_mode;} xio;
static nvObj_t relinked[NV_MAX];
int cm_get_machine_state(void){return 1;}
unsigned xio_get_line_buffers_available(void){return 10;}
nvObj_t *nv_next(nvObj_t *n){return n->nx;}
int nv_get_type(nvObj_t *n){(void)n;return NV_TYPE_CONFIG;}
nvObj_t *nv_add_string(const char *k,const char *v){(void)k;(void)v;return NULL;}
nvObj_t *nv_add_integer(const char *k,uint32_t v){
    for (int i=1;i<NV_MAX-1;i++) if (list[i].valuetype==TYPE_EMPTY) {
        list[i].valuetype=TYPE_INTEGER;strcpy(list[i].token,k);list[i].value_int=v;return &list[i];}
    return NULL;
}
nvObj_t *nv_relink_nv_list(void){       /* like config.c: drop EMPTY and SKIP pairs */
    nvObj_t *prev=NULL,*first=NULL;
    for (int i=0;i<NV_MAX;i++) {
        if (list[i].valuetype==TYPE_EMPTY||list[i].valuetype==TYPE_SKIP) continue;
        relinked[i]=list[i];relinked[i].nx=NULL;
        if (prev) prev->nx=&relinked[i]; else first=&relinked[i];
        prev=&relinked[i];
    }
    return first;
}
char *strcat_literal_P(char *d,const char *s){strcpy(d,s);return d+strlen(s);}
char *strcat_integer(char *d,uint32_t n){return d+sprintf(d,"%u",n);}
char *strcat_signed(char *d,uint32_t n){return d+sprintf(d,"%d",(int32_t)n);}
char *strcat_float(char *d,float n,int p){return d+sprintf(d,"%.*f",p,n);}
bool prep_float(nvObj_t *n){(void)n;return true;}
bool controller_defer_motion_response(int s,bool j){(void)s;(void)j;return false;}
char *json_relax(char *str);
'''

EXERCISE = r'''
static void reset(void){
    memset(list,0,sizeof(list));
    for (int i=0;i<NV_MAX;i++){list[i].valuetype=TYPE_EMPTY;list[i].nx=(i<NV_MAX-1)?&list[i+1]:NULL;}
    list[0].valuetype=TYPE_PARENT;strcpy(list[0].token,"r");
}
static void add(int i,const char *key,char *value){
    list[i].valuetype=TYPE_STRING;list[i].depth=1;strcpy(list[i].token,key);list[i].str=value;
}
static char a[600],b[600];
static void fill(char *s,int n,int salt){for(int i=0;i<n;i++)s[i]="ab\"\\\x01\x1f z"[(i+salt)%8];s[n]=0;}
int main(void){
    js.json_syntax=JSON_SYNTAX_STRICT;
    /* serializer: every split of two strings, exact-size heap buffer */
    for (int size=64;size<=160;size+=7) for (int la=0;la<size;la++) for (int lb=0;lb<size;lb+=3) {
        reset();fill(a,la,la);fill(b,lb,lb+1);add(1,"a",a);add(2,"q\"k",b);
        list[NV_MAX-1].valuetype=TYPE_ARRAY;list[NV_MAX-1].str="3,0,10";strcpy(list[NV_MAX-1].token,"f");
        char *buf=malloc(size);
        int n=json_serialize(NV_HEAD,buf,size);
        if (n>0){assert(n==(int)strlen(buf));if(size==92)fputs(buf,stdout);}
        else assert(n==-1);
        free(buf);
    }
    /* relaxed syntax keeps escaped quotes and a trailing escaped backslash inside the value */
    js.json_syntax=JSON_SYNTAX_RELAXED;
    reset();add(1,"msg","say \"hi\" \\");
    list[NV_MAX-1].valuetype=TYPE_ARRAY;list[NV_MAX-1].str="3,0,10";strcpy(list[NV_MAX-1].token,"f");
    assert(json_serialize(NV_HEAD,cs.out_buf,sizeof(cs.out_buf))>0);
    assert(!strcmp(cs.out_buf,"{r:{msg:\"say \\\"hi\\\" \\\\\"},f:[3,0,10]}\n"));
    js.json_syntax=JSON_SYNTAX_STRICT;
    /* response too big for out_buf: r{}, tid and footer survive; an OK becomes BUFFER_FULL */
    memset(a,'\x01',100);a[100]=0;                         /* 100 control bytes escape to 600 */
    js.json_verbosity=JV_VERBOSE;js.echo_json_configs=true;xio.rx_mode=1;
    reset();add(1,"msg",a);js.tid_present=true;js.tid=4000000000u;
    json_print_response(STAT_OK);
    reset();add(1,"msg",a);
    json_print_response(STAT_JSON_SYNTAX_ERROR);
    /* and a reply that fits is unchanged */
    reset();add(1,"msg","ok");js.tid_present=true;js.tid=7;
    json_print_response(STAT_OK);
    return 0;
}
'''


class JsonSerializeBounds(unittest.TestCase):
    def test_real_serializer_bounds_and_overflow_reply(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        util = (ROOT / "firmware/tinyg/util.c").read_text()
        parser = (ROOT / "firmware/tinyg/json_parser.c").read_text()
        code = (PRELUDE + function(util, r"char \*strcat_string\([^;{}]*\)\s*\n")
                + function(parser, r"static char \*_serialize_string\([^;{}]*\)(?=\s*\{)")
                + function(parser, r"int16_t json_serialize\([^;{}]*\)(?=\s*\{)")
                + function(parser, r"char \*json_relax\([^;{}]*\)(?=\s*\{)")
                + function(parser, r"void json_print_response\([^;{}]*\)(?=\s*\{)")
                + EXERCISE)
        with tempfile.TemporaryDirectory(prefix="tinyg-e102-") as tmp:
            c, exe = pathlib.Path(tmp) / "bounds.c", pathlib.Path(tmp) / "bounds"
            c.write_text(code)
            subprocess.run([cc, "-std=c99", "-Wall", "-Wno-format", "-Wno-deprecated-declarations",
                            "-Wno-unused-function", "-fsanitize=address,undefined", str(c), "-o", str(exe)],
                           check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        lines = result.stdout.splitlines()
        replies = [json.loads(line) for line in lines]      # every printed line is strict JSON
        self.assertEqual(replies[-3:], [
            {"r": {}, "tid": 4000000000, "f": [3, 13, 10]},
            {"r": {}, "f": [3, 111, 10]},
            {"r": {"msg": "ok"}, "tid": 7, "f": [3, 0, 10]},
        ])
        self.assertTrue(any("q\"k" in r["r"] for r in replies[:-3]), lines[:3])


if __name__ == "__main__":
    unittest.main()
