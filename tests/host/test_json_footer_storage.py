"""A full shared string pool must not corrupt the response footer."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]

class JsonFooterStorage(unittest.TestCase):
    def test_footer_survives_exhausted_shared_storage(self):
        source=(ROOT/'firmware/tinyg/json_parser.c').read_text()
        code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define PSTR(s) (s)
#define sprintf_P sprintf
#define NUL 0
#define JV_SILENT 0
#define STAT_OK 0
#define STAT_BUFFER_FULL 13
#define STAT_NO_DISPLAY 250
#define STAT_JSON_SYNTAX_ERROR 111
#define MACHINE_INITIALIZING 0
#define RX_MODE_CHAR 0
#define NV_FOOTER_LEN 20
enum {TYPE_SKIP=-2,TYPE_EMPTY=-1,TYPE_NULL,TYPE_BOOLEAN,TYPE_INTEGER,TYPE_STRING,
TYPE_FLOAT,TYPE_SIGNED,TYPE_DATA,TYPE_TXTCON,TYPE_ARRAY,TYPE_PARENT};
enum {NV_TYPE_REPORT,NV_TYPE_GCODE,NV_TYPE_LINENUM,NV_TYPE_CONFIG,NV_TYPE_MESSAGE};
typedef int nvType;
typedef struct nv {int valuetype,depth;char token[6];char *str;uint32_t value_int;struct nv *nx;} nvObj_t;
nvObj_t head,body,foot;
#define NV_HEAD (&head)
#define NV_BODY (&body)
#define NV_FOOT (&foot)
struct {int json_verbosity;bool tid_present,echo_json_gcode_block,echo_json_linenum,echo_json_configs,echo_json_messages;uint32_t tid;} js;
struct {char out_buf[512],*bufp,*saved_buf;unsigned linelen;} cs;
struct {int rx_mode;} xio;
char seen[20];unsigned copies;
int cm_get_machine_state(void){return 1;}
void json_relax(char *s){(void)s;}
nvObj_t *nv_add_string(const char *k,const char *v){(void)k;(void)v;return NULL;}
nvObj_t *nv_add_integer(const char *k,uint32_t v){(void)k;(void)v;return NULL;}
nvObj_t *nv_next(nvObj_t *n){return n->nx;}
int nv_get_type(nvObj_t *n){(void)n;return NV_TYPE_CONFIG;}
unsigned xio_get_line_buffers_available(void){return 10;}
int nv_copy_string(nvObj_t *n,const char *s){(void)n;(void)s;copies++;return 107;}
int serialize_ret;
int json_serialize(nvObj_t *n,char *s,int z){(void)n;(void)s;(void)z;assert(foot.str);strcpy(seen,foot.str);return serialize_ret;}
'''
        code+=function(source,r'void json_print_response\([^;{}]*\)(?=\s*\{)')
        code+=r'''
int main(void){
 js.json_verbosity=4;body.valuetype=TYPE_EMPTY;foot.str="End";xio.rx_mode=1;
 json_print_response(0);assert(!strcmp(seen,"3,0,10"));assert(copies==0);assert(foot.str==NULL);assert(foot.valuetype==TYPE_EMPTY);
 foot.str="G21";xio.rx_mode=RX_MODE_CHAR;cs.linelen=247;
 json_print_response(111);assert(!strcmp(seen,"2,111,247"));assert(cs.linelen==0);assert(copies==0);assert(foot.str==NULL);assert(foot.valuetype==TYPE_EMPTY);
 /* E102: a reply too big for out_buf still answers the line, keeping its tid and footer */
 foot.str="x";xio.rx_mode=1;serialize_ret=-1;js.tid_present=true;js.tid=4000000000u;
 json_print_response(0);assert(!strcmp(cs.out_buf,"{\"r\":{},\"tid\":4000000000,\"f\":[3,13,10]}\n"));assert(!js.tid_present);
 json_print_response(0);assert(!strcmp(cs.out_buf,"{\"r\":{},\"f\":[3,13,10]}\n"));
}
'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'check.c').write_text(code)
            subprocess.run(['cc','-std=c99','-Wno-format-security','-Wno-deprecated-declarations','-fsanitize=address,undefined',str(p/'check.c'),'-o',str(p/'check')],check=True)
            subprocess.run([str(p/'check')],check=True)
