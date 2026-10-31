"""ASan regression for a text query occupying a two-byte RX allocation."""
import pathlib
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT=pathlib.Path(__file__).resolve().parents[2]
PRELUDE=r'''
#include <assert.h>
#include <stdlib.h>
#include <string.h>
typedef int stat_t;
typedef struct {int valuetype;} nvObj_t;
enum {STAT_OK,STAT_NO_DISPLAY,STAT_ALARMED,TYPE_NULL,TYPE_PARENT,MACHINE_ALARM};
#define NUL 0
#define TEXT_RESPONSE 0
#define JSON_RESPONSE 0
#define PSTR(s) (s)
#define memcpy_P memcpy
#define ritorno(call) do {int s=(call);if(s!=STAT_OK)return s;}while(0)
static struct {int machine_state;} cm;
static int cm_is_alarmed(void) { return (cm.machine_state == MACHINE_ALARM) ? STAT_ALARMED : STAT_OK; }
static nvObj_t object;
static int gets;
static nvObj_t *nv_reset_nv_list(int ignored) {(void)ignored;return &object;}
static void sr_run_text_status_report(void) {}
static void help_general(nvObj_t *nv) {(void)nv;}
static stat_t _text_parser_kernal(char *str,nvObj_t *nv) {
    if(*str=='$')str++;
    assert(strcmp(str,"sys")==0);str[0]='s'; /* production parser normalizes in place */
    nv->valuetype=TYPE_PARENT;return STAT_OK;
}
static int nv_get(nvObj_t *nv) {(void)nv;gets++;return STAT_OK;}
static int nv_set(nvObj_t *nv) {(void)nv;assert(0);return STAT_OK;}
static void nv_persist(nvObj_t *nv) {(void)nv;assert(0);}
static void nv_print_list(int a,int b,int c) {(void)a;(void)b;(void)c;}
'''
EXERCISE=r'''
int main(void) {
    char *line=malloc(2);memcpy(line,"$",2);
    assert(text_parser(line)==STAT_OK);assert(gets==1);
    assert(line[0]=='$'&&line[1]==0);free(line);
    char explicit_query[]="$sys";
    assert(text_parser(explicit_query)==STAT_OK);assert(gets==2);
    return 0;
}
'''
class LoneDollar(unittest.TestCase):
    def test_two_byte_line_is_not_expanded_in_place(self):
        source=(ROOT/'firmware/tinyg/text_parser.c').read_text()
        code=PRELUDE+function(source,r'stat_t text_parser\(char \*str\)\s*\n')+EXERCISE
        with tempfile.TemporaryDirectory() as tmp:
            src=pathlib.Path(tmp)/'dollar.c';exe=pathlib.Path(tmp)/'dollar';src.write_text(code)
            subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
if __name__=='__main__': unittest.main()
