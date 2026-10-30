"""Exercise the real text parser: malformed settings must never reach SET/persist."""
import pathlib
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT=pathlib.Path(__file__).resolve().parents[2]
PRELUDE=r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <errno.h>
typedef int stat_t;
enum {STAT_OK,STAT_UNRECOGNIZED_NAME,STAT_BAD_NUMBER_FORMAT,STAT_INPUT_VALUE_RANGE_ERROR,STAT_NO_DISPLAY,STAT_ALARMED};
enum {TYPE_NULL,TYPE_PARENT,TYPE_FLOAT,TYPE_INTEGER};
#define NUL 0
#define TOKEN_LEN 5
#define NO_MATCH -1
#define TEXT_RESPONSE 0
#define JSON_RESPONSE 0
#define MACHINE_ALARM 2
#define strcpy_P strcpy
#define ritorno(call) do {stat_t s=(call);if(s!=STAT_OK)return s;} while(0)
typedef struct {char token[6],group[4];int index,valuetype;float value_flt;uint32_t value_int;} nvObj_t;
static struct {const char *group;} cfgArray[]={{"x"},{"1"}};
static struct {int machine_state;} cm;
static nvObj_t obj;
static unsigned gets,sets,persists;
static nvObj_t *nv_reset_nv_list(int ignored) {(void)ignored;memset(&obj,0,sizeof(obj));return &obj;}
static void sr_run_text_status_report(void) {}
static void help_general(nvObj_t *nv) {(void)nv;}
static void nv_copy_string(nvObj_t *nv,const char *s) {(void)nv;(void)s;}
static int nv_get_index(const char *g,const char *s) {(void)g;return !strcmp(s,"xvm")?0:!strcmp(s,"1mi")?1:NO_MATCH;}
static int cfg_is_type(int i) {return i==0?TYPE_FLOAT:TYPE_INTEGER;}
static int nv_get(nvObj_t *nv) {(void)nv;gets++;return STAT_OK;}
static int nv_set(nvObj_t *nv) {(void)nv;sets++;return STAT_OK;}
static void nv_persist(nvObj_t *nv) {(void)nv;persists++;}
static void nv_print_list(int s,int a,int b) {(void)s;(void)a;(void)b;}
'''
EXERCISE=r'''
static void run(const char *line,int status,int get,int set) {
    char buf[128];strcpy(buf,line);gets=sets=persists=0;
    assert(text_parser(buf)==status);assert(gets==(unsigned)get);
    assert(sets==(unsigned)set&&persists==(unsigned)set);
}
int main(void) {
    const char *queries[]={"$xvm","$xvm=","$xvm ","$xvm= \t","$1mi=","$1mi\t"};
    for(unsigned i=0;i<sizeof(queries)/sizeof(*queries);i++)run(queries[i],STAT_OK,1,0);
    const char *bad[]={"$xvm=abc","$1mi=abc","$xvm=12oops","$1mi=8oops","$xvm=nan","$xvm=inf","$xvm=1e99","$1mi=4294967296","$1mi=-2147483649","$1mi=1.5"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(*bad);i++) {
        char buf[128];strcpy(buf,bad[i]);gets=sets=persists=0;
        assert(text_parser(buf)!=STAT_OK);assert(!gets&&!sets&&!persists);
    }
    run("$xvm = 1.25e2 \t",STAT_OK,0,1);assert(obj.value_flt==125);
    run("$xvm=-2.5",STAT_OK,0,1);assert(obj.value_flt==-2.5);
    run("$1mi=8 \t",STAT_OK,0,1);assert(obj.value_int==8);
    run("$1mi=4294967295",STAT_OK,0,1);assert(obj.value_int==UINT32_MAX);
    run("$1mi=-2147483648",STAT_OK,0,1);assert(obj.value_int==0x80000000UL);
    run("$not=3",STAT_UNRECOGNIZED_NAME,0,0);
    return 0;
}
'''
class TextNumericValidation(unittest.TestCase):
    def test_invalid_values_never_set_or_persist(self):
        source=(ROOT/'firmware/tinyg/text_parser.c').read_text()
        code=PRELUDE+function(source,r'static stat_t _text_parser_kernal\(char \*str, nvObj_t \*nv\)\s*\n')+function(source,r'stat_t text_parser\(char \*str\)\s*\n')+EXERCISE
        with tempfile.TemporaryDirectory() as tmp:
            src=pathlib.Path(tmp)/'text.c';exe=pathlib.Path(tmp)/'text';src.write_text(code)
            subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
if __name__=='__main__': unittest.main()
