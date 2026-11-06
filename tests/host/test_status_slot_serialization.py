"""Exercise actual status-slot getter, JSON serializer and signed formatter offline.

Run: python3 tests/host/test_status_slot_serialization.py
"""
import json
import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


def function(filename, name):
    source = (ROOT / "firmware/tinyg" / filename).read_text()
    match = re.search(r"(?m)^(?:static )?(?:char \*|stat_t |int16_t )" + name + r"\([^\n]*\)\n\{", source)
    if not match:
        raise AssertionError(f"missing function {name}")
    return source[match.start():source.index("\n}", match.end()) + 2]


PRELUDE = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>
#define PSTR(s) s
#define sprintf_P sprintf
#define STAT_OK 0
#define NUL 0
typedef int stat_t;
enum {TYPE_NULL,TYPE_BOOLEAN,TYPE_INTEGER,TYPE_STRING,TYPE_FLOAT,
      TYPE_SIGNED,TYPE_DATA,TYPE_TXTCON,TYPE_ARRAY,TYPE_PARENT};
typedef struct nv {int valuetype,depth,precision;char token[8];char *str;
    union {uint32_t value_int;float value_flt;};void *target;struct nv *nx;} nvObj_t;
#define GET_TABLE_WORD(f) (nv->f)
struct {char out_buf[256];} cs;
static nvObj_t nodes[3];
static int16_t slot;
nvObj_t *nv_relink_nv_list(void){return nodes;}
nvObj_t *nv_next(nvObj_t *n){return n->nx;}
char *json_relax(char *s){return s;}
char *strcat_literal_P(char *d,const char *s){strcpy(d,s);return d+strlen(s);}
char *strcat_float(char *d,float f,int precision){assert(0);return d;}
bool prep_float(nvObj_t *n){assert(0);return false;}
/* The unsigned formatter is not under test; match the host ABI for its helper. */
char *strcat_integer(char *d,uint32_t v){return d+sprintf(d,"%lu",(unsigned long)v);}
'''

EXERCISE = r'''
static void init(void){
    memset(nodes,0,sizeof(nodes));
    strcpy(nodes[0].token,"r");nodes[0].valuetype=TYPE_PARENT;nodes[0].nx=&nodes[1];
    strcpy(nodes[1].token,"se00");nodes[1].depth=1;nodes[1].target=&slot;nodes[1].nx=&nodes[2];
    strcpy(nodes[2].token,"f");nodes[2].valuetype=TYPE_ARRAY;nodes[2].str="3,0,10";
}
static void output(void){int n=json_serialize(nodes,cs.out_buf,sizeof(cs.out_buf));assert(n>0);fputs(cs.out_buf,stdout);}
int main(void){
    int16_t slots[]={0,9,406,-1};
    for(unsigned i=0;i<sizeof(slots)/sizeof(slots[0]);i++){
        init();slot=slots[i];assert(sr_get_se(&nodes[1])==STAT_OK);
        assert(nodes[1].valuetype==TYPE_SIGNED);output();
    }
    int32_t signed_values[]={INT32_MIN,-32768,-1,0,1,32767,INT32_MAX};
    for(unsigned i=0;i<sizeof(signed_values)/sizeof(signed_values[0]);i++){
        init();nodes[1].valuetype=TYPE_SIGNED;nodes[1].value_int=signed_values[i];output();
    }
    return 0;
}
'''


class StatusSlotSerializationTests(unittest.TestCase):
    def test_getters_emit_valid_signed_json(self):
        compiler = shutil.which("cc")
        if not compiler:
            self.skipTest("host C compiler not installed")
        code = PRELUDE + "\n".join(function(filename, name) for filename, name in (
            ("config.c", "get_u16"), ("report.c", "sr_get_se"),
            ("util.c", "strcat_signed"), ("util.c", "strcat_string"),
            ("json_parser.c", "_serialize_string"), ("json_parser.c", "json_serialize"))) + EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-status-json-") as directory:
            path = pathlib.Path(directory)
            (path / "check.c").write_text(code)
            build = subprocess.run([compiler, "-std=c99", "-fsanitize=address,undefined","-fstrict-flex-arrays=3",
                                    str(path / "check.c"), "-o", str(path / "check")],
                                   capture_output=True, text=True)
            self.assertEqual(build.returncode, 0, build.stderr)
            result = subprocess.run([str(path / "check")], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            values = [json.loads(line)["r"]["se00"] for line in result.stdout.splitlines()]
            self.assertEqual(values, [0, 9, 406, -1, -2147483648, -32768, -1, 0, 1, 32767, 2147483647])
            self.assertTrue(all(type(value) is int for value in values))


if __name__ == "__main__":
    unittest.main()
