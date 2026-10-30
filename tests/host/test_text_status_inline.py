"""Check text SR wire format using the actual text-list dispatcher and formatter."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]

class TextStatusInline(unittest.TestCase):
    def test_inline_and_human_readable_modes(self):
        source=(ROOT/'firmware/tinyg/text_parser.c').read_text()
        code=r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#define sprintf_P sprintf
struct {char out_buf[512];} cs;
#define PSTR(s) (s)
#define printf_P printf
#define NV_BODY_LEN 36
#define TEXT_NO_DISPLAY 0
#define TEXT_RESPONSE 1
#define TEXT_INLINE_PAIRS 2
#define NUL 0
typedef int stat_t;
enum {TYPE_SKIP=-2,TYPE_EMPTY=-1,TYPE_NULL,TYPE_BOOLEAN,TYPE_INTEGER,TYPE_STRING,
TYPE_FLOAT,TYPE_SIGNED,TYPE_DATA,TYPE_TXTCON,TYPE_ARRAY,TYPE_PARENT};
typedef struct nv {int valuetype;char token[6];int precision;float value_flt;uint32_t value_int;char *str;struct nv *nx;} nvObj_t;
nvObj_t list[9];
#define NV_BODY list
nvObj_t *nv_next(nvObj_t *n){return n->nx;}
bool prep_float(nvObj_t *n){(void)n;return true;}
void nv_print(nvObj_t *n){(void)n;puts("formatted value");}
'''
        code+=function((ROOT/'firmware/tinyg/util.c').read_text(),r'char fntoa\([^;{}]*\)(?=\s*\{)')
        if 'static void _text_print_inline_pairs' in source:
            code+=function(source,r'static void _text_print_inline_pairs\([^;{}]*\)(?=\s*\{)')
        code+=function(source,r'void text_print_list\([^;{}]*\)(?=\s*\{)')
        code+=r'''
int main(void){
 for(int i=0;i<8;i++)list[i].nx=&list[i+1];
 list[0].valuetype=TYPE_PARENT;strcpy(list[0].token,"sr");
 list[1].valuetype=TYPE_FLOAT;strcpy(list[1].token,"posx");list[1].value_flt=-.125f;list[1].precision=3;
 list[2].valuetype=TYPE_SKIP;
 list[3].valuetype=TYPE_INTEGER;strcpy(list[3].token,"stat");list[3].value_int=3;
 list[4].valuetype=TYPE_INTEGER;strcpy(list[4].token,"line");list[4].value_int=4294967295u;
 list[5].valuetype=TYPE_SIGNED;strcpy(list[5].token,"in1");list[5].value_int=(uint32_t)-1;
 list[6].valuetype=TYPE_BOOLEAN;strcpy(list[6].token,"home");list[6].value_int=1;
 list[7].valuetype=TYPE_STRING;strcpy(list[7].token,"msg");list[7].str="100% ready";
 list[8].valuetype=TYPE_EMPTY;
 text_print_list(0,TEXT_INLINE_PAIRS);
 list[2].valuetype=TYPE_EMPTY; // explicit '?' still uses the human-readable dispatch
 text_print_list(0,TEXT_RESPONSE);
 text_print_list(0,TEXT_NO_DISPLAY);
}
'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'check.c').write_text(code)
            subprocess.run(['cc','-std=c99','-Wall','-Werror','-Wno-deprecated-declarations','-fsanitize=address,undefined',str(p/'check.c'),'-o',str(p/'check')],check=True)
            out=subprocess.check_output([str(p/'check')],text=True)
        self.assertEqual(out,'posx:-0.125,stat:3,line:4294967295,in1:-1,home:1,msg:100% ready\nformatted value\n')
