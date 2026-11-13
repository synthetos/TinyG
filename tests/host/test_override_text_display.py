"""Exercise the actual table-selected text printers, not just the getters."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function

FW=Path(__file__).resolve().parents[2]/'firmware/tinyg'


class OverrideTextDisplay(unittest.TestCase):
    def test_flat_values_have_labels_and_numeric_output(self):
        table=(FW/'config_app.c').read_text()
        text=(FW/'text_parser.c').read_text()
        runtime=(FW/'runtime.c').read_text()
        tokens=('mfo','mfc','mfoe','m48e','mcp','mfe')
        code='''
#include <stdio.h>
typedef unsigned long uint32_t;
typedef struct { float value_flt; unsigned long value_int; } nvObj_t;
#define PROGMEM
#define NV_MESSAGE_LEN 256
#define sprintf_P sprintf
static void text_finalize_message(char *s) { fputs(s, stdout); }
'''
        for name in ('text_print_flt','text_print_int'):
            code+=function(text,r'void '+name+r'\([^;{}]*\)\s*(?=\{)')+'\n'
        added=set();calls=[]
        for token in tokens:
            row=re.search(r'\{\s*""\s*,\s*"'+token+r'"\s*,([^}\n]+)',table)[1].split(',')
            printer=row[2].strip()
            if printer not in added:
                source=runtime if printer.startswith('ov_print_') else text
                body=function(source,r'void '+printer+r'\([^;{}]*\)\s*(?=\{)')
                fmt=re.search(r'text_print_\w+\(nv,\s*(\w+)\)',body)[1]
                code+=re.search(r'static const char '+fmt+r'\[\] PROGMEM = [^;]+;',source)[0]+'\n'+body+'\n'
                added.add(printer)
            calls.append(printer+'(&nv);')
        code+='int main(void) { nvObj_t nv={.value_flt=.5f,.value_int=1};'+''.join(calls)+'return 0;}\n'
        with tempfile.TemporaryDirectory(prefix='override-text-') as tmp:
            c=Path(tmp)/'text.c';exe=Path(tmp)/'text';c.write_text(code)
            subprocess.run(['cc','-Wall','-Werror',str(c),'-o',str(exe)],check=True)
            rows=subprocess.check_output([str(exe)],text=True).splitlines()
        self.assertEqual(len(rows),len(tokens))
        for token,row in zip(tokens,rows):
            with self.subTest(token=token):
                self.assertTrue(row.startswith('['+token+']'),row)
                self.assertEqual(float(row.split()[-1]),.5 if token in ('mfo','mfc') else 1)
