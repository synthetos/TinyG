"""Run production output functions with percent-bearing text; no hardware access."""
from pathlib import Path
import json
import re
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]


class LiteralOutput(unittest.TestCase):
    def test_completed_messages_are_not_format_strings(self):
        report = (ROOT / 'firmware/tinyg/report.c').read_text()
        text = (ROOT / 'firmware/tinyg/text_parser.c').read_text()
        js = (ROOT / 'firmware/tinyg/json_parser.c').read_text()
        code = r'''
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#define PSTR(x) (x)
#define sprintf_P sprintf
#define snprintf_P snprintf
#define printf_P printf
#define STAT_OK 0
#define STAT_NOOP 3
#define STAT_EAGAIN 2
#define TV_SILENT 0
#define JSON_MODE_TXT_OVERRIDE 2
#define NUL 0
#define LF '\n'
#define INCHES 1
#define MODEL 0
#define NV_TYPE_MESSAGE 1
#define LINE_MSG_LEN 128
#define TINYG_FIRMWARE_BUILD 449.01
typedef int stat_t;
typedef struct {char *str;} nvObj_t;
nvObj_t objects[2];
#define NV_BODY objects
struct {int comm_mode; char out_buf[512];} cs;
struct {int text_verbosity;} txt={1};
const char prompt_ok[]="tinyg [%s] ok> ";
const char prompt_err[]="tinyg [%s] err: %s: %s ";
const char *get_status_message(int status) {(void)status;return "error";}
const char *json_relax(const char *s) {return s;}
int cm_get_units_mode(int model) {(void)model;return 0;}
int nv_get_type(nvObj_t *nv) {(void)nv;return NV_TYPE_MESSAGE;}
bool controller_defer_motion_response(int s,bool j){(void)s;(void)j;return false;}
int json_serialize(nvObj_t *nv,char *s,size_t n) {
    (void)nv; snprintf(s,n,"{\"msg\":\"50%% %%s %%n %%%%\"}\n");return strlen(s);
}
'''
        code += function(report, r'static char \*_append_exception_text\(char \*dst, char \*end, const char \*src\)')
        code += function(report, r'stat_t rpt_exception\(stat_t status, const char \*msg\)')
        code += function(text, r'void text_finalize_message\(char \*msg\)')
        code += function(text, r'void text_response\(const stat_t status, char \*buf\)')
        code += function(js, r'void json_print_object\(nvObj_t \*nv\)')
        code += r'''
int main(void) {
    rpt_exception(131,"50% %s %n %%");
    char message[]="50% %s %n %%\n";
    text_finalize_message(message);
    objects[1].str="50% %s %n %%";
    text_response(0,message);
    json_print_object(NULL);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / 'output.c'
            c.write_text(code)
            exe = Path(tmp) / 'output'
            subprocess.run(['cc', '-std=gnu99', '-Wall', '-Werror', '-Wno-deprecated-declarations', '-fsanitize=address,undefined','-fstrict-flex-arrays=3',
                            str(c), '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], check=True, text=True, capture_output=True)
        lines = result.stdout.splitlines()
        self.assertEqual(json.loads(lines[0])['er']['msg'], 'error - 50% %s %n %%')
        self.assertEqual(lines[1], '50% %s %n %%')
        self.assertEqual(lines[2], 'tinyg [mm] ok> 50% %s %n %%')
        self.assertEqual(json.loads(lines[3])['msg'], '50% %s %n %%')
        # These modules emit completed strings, never caller-controlled printf formats.
        for source in (report, text, js):
            self.assertIsNone(re.search(r'\bprintf\(\s*[^"\s]', source))


if __name__ == '__main__':
    unittest.main()
