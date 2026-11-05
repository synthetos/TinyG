"""E167: every text-mode (MSG ...) is printed once, with its own line, whatever came before.

The real _dispatch_kernel() (controller.c), NV list functions (config.c) and text_response()
(text_parser.c) run against the firmware headers. The Gcode parser is replaced by a stand-in that
adds the block's message through the real nv_add_string(), as cm_message() does. Before each line the list is left as a status report, a $ query of one value, or the previous message
left it; blank and whitespace-only lines (and the second byte of a CRLF) must not repeat a message.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / "firmware/tinyg"
DEF = r"[^;{}]*\)(?=\s*(?://[^\n]*\s*)?\{)"   # a definition, its trailing comment allowed

PRELUDE = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "tinyg.h"
#include "config.h"
#include "controller.h"
#include "json_parser.h"
#include "text_parser.h"
#include "canonical_machine.h"
#include "report.h"
#include "util.h"
#define NUL (char)0x00
#define ENQ (char)0x05
#define TAB (char)0x09
#define SPC (char)0x20
static const char prompt_ok[] = "tinyg [%s] ok> ";
static const char prompt_err[] = "tinyg [%s] err: %s: %s ";
controller_t cs;
nvList_t nvl;
nvStr_t nvStr;
jsSingleton_t js;
txtSingleton_t txt;
enum { SYNC_OFF, SYNC_REQUESTED, SYNC_WAITING };
static struct { uint8_t state; } motion_sync;
uint8_t cm_get_units_mode(const GCodeState_t *gcode_state) { (void)gcode_state; return MILLIMETERS; }
bool controller_defer_motion_response(stat_t status, bool json) { (void)status; (void)json; return false; }
char *get_status_message(stat_t status) { (void)status; return "error"; }
index_t nv_get_index(const char *group, const char *token) { (void)group; (void)token; return 0; }
void cm_request_feedhold(void) {}
void cm_request_end_hold(void) {}
void cm_request_queue_flush(void) {}
void controller_request_enquiry(void) {}
void json_parser(char *str) { (void)str; }
void nv_print_list(stat_t status, uint8_t text_flags, uint8_t json_flags) { (void)status; (void)text_flags; (void)json_flags; }
stat_t sr_request_status_report(uint8_t request_type) { (void)request_type; return STAT_OK; }
stat_t text_parser(char *str) {                    /* leave the list as a $ query or a status report does */
    nvObj_t *nv = nv_reset_nv_list(str[0] == '?' ? "sr" : "");
    int n = (str[0] == '?') ? 6 : 1;
    if (str[0] == '?') nv = nv_next(nv);
    for (int i = 0; i < n; i++, nv = nv_next(nv)) { strcpy(nv->token, "posx"); nv->valuetype = TYPE_FLOAT; }
    return STAT_OK;
}
stat_t gc_gcode_parser(char *block) {              /* stand-in: a (MSG ...) comment, as cm_message() adds it */
    char *msg = strstr(block, "(MSG ");
    if (msg) {
        char text[40]; strncpy(text, msg + 5, sizeof(text) - 1); text[sizeof(text) - 1] = 0;
        char *close = strchr(text, ')'); if (close) *close = 0;
        nv_add_string("msg", text);
    }
    return STAT_OK;
}
'''

EXERCISE = r'''
static char out[4096];
static const char *line(const char *input) {        /* run one line; return what it printed */
    static char buf[128];
    strcpy(buf, input); cs.bufp = buf;
    fflush(stdout); memset(out, 0, sizeof(out)); setvbuf(stdout, out, _IOFBF, sizeof(out));
    _dispatch_kernel();
    fflush(stdout); setvbuf(stdout, NULL, _IONBF, 0);
    return out;
}
int main(void) {
    txt.text_verbosity = 1;
    cs.comm_mode = TEXT_MODE;
    nv_reset_nv_list("");
    line("?");
    assert(strstr(line("G21 (MSG after-status)"), "after-status"));
    line("$fb");
    assert(strstr(line("G21 (MSG first-msg)"), "first-msg"));
    const char *second = line("G21 (MSG second-msg)");
    assert(strstr(second, "second-msg") && !strstr(second, "first-msg"));
    assert(!strstr(line(""), "second-msg"));         /* a blank line, or the second byte of a CRLF */
    assert(strstr(line("G21 (MSG third-msg)"), "third-msg"));
    assert(!strstr(line("   "), "third-msg"));      /* whitespace only */
    assert(strstr(line("G21 (MSG fourth-msg)"), "fourth-msg"));
    assert(!strstr(line("\t"), "fourth-msg"));
    assert(!strstr(line("G21"), "msg"));
    for (int i = 0; i < 40; i++) {                   /* the shared string space is not used up */
        char block[48]; snprintf(block, sizeof(block), "G21 (MSG msg-number-%02d)", i);
        char want[16]; snprintf(want, sizeof(want), "msg-number-%02d", i);
        assert(strstr(line(block), want));
    }
    return 0;
}
'''


class TextMessageSlot(unittest.TestCase):
    def test_each_message_printed_once(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        config = (FIRMWARE / "config.c").read_text()
        text = (FIRMWARE / "text_parser.c").read_text()
        controller = (FIRMWARE / "controller.c").read_text()
        code = PRELUDE
        for name in ("nvObj_t \\*nv_prev\\(", "nvObj_t \\*nv_next\\(", "uint8_t nv_get_type\\(",
                     "stat_t nv_copy_string\\(", "static void _add_object_helper\\(",
                     "nvObj_t \\*nv_add_string\\(", "nvObj_t \\*nv_reset_nv_list\\("):
            code += function(config, name + DEF) + "\n"
        code += function(text, r"void text_response\(" + DEF) + "\n"
        code += function(controller, r"static void _dispatch_kernel\(" + DEF) + "\n"
        code += EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-e167-") as tmp:
            tmp = Path(tmp)
            (tmp / "avr").mkdir()
            (tmp / "avr/pgmspace.h").write_text(
                "#define PROGMEM\n#define PSTR(s) (s)\n#define printf_P printf\n#define strcmp_P strcmp\n")
            c, exe = tmp / "msg.c", tmp / "msg"
            c.write_text(code)
            subprocess.run([cc, "-std=gnu99", "-O1", "-fsanitize=address,undefined","-fstrict-flex-arrays=3", "-D__TEXT_MODE",
                            "-D_POSIX_C_SOURCE=200809L", "-Wno-macro-redefined", "-I" + str(tmp),
                            "-I" + str(FIRMWARE), str(c), "-lm", "-o", str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
