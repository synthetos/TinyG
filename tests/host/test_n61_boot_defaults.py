"""N61: a setting that cannot load at boot never stays stale or zero, and the board says which settings it restored.

The real report lists, _load_config_default(), _load_config_fallback() and config_report_load_failure() (config.c)
run with the real JSON verbosity setter json_set_jv() (json_parser.c) in the table:
  - nv_set() refuses the default (a gate, as a draft of N60 did for every setting at boot): the row's own setter runs,
    so its side effects happen - jv=3 also turns on the footer, message and config echo flags;
  - a setter that refuses its own default leaves the setting as it was and is reported separately (status 90),
    never as restored;
  - a stored value that fails and whose default applies is reported as restored (status 118); more than eight are
    named eight at a time with "(+N more)"; each list is reported once; nothing is written to EEPROM.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from config_fallback_source import fallback_source
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]

PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define NUL 0
#define NO_MATCH 65535
#define TOKEN_LEN 5
#define TYPE_FLOAT 1
#define TYPE_INTEGER 2
#define PSTR(s) (s)
#define strncpy_P strncpy
#define sprintf_P(buf, ...) snprintf(buf, 32, __VA_ARGS__)
typedef int stat_t;
typedef uint16_t index_t;
enum {STAT_OK = 0, STAT_CONFIG_ASSERTION_FAILURE = 90, STAT_INPUT_VALUE_RANGE_ERROR = 110,
      STAT_SETTINGS_RESTORED_TO_DEFAULTS = 118};
enum {JV_SILENT = 0, JV_FOOTER, JV_MESSAGES, JV_CONFIGS, JV_LINENUM, JV_VERBOSE};
typedef struct {index_t index; char token[6]; float value_flt; uint32_t value_int; int valuetype;} nvObj_t;
typedef stat_t (*fptrCmd)(nvObj_t *nv);
static struct { uint8_t json_verbosity; bool echo_json_footer, echo_json_messages, echo_json_configs,
                echo_json_linenum, echo_json_gcode_block; } js;
'''

TABLE = r'''
static float xvm;
static int refused_calls;
static stat_t set_xvm(nvObj_t *nv) { xvm = nv->value_flt; return STAT_OK; }
static stat_t refuses_own_default(nvObj_t *nv) { (void)nv; refused_calls++; return STAT_INPUT_VALUE_RANGE_ERROR; }
typedef struct {char token[6]; fptrCmd set; float def; int type;} row_t;
static row_t cfgArray[16] = {
    {"jv", json_set_jv, 3, TYPE_INTEGER}, {"xvm", set_xvm, 800, TYPE_FLOAT}, {"1mi", refuses_own_default, 8, TYPE_INTEGER},
};
#define GET_TABLE_WORD(a) cfgArray[nv->index].a
#define GET_TABLE_FLOAT(a) cfgArray[nv->index].def
static int cfg_is_type(index_t i) { return cfgArray[i].type; }
static bool gate_refuses;
static int persists, reports, reported_status[4];
static char reported[4][96];
static stat_t nv_set(nvObj_t *nv) { return gate_refuses ? 117 : ((fptrCmd)GET_TABLE_WORD(set))(nv); }
static void nv_persist(nvObj_t *nv) { (void)nv; persists++; }
static void rpt_exception(int status, const char *msg) { reported_status[reports] = status;
                                                         strncpy(reported[reports], msg, 95); reports++; }
'''

EXERCISE = r'''
static nvObj_t at(index_t i) { nvObj_t nv; memset(&nv, 0, sizeof(nv)); nv.index = i; return nv; }

int main(void) {
    /* boot with every setting refused by a gate in nv_set(): the real jv setter still runs, with its side effects */
    gate_refuses = true;
    nvObj_t nv = at(0);
    _load_config_fallback(&nv);
    assert(js.json_verbosity == 3 && js.echo_json_footer && js.echo_json_messages && js.echo_json_configs);
    assert(!js.echo_json_linenum && !js.echo_json_gcode_block);
    nv = at(1); _load_config_fallback(&nv);
    assert(xvm == 800);
    /* a setter that refuses its own default: left as it was, reported apart, never as restored */
    nv = at(2); _load_config_fallback(&nv);
    assert(refused_calls == 1);
    assert(persists == 0);                                          /* loading never writes EEPROM */
    config_report_load_failure();
    assert(reports == 2);
    assert(reported_status[0] == STAT_SETTINGS_RESTORED_TO_DEFAULTS && strcmp(reported[0], "jv,xvm") == 0);
    assert(reported_status[1] == STAT_CONFIG_ASSERTION_FAILURE && strcmp(reported[1], "1mi") == 0);
    config_report_load_failure();
    assert(reports == 2);                                           /* each list once */

    /* twelve restored settings: eight names, then the count; a setting failing twice is counted once */
    gate_refuses = false;
    for (int i = 3; i < 15; i++) { snprintf(cfgArray[i].token, 6, "t%d", i); cfgArray[i].set = set_xvm;
                                   cfgArray[i].def = i; cfgArray[i].type = TYPE_FLOAT; }
    for (index_t i = 3; i < 15; i++) { nv = at(i); _load_config_fallback(&nv); _load_config_fallback(&nv); }
    config_report_load_failure();
    assert(reports == 3 && reported_status[2] == STAT_SETTINGS_RESTORED_TO_DEFAULTS);
    assert(strcmp(reported[2], "t3,t4,t5,t6,t7,t8,t9,t10 (+4 more)") == 0);
    return 0;
}
'''


class BootDefaults(unittest.TestCase):
    def test_restored_settings_run_their_setters_and_are_reported(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        fw = ROOT / "firmware/tinyg"
        jv = function((fw / "json_parser.c").read_text().replace("\r\n", "\n"), r"stat_t json_set_jv\(nvObj_t \*nv\)")
        code = PRELUDE + "static " + jv + "\n" + TABLE + fallback_source((fw / "config.c").read_text()) + EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-n61-") as tmp:
            c, exe = Path(tmp) / "n61.c", Path(tmp) / "n61"
            c.write_text(code)
            subprocess.run([cc, "-std=gnu99", "-Wall", "-Werror", "-Wno-unused-function",
                            "-fsanitize=address,undefined", str(c), "-o", str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
