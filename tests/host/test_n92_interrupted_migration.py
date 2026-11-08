"""N92: a master settings migration cut off by a power loss at any EEPROM write finishes on the next boot
with the same settings as one that ran straight through.

The real config_init() and migration functions from config.c run against a synthetic EEPROM. The table
makes the layouts collide the way the real ones do: an edge integer row lands on a master float record
that a later edge row still needs (on the real tables, edge 4pm on master 2ma), a master record sits at
its own edge index, and master records lie under the status report slots. Every write is cut in turn,
then each write of the resumed boot is cut again.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function
from config_fallback_source import fallback_source, REFUSING_SETTER

ROOT = Path(__file__).resolve().parents[2]

PRELUDE = r'''
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#define __AVR 1
#define TYPE_INTEGER 2
#define TYPE_DATA 4
#define TYPE_FLOAT 5
#define MILLIMETERS 1
#define STAT_OK 0
#define STAT_COMMAND_NOT_ACCEPTED 106
#define STAT_CONFIG_ASSERTION_FAILURE 90
#define STAT_SETTINGS_RESTORED_TO_DEFAULTS 118
#define STAT_INTERNAL_RANGE_ERROR 21
#define F_INITIALIZE 1
#define F_PERSIST 2
#define TOKEN_LEN 5
#define NUL 0
#define NO_MATCH -1
#define NVM_VALUE_LEN 4
#define E2END 63                        /* 16 records: the table, then room for the journal */
#define PROGMEM
#define PSTR(s) (s)
#define SR_DEFAULTS "defaults"
#define strcpy_P strcpy
#define strncpy_P strncpy
#define strlen_P strlen
#define pgm_read_byte(p) (*(const unsigned char *)(p))
#define sprintf_P(buf, ...) snprintf(buf, 32, __VA_ARGS__)
#define fp_EQ(a,b) (fabs((a)-(b))<.00001)
#define GET_TABLE_FLOAT(f) cfgArray[nv->index].f
typedef int stat_t;
typedef int index_t;
typedef union {uint32_t u; float f;} value_t;
typedef struct {int index, valuetype; char token[6]; union {uint32_t value_int; float value_flt;};} nvObj_t;
struct {int cycle_state;} cm;
struct {char token[6]; int type, flags; float default_value;} cfgArray[] = {
 {"fb",   TYPE_FLOAT,   0, 0},
 {"1ma",  TYPE_INTEGER, 3, 1},     /* master record 1: at its own index */
 {"2ma",  TYPE_INTEGER, 3, 2},     /* master record 4 */
 {"4pm",  TYPE_INTEGER, 3, 0},     /* edge only, lands on master st */
 {"xvm",  TYPE_FLOAT,   3, 1000},  /* master record 6 */
 {"se00", TYPE_INTEGER, 3, 0},     /* lands on master zsn; written by the status report defaults */
 {"st",   TYPE_INTEGER, 3, 1},     /* master record 3 */
 {"zsn",  TYPE_INTEGER, 3, 1},     /* master record 5 */
 {"jv",   TYPE_INTEGER, 3, 4},     /* master record 2 */
 {"rxm",  TYPE_INTEGER, 3, 1},     /* edge only: master's protocol is forced */
 {"adc3", TYPE_FLOAT,   0, 0},
};
#define COUNT ((int)(sizeof(cfgArray)/sizeof(*cfgArray)))
#define RECORDS ((E2END + 1) / NVM_VALUE_LEN)
#define MASTER_NV_ROWS 8
static const char master_tokens[] = "\0001ma\000jv\000st\0002ma\000zsn\000xvm\000\000";
static value_t eeprom[RECORDS], model[COUNT];
static nvObj_t object;
static unsigned writes;
static unsigned cut;                    /* the write that never happens: power is lost before it */
static jmp_buf power;
static bool tx;
static void _set_defa(nvObj_t *nv);
stat_t nv_persist(nvObj_t *nv);
nvObj_t *nv_reset_nv_list(char c) {(void)c; memset(&object, 0, sizeof(object)); return &object;}
void config_init_assertions(void) {}
static bool config_loaded;
static bool cm_persistence_busy(void) {return cm.cycle_state != 0;}
void cm_set_units_mode(int m) {assert(m == MILLIMETERS);}
void rpt_print_loading_configs_message(void) {}
void rpt_print_initializing_message(void) {}
void rpt_exception_P(int s, const char *m) {(void)s; (void)m; assert(0);}
stat_t cm_panic_P(int s, const char *m) {(void)m; assert(0); return s;}
void rpt_exception(int status, const char *token) {(void)status; (void)token; assert(tx);}
bool nv_index_is_single(int i) {return i >= 0 && i < COUNT;}
bool nv_index_lt_groups(int i) {return nv_index_is_single(i);}
index_t nv_index_max(void) {return COUNT;}
index_t nv_get_index(const char *g, const char *t) {
 (void)g;
 for (int i = 0; i < COUNT; i++) if (!strcmp(cfgArray[i].token, t)) return i;
 return NO_MATCH;
}
bool cfg_has_flag(int i, int f) {return cfgArray[i].flags & f;}
int cfg_is_type(int i) {return cfgArray[i].type;}
stat_t read_persistent_value(nvObj_t *nv) {
 assert(nv->index >= 0 && nv->index < RECORDS);
 nv->value_int = eeprom[nv->index].u; return 0;
}
stat_t write_persistent_value(nvObj_t *nv) {
 assert(nv->index >= 0 && nv->index < RECORDS);
 if (eeprom[nv->index].u == nv->value_int) return 0;   /* unchanged values are not written */
 if (++writes == cut) longjmp(power, 1);
 eeprom[nv->index].u = nv->value_int; return 0;
}
stat_t nv_set(nvObj_t *nv) {model[nv->index].u = nv->value_int; return 0;}
stat_t nv_get(nvObj_t *nv) {nv->value_int = model[nv->index].u; return 0;}
void sr_init_status_report_P(const char *s) {   /* the defaults write the list slots */
 if (!*s) return;
 nvObj_t sr = {.index = 5, .value_int = 7}; nv_set(&sr); write_persistent_value(&sr);
}
'''

MAIN = r'''
static void master_board(void) {
 for (int i = 0; i < RECORDS; i++) eeprom[i].u = 0xFFFFFFFF;
 eeprom[0].f = 440.21f;
 eeprom[1].f = 2;       /* 1ma */
 eeprom[2].f = 2;       /* jv: becomes 3 */
 eeprom[3].f = 0;       /* st */
 eeprom[4].f = 1;       /* 2ma */
 eeprom[5].f = 0;       /* zsn: becomes 4 */
 eeprom[6].f = 1500;    /* xvm */
 eeprom[7].u = 0xdeadbeef;
}
static bool boot(unsigned at) {         /* false: the power went before write number `at` */
 memset(model, 0, sizeof(model));
 config_restored.count = 0; config_restored.last = NO_MATCH;
 config_failed.count = 0; config_failed.last = NO_MATCH;
 config_loaded = false; writes = 0; cut = at;
 if (setjmp(power)) return false;
 config_init();
 return true;
}
static void same(const value_t *want_eeprom, const value_t *want_model, unsigned c1, unsigned c2) {
 for (int i = 0; i < COUNT; i++) {
  if (eeprom[i].u != want_eeprom[i].u || ((cfgArray[i].flags & 1) && model[i].u != want_model[i].u)) {
   fprintf(stderr, "cuts %u,%u: row %d (%s) eeprom %08x want %08x model %08x want %08x\n", c1, c2, i,
           cfgArray[i].token, eeprom[i].u, want_eeprom[i].u, model[i].u, want_model[i].u);
   assert(0);
  }
 }
}
int main(void) {
 value_t want_eeprom[RECORDS], want_model[COUNT];
 master_board();
 assert(boot(0));
 unsigned total = writes;
 assert(eeprom[0].f == 10002);
 assert(model[1].u == 2 && model[2].u == 1 && model[3].u == 0 && model[4].f == 1500);
 assert(model[5].u == 7 && model[6].u == 0 && model[7].u == 4 && model[8].u == 3 && model[9].u == 0);
 assert(config_restored.count == 0);
 memcpy(want_eeprom, eeprom, sizeof(eeprom)); memcpy(want_model, model, sizeof(model));
 unsigned resumed = 0;
 for (unsigned c1 = 1; c1 <= total; c1++) {
  master_board();
  assert(!boot(c1));
  value_t after_cut[RECORDS]; memcpy(after_cut, eeprom, sizeof(eeprom));
  assert(boot(0));
  same(want_eeprom, want_model, c1, 0);
  unsigned second = writes;
  for (unsigned c2 = 1; c2 <= second; c2++) {   /* and the resumed boot cut again */
   memcpy(eeprom, after_cut, sizeof(eeprom));
   assert(!boot(c2));
   assert(boot(0));
   same(want_eeprom, want_model, c1, c2);
   resumed++;
  }
 }
 assert(total > 10 && resumed > total);
 printf("%u writes, %u double cuts\n", total, resumed);
}
'''


class InterruptedMasterMigration(unittest.TestCase):
    def test_every_cut_resumes_to_the_same_settings(self):
        source = (ROOT / "firmware/tinyg/config.c").read_text().replace("\r\n", "\n")
        code = PRELUDE
        code += "\n".join(x for x in source.splitlines()
                          if re.match(r"#define (CONFIG_NVM_LAYOUT|MASTER_|NVM_RECORDS)", x)) + "\n"
        code += REFUSING_SETTER + fallback_source(source)
        for name in ("_nvm_layout_is_current", "_nvm_layout_is_legacy", "_write_nvm_layout_marker",
                     "_nvm_layout_is_master", "_master_migration_done", "_write_master_progress",
                     "_master_journaled", "_is_sr_slot", "_migrate_master"):
            code += function(source, r"static (?:bool|void|index_t) " + name + r"\([^;{}]*\)(?=[ \t]*(?://[^\n]*)?\s*\{)") + "\n"
        code += function(source, r"stat_t nv_persist\([^;{}]*\)[^\n]*\n(?=\{)") + "\n"
        code += function(source, r"static void _set_defa\([^;{}]*\)(?=\s*\{)") + "\n"
        code += function(source, r"void config_init\([^;{}]*\)(?=\s*\{)") + "\n"
        code += MAIN
        with tempfile.TemporaryDirectory() as d:
            c = Path(d) / "check.c"
            exe = Path(d) / "check"
            c.write_text(code)
            subprocess.run(["cc", "-Wall", "-Werror", "-Wno-unused-function", "-fsanitize=address,undefined",
                            str(c), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
