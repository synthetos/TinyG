"""N92: a master settings migration cut off by a power loss at any EEPROM byte finishes on the next boot with
the same settings as one that ran straight through.

The real config_init() and migration functions from config.c run against a byte-addressed synthetic EEPROM in
which a record write is four byte writes, as EEPROM_WriteBytes() does. The generated table is as long as the real
one, so checkpoints cross 255 rows, and it makes the layouts collide the way the real ones do: a block of edge
rows 17 below their master records (the motor rows), a block above them (the user data rows), records at their
own index, integer edge rows on master float records still needed later (edge 4pm on master 2ma), and master
records under the status report slots. Every byte write is cut in turn, then a spread of the resumed boot's.
The checkpoint writer is also cut at every byte from random and adversarial slot contents: the newest valid
checkpoint must then be the previous one or the new one, never a mix.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function
from config_fallback_source import fallback_source, REFUSING_SETTER

ROOT = Path(__file__).resolve().parents[2]
EDGE_ROWS, MASTER_ROWS, SR_FIRST = 300, 280, 270


def tables():
    """Edge rows (token, type, flags, default) and master tokens (index -> token), with the kept tokens' values."""
    edge = [("fb", "TYPE_FLOAT", 0, 0)] + [None] * (EDGE_ROWS - 1)
    master = [""] * MASTER_ROWS
    values = {}                                             # master index -> stored float

    def kept(mi, ei, token, kind, value):
        assert master[mi] == "" and edge[ei] is None, (mi, ei)
        master[mi] = token
        edge[ei] = (token, kind, 3, 1)
        values[mi] = value

    kept(2, 2, "hp", "TYPE_INTEGER", 1)                    # at its own index
    kept(3, 3, "hv", "TYPE_INTEGER", 8)
    kept(5, 266, "jv", "TYPE_INTEGER", 2)                  # becomes 3
    kept(6, 12, "zsn", "TYPE_INTEGER", 0)                  # becomes 4
    for i, mi in enumerate(range(40, 100)):                # motors: edge 17 lower
        kept(mi, mi - 17, f"m{i:03d}", "TYPE_INTEGER" if i % 3 else "TYPE_FLOAT", (i % 7) + 1 if i % 3 else i + 0.25)
    for i, mi in enumerate(range(230, 250)):               # far lower
        kept(mi, mi - 110, f"f{i:03d}", "TYPE_FLOAT", 1000 + i)
    for i, mi in enumerate(range(270, 280)):               # under the status report slots, written after every
        kept(mi, mi - 15, f"s{i:03d}", "TYPE_INTEGER", i + 2)  # row checkpoint: only the last one covers them
    kept(210, 297, "q000", "TYPE_FLOAT", 77.5)
    for i, mi in enumerate(range(250, 260)):               # user data: edge higher, past 255
        kept(mi, mi + 35, f"u{i:03d}", "TYPE_INTEGER", i + 3)
    master[7] = "gone"                                     # persisted on master, no edge home
    values[7] = 5
    for i in range(SR_FIRST, SR_FIRST + 10):
        edge[i] = (f"se{i - SR_FIRST:02d}", "TYPE_INTEGER", 3, 0)
    edge[299] = ("rxm", "TYPE_INTEGER", 3, 1)
    for i in range(EDGE_ROWS):
        if edge[i] is None:                                # edge-only rows: defaults, some not persisted
            edge[i] = (f"e{i:03d}", "TYPE_FLOAT" if i % 2 else "TYPE_INTEGER", 0 if i % 11 == 0 else 3, i % 5)
    return edge, master, values


def prelude():
    edge, master, values = tables()
    rows = ",\n".join(f' {{"{t}", {k}, {f}, {d}}}' for t, k, f, d in edge)
    toks = "".join(t + "\\000" for t in master)
    image = "\n".join(f" eeprom_put({mi}, {float(v)}f);" for mi, v in values.items())
    return PRELUDE.replace("@ROWS@", rows).replace("@MASTER_ROWS@", str(MASTER_ROWS)) \
                  .replace("@TOKENS@", toks).replace("@IMAGE@", image)


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
#define E2END 2047
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
typedef struct {int index, valuetype; char token[6]; union {uint32_t value_int; float value_flt;};} nvObj_t;
struct {int cycle_state;} cm;
struct {char token[6]; int type, flags; float default_value;} cfgArray[] = {
@ROWS@
};
#define COUNT ((int)(sizeof(cfgArray)/sizeof(*cfgArray)))
#define RECORDS ((E2END + 1) / NVM_VALUE_LEN)
#define MASTER_NV_ROWS @MASTER_ROWS@
#define MASTER_SR_FIRST 280
#define NV_STATUS_REPORT_LEN 10
#define master_report_tokens master_tokens
static const char master_tokens[] = "@TOKENS@";
static uint8_t eeprom[RECORDS * 4];
static uint32_t model[COUNT];
static nvObj_t object;
static unsigned writes;
static unsigned cut;                    /* the byte write that never happens: power is lost before it */
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
static void byte_write(int addr, uint8_t v) {
 assert(addr >= 0 && addr < RECORDS * 4);
 if (++writes == cut) longjmp(power, 1);
 eeprom[addr] = v;
}
stat_t read_persistent_value(nvObj_t *nv) {
 assert(nv->index >= 0 && nv->index < RECORDS);
 memcpy(&nv->value_int, &eeprom[nv->index * 4], 4); return 0;
}
stat_t write_persistent_value(nvObj_t *nv) {   /* EEPROM_WriteBytes: four byte writes */
 uint32_t old; memcpy(&old, &eeprom[nv->index * 4], 4);
 if (old == nv->value_int) return 0;
 for (int k = 0; k < 4; k++) byte_write(nv->index * 4 + k, (uint8_t)(nv->value_int >> (8 * k)));
 return 0;
}
void write_persistent_byte(index_t index, uint8_t offset, uint8_t value) {
 assert(index >= 0 && index < RECORDS && offset < 4);
 byte_write(index * 4 + offset, value);
}
stat_t nv_set(nvObj_t *nv) {model[nv->index] = nv->value_int; return 0;}
stat_t nv_get(nvObj_t *nv) {nv->value_int = model[nv->index]; return 0;}
void sr_init_status_report_P(const char *s) {   /* the defaults write the list slots */
 if (!*s) {for(int i=0;i<10;i++){nvObj_t n={.index=270+i};read_persistent_value(&n);nv_set(&n);}return;}
 for (int i = 0; i < 10; i++) {nvObj_t sr = {.index = 270 + i, .value_int = 40 + i}; nv_set(&sr); write_persistent_value(&sr);}
}
static void eeprom_put(int i, float f) {memcpy(&eeprom[i * 4], &f, 4);}
static void master_board(void) {
 memset(eeprom, 0xFF, sizeof(eeprom));
 eeprom_put(0, 440.21f);
@IMAGE@
 for(int i=0;i<10;i++)eeprom_put(MASTER_SR_FIRST+i,i<2?2+i:-1);
}
'''

MAIN = r'''
static bool boot(unsigned at) {         /* false: the power went before byte write number `at` */
 memset(model, 0, sizeof(model));
 config_restored.count = 0; config_restored.last = NO_MATCH;
 config_failed.count = 0; config_failed.last = NO_MATCH;
 config_loaded = false; writes = 0; cut = at;
 if (setjmp(power)) return false;
 config_init();
 return true;
}
static uint8_t want_eeprom[RECORDS * 4];
static uint32_t want_model[COUNT];
static void same(unsigned c1, unsigned c2) {
 index_t slot; uint8_t seq;
 assert(_master_checkpoint(&slot, &seq) == 0);                 /* cleared */
 float r0; memcpy(&r0, eeprom, 4); assert(r0 == 10003);
 for (int i = 0; i < CHECKPOINT_A; i++) {                       /* the checkpoint slots may hold other bytes */
  uint32_t e, w; memcpy(&e, &eeprom[i * 4], 4); memcpy(&w, &want_eeprom[i * 4], 4);
  bool m = i < COUNT && (cfgArray[i].flags & 1) && model[i] != want_model[i];
  if (e != w || m) {
   fprintf(stderr, "cuts %u,%u: record %d (%s) eeprom %08x want %08x model %08x want %08x\n", c1, c2, i,
           i < COUNT ? cfgArray[i].token : "-", e, w, i < COUNT ? model[i] : 0, i < COUNT ? want_model[i] : 0);
   assert(0);
  }
 }
}
static uint32_t edge(const char *t) {return model[nv_get_index("", t)];}
static float edge_f(const char *t) {uint32_t u = edge(t); float f; memcpy(&f, &u, 4); return f;}
int main(void) {
 master_board();
 assert(boot(0));
 unsigned total = writes;
 assert(edge("jv") == 3 && edge("zsn") == 4 && edge("rxm") == 0 && edge("hv") == 8 && edge("m001") == 2);
 assert(edge_f("m000") == .25f && edge_f("f019") == 1019 && edge("s009") == 11 && edge("u009") == 12);
 assert(edge_f("q000") == 77.5f && edge("se00") == 2 && edge("se01") == 3 && edge("se03") == (uint32_t)NO_MATCH && config_restored.count == 0);
 memcpy(want_eeprom, eeprom, sizeof(eeprom)); memcpy(want_model, model, sizeof(model));
 unsigned doubles = 0;
 for (unsigned c1 = 1; c1 <= total; c1++) {
  master_board();
  assert(!boot(c1));
  static uint8_t after_cut[RECORDS * 4]; memcpy(after_cut, eeprom, sizeof(eeprom));
  assert(boot(0));
  same(c1, 0);
  unsigned second = writes;
  if (c1 % 23) continue;
  for (unsigned c2 = 1 + c1 % 7; c2 <= second; c2 += 7) {      /* and a spread of cuts in the resumed boot */
   memcpy(eeprom, after_cut, sizeof(eeprom));
   assert(!boot(c2));
   assert(boot(0));
   same(c1, c2);
   doubles++;
  }
 }
 assert(total > 1000 && doubles > total);
 printf("%u byte writes, %u double cuts\n", total, doubles);
}
'''


SLOT_MAIN = r'''
static uint32_t rng = 12345;
static uint32_t rnd(void) {rng = rng * 1103515245u + 12345u; return rng >> 8;}
static uint32_t checkpoint(uint8_t seq, int rows) {
 uint32_t n = seq | ((uint32_t)(rows & 0xFF) << 8) | ((uint32_t)(CHECKPOINT_TAG | (rows >> 8)) << 16);
 return n | (uint32_t)_checkpoint_check(n) << 24;
}
static void put(int slot, uint32_t v) {memcpy(&eeprom[slot * 4], &v, 4);}
int main(void) {
 unsigned trials = 0, torn = 0;
 for (unsigned trial = 0; trial < 300000; trial++) {
  memset(eeprom, 0xFF, sizeof(eeprom));
  int newer = (rnd() & 1) ? CHECKPOINT_A : CHECKPOINT_B, older = CHECKPOINT_A + CHECKPOINT_B - newer;
  uint8_t s = rnd();
  switch (rnd() % 4) {                  /* what the slots hold before the write */
  case 0: break;                                                        /* erased: the first checkpoint */
  case 1: put(newer, checkpoint(s, 1 + rnd() % COUNT)); put(older, rnd() ^ (rnd() << 16)); break;  /* garbage */
  default: put(newer, checkpoint(s, 1 + rnd() % COUNT)); put(older, checkpoint(s - 1, 1 + rnd() % COUNT));
  }
  index_t slot; uint8_t seq;
  int before = _master_checkpoint(&slot, &seq), next = 1 + rnd() % COUNT;
  if (next == before) continue;
  static uint8_t saved[sizeof(eeprom)]; memcpy(saved, eeprom, sizeof(eeprom));
  for (unsigned c = 1; ; c++) {         /* cut before each byte write, then let it complete */
   memcpy(eeprom, saved, sizeof(eeprom));
   writes = 0; cut = c;
   bool done = !setjmp(power);
   if (done) _write_master_checkpoint(next);
   int after = _master_checkpoint(&slot, &seq);
   if (after != before && after != next) {
    fprintf(stderr, "trial %u cut %u: checkpoint %d, want %d or %d\n", trial, c, after, before, next);
    assert(0);
   }
   if (done) {assert(after == next); break;}
   torn++;
  }
  trials++;
 }
 printf("%u checkpoint writes, %u torn\n", trials, torn);
}
'''


class InterruptedMasterMigration(unittest.TestCase):
    def build(self, main):
        source = (ROOT / "firmware/tinyg/config.c").read_text().replace("\r\n", "\n")
        code = prelude()
        code += "\n".join(x for x in source.splitlines()
                          if re.match(r"#define (CONFIG_NVM_LAYOUT|MASTER_|NVM_RECORDS|CHECKPOINT_)", x)) + "\n"
        code += REFUSING_SETTER + fallback_source(source)
        for name in ("_nvm_layout_is_current", "_nvm_layout_is_legacy", "_write_nvm_layout_marker",
                     "_nvm_layout_is_master", "_checkpoint_check", "_checkpoint_rows", "_master_checkpoint",
                     "_write_master_checkpoint", "_clear_master_checkpoints", "_master_journaled",
                     "_master_source_pending", "_is_sr_slot", "_master_report_index", "_migrate_master"):
            code += function(source, r"static (?:bool|void|index_t|uint8_t) " + name +
                             r"\([^;{}]*\)(?=[ \t]*(?://[^\n]*)?\s*\{)") + "\n"
        code += function(source, r"stat_t nv_persist\([^;{}]*\)[^\n]*\n(?=\{)") + "\n"
        code += function(source, r"static void _set_defa\([^;{}]*\)(?=\s*\{)") + "\n"
        code += function(source, r"void config_init\([^;{}]*\)(?=\s*\{)") + "\n"
        code += main
        with tempfile.TemporaryDirectory() as d:
            c = Path(d) / "check.c"
            exe = Path(d) / "check"
            c.write_text(code)
            subprocess.run(["cc", "-O1", "-Wall", "-Werror", "-Wno-unused-function", "-fsanitize=address,undefined",
                            str(c), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_every_byte_cut_resumes_to_the_same_settings(self):
        self.build(MAIN)

    def test_a_torn_checkpoint_leaves_the_previous_one(self):
        self.build(SLOT_MAIN)


if __name__ == "__main__":
    unittest.main()
