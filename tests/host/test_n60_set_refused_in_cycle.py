"""N60: nv_set() and set_grp() refuse a persisted setting while a cycle runs, before any value changes.

The real _refused_in_cycle(), nv_set() and set_grp() (config.c) run against a stub cfgArray: one persisted row, one
RAM-only row (an action, like an rt leaf), the status report list row, and a group. While idle every SET applies; in a
cycle (or with a block queued, or a lone dwell running) the persisted row, the status report list and a group holding a persisted child are refused with 117 and nothing
is written, not even the group's RAM-only child; the RAM-only row still applies. Until config_init() has finished
(it loads EEPROM after commands were already queued, with the machine already READY) every SET applies: refusing them there left a board
with zeroed settings and JSON verbosity 0, silent (bench 2, 2026-11-04).
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]

PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
typedef int stat_t;
typedef int index_t;
enum {STAT_OK = 0, STAT_INTERNAL_RANGE_ERROR = 3, STAT_UNRECOGNIZED_NAME = 100, STAT_SETTING_REFUSED_IN_CYCLE = 117};
enum {CYCLE_OFF = 0, CYCLE_MACHINING = 1};
enum {TEXT_MODE = 0, JSON_MODE = 1};
enum {TYPE_EMPTY = -1, TYPE_NULL = 0, TYPE_INTEGER = 2, TYPE_PARENT = 6};
#define F_PERSIST 0x04
#define NV_MAX_OBJECTS 8
typedef struct nvObject nvObj_t;
typedef stat_t (*fptrCmd)(nvObj_t *nv);
struct nvObject { nvObj_t *nx; index_t index; int valuetype; int value_int; int8_t precision; };
typedef struct { uint8_t flags; fptrCmd set; int8_t precision; } cfgItem_t;
static struct { int cycle_state; } cm;
static bool config_loaded;                          /* config.c: set at the end of config_init() */
static bool queued, running;                        /* planner: a runnable block / the runtime busy (a lone dwell) */
static bool mp_has_runnable_buffer(void) { return queued; }
static bool mp_runtime_is_idle(void) { return !running; }
static struct { int comm_mode; } cs = { JSON_MODE };
static int ram[8], writes;
static stat_t set_val(nvObj_t *nv) { ram[nv->index] = nv->value_int; writes++; return STAT_OK; }
static stat_t set_grp(nvObj_t *nv);
static stat_t set_nul(nvObj_t *nv) { (void)nv; return STAT_OK; }
static stat_t rt_set_grp(nvObj_t *nv) { (void)nv; return STAT_OK; }
static stat_t rt_set_hb(nvObj_t *nv) { (void)nv; return STAT_OK; }
static stat_t rt_set_qt(nvObj_t *nv) { (void)nv; return STAT_OK; }
static stat_t rt_set_fi(nvObj_t *nv) { (void)nv; return STAT_OK; }
static stat_t sr_set(nvObj_t *nv) { return set_val(nv); }
static stat_t srs_set(nvObj_t *nv) { return set_val(nv); }
enum {PERSISTED = 0, RAM_ONLY = 1, SR = 2, GROUP = 3};
static const cfgItem_t cfgArray[] = {
    { F_PERSIST, set_val, 3 }, { 0, set_val, 0 }, { 0, sr_set, 0 }, { 0, set_grp, 0 },
};
#define GET_TABLE_WORD(a) cfgArray[nv->index].a
static index_t nv_index_max(void) { return 4; }
static bool nv_index_is_single(index_t i) { return i != GROUP; }
static bool cfg_has_flag(index_t i, uint8_t mask) { return (cfgArray[i].flags & mask) != 0; }
static void rt_qt_invalidate(void) {}
static nvObj_t *nv_next(nvObj_t *nv) { return nv->nx; }
static stat_t nv_get(nvObj_t *nv) { (void)nv; return STAT_OK; }
static stat_t nv_persist(nvObj_t *nv) { (void)nv; return STAT_OK; }
#define ritorno(a) do { stat_t _s = (a); if (_s != STAT_OK) return _s; } while (0)
'''

EXERCISE = r'''
static nvObj_t one(index_t i, int v) { nvObj_t n = { NULL, i, TYPE_INTEGER, v, 0 }; return n; }

int main(void) {
    nvObj_t a = one(PERSISTED, 7), b = one(RAM_ONLY, 8), s = one(SR, 9);
    queued = true;                                                  /* boot: config_init() loads settings after */
    assert(nv_set(&a) == STAT_OK && ram[PERSISTED] == 7);          /* commands were queued: it must apply */
    queued = false;
    config_loaded = true;
    assert(nv_set(&a) == STAT_OK && ram[PERSISTED] == 7);          /* idle: applies */
    cm.cycle_state = CYCLE_MACHINING;
    a.value_int = 70; writes = 0;
    assert(nv_set(&a) == STAT_SETTING_REFUSED_IN_CYCLE && ram[PERSISTED] == 7 && writes == 0);
    assert(nv_set(&s) == STAT_SETTING_REFUSED_IN_CYCLE && ram[SR] == 0);
    assert(nv_set(&b) == STAT_OK && ram[RAM_ONLY] == 8);            /* RAM-only rows and actions still run */

    /* a group: its RAM-only child comes first, the persisted child second; neither may apply */
    nvObj_t end = { NULL, 0, TYPE_EMPTY, 0, 0 };
    nvObj_t c2 = { &end, PERSISTED, TYPE_INTEGER, 71, 0 }, c1 = { &c2, RAM_ONLY, TYPE_INTEGER, 81, 0 };
    nvObj_t g = { &c1, GROUP, TYPE_PARENT, 0, 0 };
    writes = 0;
    assert(nv_set(&g) == STAT_SETTING_REFUSED_IN_CYCLE && writes == 0 && ram[RAM_ONLY] == 8 && ram[PERSISTED] == 7);
    nvObj_t q2 = { &end, PERSISTED, TYPE_NULL, 0, 0 }, q1 = { &q2, RAM_ONLY, TYPE_INTEGER, 82, 0 };
    nvObj_t gq = { &q1, GROUP, TYPE_PARENT, 0, 0 };              /* a GET of a persisted child is fine */
    assert(nv_set(&gq) == STAT_OK && ram[RAM_ONLY] == 82);

    cm.cycle_state = CYCLE_OFF;
    running = true; a.value_int = 72;                               /* a lone G4 dwell: no cycle, runtime busy */
    assert(nv_set(&a) == STAT_SETTING_REFUSED_IN_CYCLE && ram[PERSISTED] == 7);
    running = false; queued = true;                                 /* a block queued, not yet running */
    assert(nv_set(&a) == STAT_SETTING_REFUSED_IN_CYCLE && ram[PERSISTED] == 7);
    queued = false;                                                 /* idle again: all of it applies */
    assert(nv_set(&g) == STAT_OK && ram[RAM_ONLY] == 81 && ram[PERSISTED] == 71);
    assert(nv_set(&s) == STAT_OK && ram[SR] == 9);
    return 0;
}
'''


class SetRefusedInCycle(unittest.TestCase):
    def test_refused_before_it_applies(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        source = (ROOT / "firmware/tinyg/config.c").read_text().replace("\r\n", "\n")
        code = (PRELUDE + function(source, r"static bool _refused_in_cycle\(nvObj_t \*nv\)") + "\n"
                + function(source, r"stat_t nv_set\(nvObj_t \*nv\)") + "\n"
                + function(source, r"stat_t set_grp\(nvObj_t \*nv\)") + "\n" + EXERCISE)
        with tempfile.TemporaryDirectory(prefix="tinyg-n60-") as tmp:
            c, exe = Path(tmp) / "n60.c", Path(tmp) / "n60"
            c.write_text(code)
            subprocess.run([cc, "-std=gnu99", "-Wall", "-Werror", "-Wno-unused-function",
                            "-fsanitize=address,undefined", str(c), "-o", str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
