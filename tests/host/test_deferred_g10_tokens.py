"""Deferred G10 write-back persists g54x..g59c with the offsets they belong to.

The real cm_deferred_write_callback() (canonical_machine.c) runs against stub NV functions. The token is
built without sprintf so older avr-gcc (the CI toolchain) cannot warn about a format overflow; this checks
the 36 tokens and values are still exactly those G54..G59 x A..C rows.
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
#include <string.h>
#define NUL 0
#define AXES 6
#define COORDS 6
#define TOKEN_LEN 5
typedef int stat_t;
typedef int index_t;
enum {STAT_OK = 0};
enum {CYCLE_OFF = 0, CYCLE_MACHINING = 1};
typedef struct {index_t index; char token[TOKEN_LEN+1]; float value_flt;} nvObj_t;
static struct {int cycle_state; bool deferred_write_flag; float offset[COORDS+1][AXES];} cm;
static char seen[64][TOKEN_LEN+1]; static float values[64]; static int count;
static index_t nv_get_index(const char *group, const char *token) {
    (void)group; assert(strlen(token) == 4); strcpy(seen[count], token); return count;
}
static bool st_aline_in_flight(void) { return false; }
static void nv_persist(nvObj_t *nv) { assert(nv->index == count); values[count++] = nv->value_flt; }
'''

EXERCISE = r'''
int main(void) {
    for (int i = 1; i <= COORDS; i++) for (int j = 0; j < AXES; j++) cm.offset[i][j] = i * 10 + j;
    cm.cycle_state = CYCLE_MACHINING; cm.deferred_write_flag = true;
    cm_deferred_write_callback();
    assert(count == 0);                                  /* not while a cycle runs */
    cm.cycle_state = CYCLE_OFF;
    cm_deferred_write_callback();
    assert(count == 36 && !cm.deferred_write_flag);
    for (int i = 1; i <= COORDS; i++) for (int j = 0; j < AXES; j++) {
        char want[6]; snprintf(want, sizeof(want), "g%d%c", 53 + i, "xyzabc"[j]);
        int k = (i - 1) * AXES + j;
        if (strcmp(seen[k], want) || values[k] != i * 10 + j) { fprintf(stderr, "%d: %s %g\n", k, seen[k], values[k]); return 1; }
    }
    cm_deferred_write_callback();
    assert(count == 36);                                 /* the flag was consumed */
    return 0;
}
'''


class DeferredG10Tokens(unittest.TestCase):
    def test_tokens_and_values(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        source = (ROOT / "firmware/tinyg/canonical_machine.c").read_text()
        code = PRELUDE + function(source, r"static void _persist_coord_offsets\(\)") + function(source, r"stat_t cm_deferred_write_callback\(\)") + "\n" + EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-g10-") as tmp:
            c, exe = Path(tmp) / "g10.c", Path(tmp) / "g10"
            c.write_text(code)
            subprocess.run([cc, "-std=gnu99", "-Wall", "-Werror", "-fsanitize=address,undefined", str(c), "-o", str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
