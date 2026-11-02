"""Spindle and coolant state readouts (spindle.c), named as in g2core: spe, spd, sps, cof, com.

The real getters run against every spindle mode and coolant state: spe is 1 for CW and CCW, spd is 1 only
for CCW (g2core: 0 = CW, 1 = CCW), sps is the model's speed, cof/com follow flood and mist. The rows are
read-only (set_nul) and never initialized or persisted.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FW = Path(os.environ.get("TINYG_FIRMWARE", ROOT / "firmware/tinyg"))

CODE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef uint8_t stat_t;
#define STAT_OK 0
enum { TYPE_INTEGER = 2, TYPE_FLOAT = 4 };
enum { SPINDLE_OFF = 0, SPINDLE_CW, SPINDLE_CCW };
typedef struct { uint8_t valuetype; union { float value_flt; uint32_t value_int; }; } nvObj_t;
struct { struct { uint8_t spindle_mode, mist_coolant, flood_coolant; float spindle_speed; } gm; } cm;
'''


class SpindleCoolantStatus(unittest.TestCase):
    def test_readouts(self):
        src = (FW / "spindle.c").read_text(encoding="latin-1")
        code = CODE + function(src, r"static stat_t _get_state\(nvObj_t \*nv, uint8_t value\)") + "\n"
        for line in re.findall(r"^stat_t (?:spindle|coolant)_get_\w+\(nvObj_t \*nv\) \{[^\n]*\}$", src, re.M):
            code += line + "\n"
        code += function(src, r"stat_t spindle_get_sps\(nvObj_t \*nv\)") + r'''
static uint32_t i(stat_t (*f)(nvObj_t *)) { nvObj_t nv = {0}; assert(f(&nv) == STAT_OK && nv.valuetype == TYPE_INTEGER); return nv.value_int; }
int main(void) {
    cm.gm.spindle_mode = SPINDLE_OFF; assert(i(spindle_get_spe) == 0 && i(spindle_get_spd) == 0);
    cm.gm.spindle_mode = SPINDLE_CW;  assert(i(spindle_get_spe) == 1 && i(spindle_get_spd) == 0);
    cm.gm.spindle_mode = SPINDLE_CCW; assert(i(spindle_get_spe) == 1 && i(spindle_get_spd) == 1);
    cm.gm.spindle_speed = 12345.5f; { nvObj_t nv = {0}; assert(spindle_get_sps(&nv) == STAT_OK && nv.valuetype == TYPE_FLOAT && nv.value_flt == 12345.5f); }
    for (int f = 0; f < 2; f++) for (int m = 0; m < 2; m++) {
        cm.gm.flood_coolant = f; cm.gm.mist_coolant = m;
        assert(i(coolant_get_cof) == (uint32_t)f && i(coolant_get_com) == (uint32_t)m);
    }
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c, exe = Path(tmp) / "sp.c", Path(tmp) / "sp"
            c.write_text(code)
            subprocess.run(["cc", "-std=gnu99", "-O1", "-Wall", "-Werror", "-fsanitize=address,undefined", str(c), "-o", str(exe)], check=True)
            r = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_rows_read_only_and_not_persisted(self):
        table = (FW / "config_app.c").read_text(encoding="latin-1")
        for key, getter in (("spe", "spindle_get_spe"), ("spd", "spindle_get_spd"), ("sps", "spindle_get_sps"),
                            ("cof", "coolant_get_cof"), ("com", "coolant_get_com")):
            row = re.search(r'\{\s*""\s*,\s*"' + key + r'"\s*,\s*(_\w+)\s*,[^,]*,[^,]*,\s*(\w+)\s*,\s*(\w+)\s*,', table)
            self.assertIsNotNone(row, key)
            self.assertIn(row.group(1), ("_i0", "_f0"), key)            # no F_INITIALIZE or F_PERSIST
            self.assertEqual((row.group(2), row.group(3)), (getter, "set_nul"), key)


if __name__ == "__main__":
    unittest.main()
