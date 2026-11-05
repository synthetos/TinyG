"""Retired restore tokens must not alter live settings or EEPROM indexes."""
from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = Path(os.environ.get("TINYG_SOURCE", Path(__file__).resolve().parents[2]))


class MasterCompatibility(unittest.TestCase):
    def test_compatibility_values(self):
        source = (ROOT / "firmware/tinyg/config_app.c").read_text()
        code = r"""
#include <assert.h>
#include <math.h>
#include <stdint.h>
#define STAT_OK 0
#define STAT_INPUT_VALUE_RANGE_ERROR 1
#define TYPE_INTEGER 2
#define TYPE_FLOAT 3
typedef int stat_t;
typedef struct {int index, valuetype, precision; union {float value_flt; uint32_t value_int;};} nvObj_t;
int cfg_is_type(int index) {return index == 2 ? TYPE_FLOAT : TYPE_INTEGER;}
"""
        for name in ("get_legacy_zero", "set_legacy_zero", "set_legacy_arc_length"):
            code += function(source, r'static stat_t ' + name + r'\(nvObj_t \*nv\)(?=\s*\{)')
        code += r"""
int main(void) {
    nvObj_t nv={0};
    for(int i=0;i<3;i++) {
        nv.index=i;nv.value_int=123;nv.precision=99;
        assert(get_legacy_zero(&nv)==STAT_OK);
        assert(nv.valuetype==cfg_is_type(i));
        if(i==2) {assert(nv.value_flt==0);assert(nv.precision==4);}
        else {assert(nv.value_int==0);}
    }
    nv.value_int=0;assert(set_legacy_zero(&nv)==STAT_OK);
    nv.value_int=1;assert(set_legacy_zero(&nv)!=STAT_OK);
    nv.value_int=UINT32_MAX;assert(set_legacy_zero(&nv)!=STAT_OK);
    float good[]={0,0.1f,1,100};
    for(unsigned i=0;i<4;i++) {
        nv.value_flt=good[i];assert(set_legacy_arc_length(&nv)==STAT_OK);
        assert(nv.value_flt==good[i]);
    }
    float bad[]={-1,INFINITY,NAN};
    for(unsigned i=0;i<3;i++) {
        nv.value_flt=bad[i];assert(set_legacy_arc_length(&nv)!=STAT_OK);
    }
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            src=Path(tmp)/"compat.c"; exe=Path(tmp)/"compat"
            src.write_text(code)
            subprocess.run(["cc", "-std=c99", "-Wall", "-Werror", "-fsanitize=address,undefined","-fstrict-flex-arrays=3", str(src), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_rows_append_after_persisted_slots(self):
        source = (ROOT / "firmware/tinyg/config_app.c").read_text()
        end = source.index('"se29"')
        groups = source.index('// Group lookups')
        for token in ("net", "fd", "ma"):
            row = re.search(r'\{[^\n]*"' + token + r'"[^\n]*\}', source)
            self.assertIsNotNone(row, token)
            self.assertGreater(row.start(), end)
            self.assertLess(row.start(), groups)
            self.assertRegex(row.group(), r',\s*_[if]0,')
            self.assertIn('get_legacy_zero', row.group())
