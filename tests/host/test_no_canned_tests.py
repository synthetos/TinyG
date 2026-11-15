"""Disabled canned commands must reject without touching the input source."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]


class NoCannedTests(unittest.TestCase):
    def test_disabled_programs_do_not_prepare_or_switch_input(self):
        source = (ROOT/'firmware/tinyg/test.c').read_text()
        code = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#define STAT_OK 0
#define STAT_ERROR 1
#define STAT_FUNCTION_IS_STUBBED 17
#define XIO_DEV_PGM 2
#define PSTR(x) x
#define printf_P printf
#define ritorno(x) do {uint8_t s=(x);if(s)return s;}while(0)
typedef struct {uint32_t value_int;} nvObj_t;
static unsigned prepared,switched;
static uint8_t xio_prepare_pgm_source(void){prepared++;return 0;}
static void controller_set_primary_source(unsigned s){switched++;}
'''
        code += function(source, r'uint8_t run_test\(')
        code += r'''
int main(void){
    nvObj_t n={0};assert(run_test(&n)==STAT_OK);
    unsigned ids[]={1,2,4,8,14,50,51,99};
    for(unsigned i=0;i<sizeof(ids)/sizeof(ids[0]);i++){
        n.value_int=ids[i];assert(run_test(&n)==STAT_FUNCTION_IS_STUBBED);
        assert(!prepared && !switched);
    }
}
'''
        with tempfile.TemporaryDirectory() as work:
            c, exe = Path(work)/'check.c', Path(work)/'check'
            c.write_text(code)
            subprocess.run(['cc','-std=gnu99',str(c),'-o',str(exe)],check=True)
            result = subprocess.run([str(exe)],check=True,capture_output=True)
            self.assertEqual(result.stdout, b"", "command rejection must not inject bare text into JSON")


if __name__ == '__main__':
    unittest.main()
