"""Check the alarm-clear grammar without issuing motor commands."""
import pathlib
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT=pathlib.Path(__file__).resolve().parents[2]
PRELUDE=r'''
#include <assert.h>
#include <ctype.h>
#include <string.h>
#define NUL 0
#define PSTR(s) (s)
#define strcmp_P strcmp
#define MACHINE_ALARM 2
static struct {int machine_state;} cm;
static unsigned clears;
static void cm_clear(void) {clears++;}
'''
EXERCISE=r'''
int main(void) {
    const char *yes[]={"M2","M02","M30","N10M30","N0M2","N00123M02"};
    const char *no[]={"","N","NM30","N-1M30","N1.5M30","N1e2M30","N1M300","N1M20","N1M258","N1M30X1","G1X1M30","N1M30M3","/N1M30","M30.1"};
    for(unsigned state=0;state<14;state++) {
        cm.machine_state=state;
        for(unsigned i=0;i<sizeof(yes)/sizeof(*yes);i++) {
            clears=0;_parse_clear(yes[i]);assert(clears==(state==MACHINE_ALARM));
        }
        for(unsigned i=0;i<sizeof(no)/sizeof(*no);i++) {clears=0;_parse_clear(no[i]);assert(!clears);}
    }
    return 0;
}
'''
class NumberedClear(unittest.TestCase):
    def test_only_standalone_end_commands_clear_alarm(self):
        source=(ROOT/'firmware/tinyg/gcode_parser.c').read_text()
        code=PRELUDE+function(source,r'static void _parse_clear\(const char \*str\)\s*\n')+EXERCISE
        with tempfile.TemporaryDirectory() as tmp:
            src=pathlib.Path(tmp)/'clear.c';exe=pathlib.Path(tmp)/'clear';src.write_text(code)
            subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
if __name__=='__main__': unittest.main()
