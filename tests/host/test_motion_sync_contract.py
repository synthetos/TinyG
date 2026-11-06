"""M400's tokenized-envelope and standalone-block eligibility contract."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]

class MotionSyncContract(unittest.TestCase):
    def test_contract(self):
        json=(ROOT/'firmware/tinyg/json_parser.c').read_text()
        gc=(ROOT/'firmware/tinyg/gcode_parser.c').read_text()
        case=re.search(r'case 400:\s*\{(.*?)SET_(?:NON_MODAL \(next_action, |NEXT_ACTION \()NEXT_ACTION_WAIT_FOR_COMPLETION\);',gc,re.S)
        self.assertIsNotNone(case)
        code=r'''
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <string.h>
#define NUL 0
#define JTOK_FLAG 0x80
#define JTOK_DEPTH 0x70
#define JTOK_DSHIFT 4
#define JTOK_KEY 8
#define JTOK_PARENT 4
#define PSTR(s) (s)
#define strcmp_P strcmp
#define STAT_COMMAND_NOT_ACCEPTED 106
struct {bool tid_present;} js;
'''
        code+=function(json,r'static bool _motion_sync_envelope\([^;{}]*\)(?=\s*\{)')
        code+='\nint validate(float value,const char *buf){'+case[1]+'return 0;}\n'
        code+=r'''
int main(void){
 assert(_motion_sync_envelope("gc\x98\"M400\"\x93"));
 js.tid_present=true;
 assert(_motion_sync_envelope("tid\x98" "0\x92gc\x98\"M400\"\x93"));
 assert(_motion_sync_envelope("gc\x98\"M400\"\x93tid\x98" "4294967295\x92"));
 assert(!_motion_sync_envelope("gc\x98\"M400\"\x93" "fb\x98null\x90"));
 assert(!_motion_sync_envelope("fb\x98null\x90gc\x98\"M400\"\x93"));
 assert(!_motion_sync_envelope("gc\x98\"M400\"\x93gc\x98\"G91\"\x93"));
 assert(!_motion_sync_envelope("txt\x98\"M400\"\x93"));
 assert(!_motion_sync_envelope("x\x9cgc\xa8\"M400\"\xa3"));
 assert(!_motion_sync_envelope("gc\x98\"M400\"\x93tid\x98" "1\x92tid\x98" "2\x92"));
 js.tid_present=false;
 assert(!_motion_sync_envelope("gc\x98\"M400\"\x93tid\x98\"bad\"\x93"));
 // Literal text inside a value does not create another key.
 assert(_motion_sync_envelope("gc\x98\"M400 (tid:123 gc:hello)\"\x93"));
 const char *good[]={"M400","N0M400","N123M400","N000123M400","N99999M400"};
 for(unsigned i=0;i<sizeof(good)/sizeof(*good);i++)assert(validate(400,good[i])==0);
 const char *bad[]={"M400G91","G91M400","M400X1","M400M400","M400.0","NM400","N-1M400","N+1M400","N1.0M400","N1N2M400","N1G91M400","N1M400X1","N1M400M400","M400N1"};
 for(unsigned i=0;i<sizeof(bad)/sizeof(*bad);i++)assert(validate(400,bad[i])==106);
 assert(validate(400.5f,"M400.5")==106);
}
'''
        # Split hexadecimal escapes before following letters that are hex digits.
        code=code.replace('\\x92gc','\\x92" "gc').replace('\\x90gc','\\x90" "gc').replace('\\x93gc','\\x93" "gc').replace('\\x9cgc','\\x9c" "gc')
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory);(path/'check.c').write_text(code)
            subprocess.run(['cc','-Wall','-Werror','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(path/'check.c'),'-o',str(path/'check')],check=True)
            subprocess.run([str(path/'check')],check=True)
