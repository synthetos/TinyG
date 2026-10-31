"""Unsupported override words must fail before any part of the block executes."""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = Path(os.environ.get('TINYG_FIRMWARE', ROOT/'firmware/tinyg'))


class UnsupportedOverrides(unittest.TestCase):
    def test_parser_rejects_inert_controls(self):
        source = (FIRMWARE/'gcode_parser.c').read_text()
        names = ['_get_next_gcode_word', '_point', '_takes_subcode', '_validate_gcode_block', '_parse_gcode_block']
        definitions = []
        for name in names:
            sig = r'static (?:stat_t|uint8_t|bool) ' + name + r'\([^;{}]*\)\s*\{'
            match = re.search(sig, source)
            assert match, name
            definitions.append(function(source[match.start():], sig[:-5]))
        code = r'''
#include <stdlib.h>
#include <assert.h>
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "controller.h"
#define NUL 0
cmSingleton_t cm;
controller_t cs;
stat_t status_code;
struct { uint8_t modals[MODAL_GROUP_COUNT]; } gp;
static int executions, alarms;
uint8_t cm_get_motion_mode(const GCodeState_t *state) { return state->motion_mode; }
stat_t cm_alarm(const stat_t status, const char *message) { alarms++; return status; }
static stat_t _execute_gcode_block(void) { executions++; return STAT_OK; }
'''
        code += '\n'.join(line for line in source.splitlines() if line.startswith('#define SET_')) + '\n'
        code += '\n'.join(definitions)
        code += r'''
int main(void) {
    const char *commands[]={"M48","M49","M50","M50P0","M50P1","M50.1P0.5",
        "M50.2P0.5","M50.3P0.5","M51","M51P0","M51P1","M51.1P0.5","F600M50S1000"};
    for(unsigned i=0;i<sizeof(commands)/sizeof(commands[0]);i++) {
        memset(&cm,0,sizeof(cm)); executions=0; alarms=0;
        char line[40]; strcpy(line,commands[i]);
        assert(_parse_gcode_block(line)==STAT_MCODE_COMMAND_UNSUPPORTED);
        assert(executions==0 && alarms==0);
    }
    char supported[]="G1X1F120";
    assert(_parse_gcode_block(supported)==STAT_OK && executions==1);
    assert(cm.gf.target[AXIS_X] && cm.gn.target[AXIS_X]==1);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp); (tmp/'avr').mkdir()
            (tmp/'avr/pgmspace.h').write_text('#define PROGMEM\n#define PSTR(s) (s)\n#define strcmp_P strcmp\n')
            c=tmp/'overrides.c'; c.write_text(code); exe=tmp/'overrides'
            subprocess.run(['cc','-std=gnu99','-O1','-g','-fsanitize=address,undefined',
                            '-D_POSIX_C_SOURCE=200809L','-Wno-macro-redefined','-I'+str(tmp),
                            '-I'+str(FIRMWARE),str(c),'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)


if __name__ == '__main__':
    unittest.main()
