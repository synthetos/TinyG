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
#include <stdio.h>
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
    /* an E after a value is a word, not an exponent (E159) */
    const char *exponents[]={"G1X10E5","G1X1E-3F100","G0X0.001E3","G1X1F1E2","M3E0"};
    for(unsigned i=0;i<sizeof(exponents)/sizeof(exponents[0]);i++) {
        memset(&cm,0,sizeof(cm)); executions=0; char line[40]; strcpy(line,exponents[i]);
        assert(_parse_gcode_block(line)==STAT_GCODE_COMMAND_UNSUPPORTED && executions==0);
    }
    /* dotted codes (E150): only a single-digit sub-code of a base that has sub-codes runs */
    const char *g_rejected[]={"G59.1","G59.3","G54.1","G17.1","G21.5","G64.1","G4.1P1","G0.5X1",
        "G28.01","G92.19","G61.15","G90.11","G28.0005","G92.1005","G59.0001","G591E-1","G59.1E0","G92E0"};
    for(unsigned i=0;i<sizeof(g_rejected)/sizeof(g_rejected[0]);i++) {
        memset(&cm,0,sizeof(cm)); executions=0; char line[40]; strcpy(line,g_rejected[i]);
        if(_parse_gcode_block(line)!=STAT_GCODE_COMMAND_UNSUPPORTED) { fprintf(stderr,"%s\n",g_rejected[i]); assert(0); }
        assert(executions==0);
    }
    const char *m_rejected[]={"M3.1","M5.5","M30.1","M201.39","M201.31","M201.3005","M301E-1"};
    for(unsigned i=0;i<sizeof(m_rejected)/sizeof(m_rejected[0]);i++) {
        memset(&cm,0,sizeof(cm)); executions=0; char line[40]; strcpy(line,m_rejected[i]);
        if(_parse_gcode_block(line)!=STAT_MCODE_COMMAND_UNSUPPORTED) { fprintf(stderr,"%s\n",m_rejected[i]); assert(0); }
        assert(executions==0);
    }
    const char *accepted[]={"G92.1","G92.10","G92.100","G28.1","G28.10","G61.1","G61.10","G90.1","G91.1","G59","G59.0","G28.3X0"};
    for(unsigned i=0;i<sizeof(accepted)/sizeof(accepted[0]);i++) {
        memset(&cm,0,sizeof(cm)); executions=0; char line[40]; strcpy(line,accepted[i]);
        if(_parse_gcode_block(line)!=STAT_OK) { fprintf(stderr,"%s\n",accepted[i]); assert(0); }
        assert(executions==1);
    }
    memset(&cm,0,sizeof(cm)); executions=0;
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
