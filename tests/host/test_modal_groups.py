"""E152: the parser enforces RS274NGC modal-group rules (NIST 3.3.5, 3.4) before executing a block.

The real _parse_gcode_block() and _validate_gcode_block() run on the firmware headers with a stub
executor. Two words of one modal group, a group-1 motion code with an axis-word group-0 code, axis
words while G80 is in effect, and G53 outside G0/G1 are rejected with nothing executed; M7 with M8 and
ordinary multi-group lines still run.
"""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = Path(os.environ.get('TINYG_FIRMWARE', ROOT/'firmware/tinyg'))


class ModalGroups(unittest.TestCase):
    def test_modal_group_rules(self):
        source = (FIRMWARE/'gcode_parser.c').read_text()
        definitions = []
        for name in ['_get_next_gcode_word', '_point', '_takes_subcode', '_validate_gcode_block', '_parse_gcode_block']:
            sig = r'static (?:stat_t|uint8_t|bool) ' + name + r'\([^;{}]*\)\s*\{'
            match = re.search(sig, source)
            if match is None and name == '_takes_subcode':
                continue                                    # older parser without E150
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
static int executions;
uint8_t cm_get_motion_mode(const GCodeState_t *state) { return state->motion_mode; }
stat_t cm_alarm(const stat_t status, const char *message) { return status; }
static stat_t _execute_gcode_block(void) { executions++; return STAT_OK; }
'''
        code += '\n'.join(line for line in source.splitlines() if line.startswith('#define SET_')) + '\n'
        code += '\n'.join(definitions)
        code += r'''
static stat_t run(const char *text, uint8_t motion_mode) {
    memset(&cm, 0, sizeof(cm)); executions = 0;
    cm.gm.motion_mode = motion_mode;
    char line[60]; strcpy(line, text);
    return _parse_gcode_block(line);
}
static void expect(const char *text, uint8_t motion_mode, stat_t want) {
    stat_t got = run(text, motion_mode);
    if (got != want) { fprintf(stderr, "%s: got %d, wanted %d\n", text, got, want); assert(0); }
    assert(executions == (want == STAT_OK));
}
int main(void) {
    const uint8_t T = MOTION_MODE_STRAIGHT_TRAVERSE, C = MOTION_MODE_CANCEL, A = MOTION_MODE_CW_ARC;
    const stat_t V = STAT_GCODE_MODAL_GROUP_VIOLATION;
    /* two words of one group */
    expect("G0G1X1", T, V);  expect("G90G91", T, V);  expect("G20G21", T, V);  expect("G17G18", T, V);
    expect("G54G55", T, V);  expect("G61G64", T, V);  expect("G93G94", T, V);
    expect("G4P1G92X0", T, V);  expect("G28.2X0G28.3X0", T, V);
    expect("M3M5", T, V);  expect("M0M2", T, V);  expect("M7M9", T, V);  expect("M8M9", T, V);  expect("M7M7", T, V);
    /* group-1 motion with an axis-word group-0 code */
    expect("G1X10G28.2Z0", T, V);  expect("G0X1G92X0", T, V);  expect("G1X1G10L2P1X0", T, V);
    expect("G1X1G28X0", T, V);  expect("G0X1G38.2Z-1F100", T, V);
    /* axis words while G80 is in effect, with no group-0 axis code */
    expect("X10", C, STAT_GCODE_AXIS_CANNOT_BE_PRESENT);  expect("G80X10", T, STAT_GCODE_AXIS_CANNOT_BE_PRESENT);
    /* G53 needs G0 or G1 */
    expect("G53X0", C, STAT_GCODE_G53_WITHOUT_G0_OR_G1);  expect("G53X0", A, STAT_GCODE_G53_WITHOUT_G0_OR_G1);
    /* still accepted */
    expect("M7M8", T, STAT_OK);  expect("G0X1", C, STAT_OK);  expect("G90G21G17G54G61G94G0X1", T, STAT_OK);
    expect("G92X0", C, STAT_OK);  expect("G28.2X0", C, STAT_OK);  expect("G28.3X0", C, STAT_OK);
    expect("G53G0X0", C, STAT_OK);  expect("G53X0", T, STAT_OK);  expect("X5", T, STAT_OK);  expect("G80", T, STAT_OK);
    expect("G1X1F100M3S1000", T, STAT_OK);  expect("G4P0.1", C, STAT_OK);  expect("G10L2P1X0", C, STAT_OK);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp); (tmp/'avr').mkdir()
            (tmp/'avr/pgmspace.h').write_text('#define PROGMEM\n#define PSTR(s) (s)\n#define strcmp_P strcmp\n')
            c = tmp/'modal.c'; c.write_text(code); exe = tmp/'modal'
            subprocess.run(['cc', '-std=gnu99', '-O1', '-g', '-fsanitize=address,undefined',
                            '-D_POSIX_C_SOURCE=200809L', '-Wno-macro-redefined', '-I'+str(tmp),
                            '-I'+str(FIRMWARE), str(c), '-lm', '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
