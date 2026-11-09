"""AVR layout contract: reclaim two unused floats without removing the parser P word."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class PlannerModelLayout(unittest.TestCase):
    @unittest.skipUnless(shutil.which('avr-gcc'), 'AVR compiler required for firmware layout')
    def test_compact_model_retains_the_parser_parameter_and_motion_vectors(self):
        source = r'''
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "planner.h"
_Static_assert(sizeof(GCodeState_t) == 68, "remove the two unused model floats");
_Static_assert(sizeof(((mpBuf_t *)0)->gm) == 68, "each queued model must shrink");
_Static_assert(sizeof(cm.gn.parameter) == 4, "parser P word must remain");
_Static_assert(sizeof(cm.gf.parameter) == 1, "parser P presence flag must remain");
_Static_assert(sizeof(((GCodeState_t *)0)->target) == 24, "six full precision target axes");
_Static_assert(sizeof(((GCodeState_t *)0)->work_offset) == 24, "historical coordinate frame retained");
_Static_assert(sizeof(((GCodeState_t *)0)->move_time) == 4, "move timing retained in this increment");
'''
        with tempfile.TemporaryDirectory() as d:
            src, obj = Path(d)/'layout.c', Path(d)/'layout.o'
            src.write_text(source)
            subprocess.run(['avr-gcc','-mmcu=atxmega192a3','-DF_CPU=32000000UL','-std=gnu99',
                            '-funsigned-char','-funsigned-bitfields','-fpack-struct','-fshort-enums',
                            '-I'+str(ROOT/'firmware/tinyg'),'-c',str(src),'-o',str(obj)],check=True)


if __name__ == '__main__': unittest.main()
