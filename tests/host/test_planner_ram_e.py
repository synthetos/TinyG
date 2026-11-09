"""The planner's static publication workspace must hold only the 13 trapezoid floats."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

FW = Path(__file__).resolve().parents[2] / 'firmware/tinyg'


class TrapezoidWorkspace(unittest.TestCase):
    @unittest.skipUnless(shutil.which('avr-gcc') and shutil.which('avr-nm'), 'AVR tools required')
    def test_workspace_is_52_bytes_on_avr(self):
        with tempfile.TemporaryDirectory() as d:
            obj = Path(d)/'plan.o'
            subprocess.run(['avr-gcc','-mmcu=atxmega192a3','-DF_CPU=32000000UL','-O2','-std=gnu99',
                            '-funsigned-char','-funsigned-bitfields','-fpack-struct','-fshort-enums',
                            '-I'+str(FW),'-c',str(FW/'plan_line.c'),'-o',str(obj)],check=True)
            symbols = subprocess.check_output(['avr-nm','-S',str(obj)],text=True).splitlines()
            sizes = [int(row.split()[1],16) for row in symbols if row.split()[-1].startswith('plan.')]
            self.assertEqual(sizes, [52], 'keep the workspace static and compact; never move it to the stack')


if __name__ == '__main__': unittest.main()
