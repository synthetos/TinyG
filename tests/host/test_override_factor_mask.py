"""AVR code generation must finish floating-point conversion before CLI."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

FW = Path(__file__).resolve().parents[2] / 'firmware/tinyg'


class FactorMask(unittest.TestCase):
    def test_float_conversion_precedes_interrupt_mask(self):
        if not all(shutil.which(x) for x in ('avr-gcc', 'avr-objdump', 'avr-nm')):
            self.skipTest('AVR toolchain required')
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            main = out / 'main.c'
            main.write_text('#include "override.h"\nvolatile float factor=.5f;\n'
                            'int main(void){ov_init();return ov_set_factor(OV_FEED,factor,true,true);}\n')
            elf = out / 'factor.elf'
            subprocess.run(['avr-gcc', '-mmcu=atxmega192a3', '-O2', '-std=gnu99',
                            '-funsigned-char', '-funsigned-bitfields', '-fpack-struct', '-fshort-enums',
                            '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections', '-Wl,--relax',
                            '-I'+str(FW), str(main), str(FW/'override.c'), '-lm', '-o', str(elf)], check=True)
            symbols = subprocess.check_output(['avr-nm', '-S', str(elf)], text=True)
            sym = re.search(r'^([0-9a-f]+) ([0-9a-f]+) T ov_set_factor$', symbols, re.M)
            self.assertIsNotNone(sym)
            lo, size = (int(x, 16) for x in sym.groups())
            asm = subprocess.check_output(['avr-objdump', '-d', str(elf)], text=True)
            rows = []
            for line in asm.splitlines():
                match = re.match(r'\s*([0-9a-f]+):', line)
                if match and lo <= int(match[1], 16) < lo+size:
                    rows.append(line)
            cli = [i for i, line in enumerate(rows) if re.search(r'\bcli\b', line)]
            self.assertEqual(len(cli), 1)
            fp = [i for i, line in enumerate(rows)
                  if re.search(r'<__(?:mul|add|fixun|cmp|ge).*sf', line)]
            self.assertTrue(fp)
            self.assertTrue(all(i < cli[0] for i in fp), '\n'.join(rows))
