"""Bounded watermark scanning; hardware startup paint is verified in the AVR disassembly."""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]
class StackWatermark(unittest.TestCase):
 def test_scan_bounds(self):
  s=(ROOT/'firmware/tinyg/stack_report.c').read_text()
  f=function(s,r'static uint16_t _stack_unused\([^;{}]*\)(?=\s*\{)')
  code='#include <stdint.h>\n#include <assert.h>\n#define STACK_PAINT 0xa5\n'+f+r'''
int main(void) {
 uint8_t ram[64];for(int i=0;i<64;i++)ram[i]=STACK_PAINT;
 assert(_stack_unused(ram,ram)==0);assert(_stack_unused(ram,ram+64)==64);
 for(int i=0;i<64;i++) {ram[i]=0;assert(_stack_unused(ram,ram+64)==i);ram[i]=STACK_PAINT;}
 assert(_stack_unused(ram+20,ram+19)==0);
 return 0;
}
'''
  with tempfile.TemporaryDirectory() as d:
   c=Path(d)/'check.c';e=Path(d)/'check';c.write_text(code)
   subprocess.run(['cc','-fsanitize=address,undefined',str(c),'-o',str(e)],check=True)
   subprocess.run([str(e)],check=True)
