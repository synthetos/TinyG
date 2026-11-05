"""Exact unsigned conversion on AVR must not rely on 32-bit float rounding."""
from pathlib import Path
from decimal import Decimal
import random
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT=Path(__file__).resolve().parents[2]

class UnsignedJsonNumbers(unittest.TestCase):
    def test_decimal_integer_boundaries(self):
        source=(ROOT/'firmware/tinyg/util.c').read_text()
        code=r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define STAT_OK 0
#define STAT_VALUE_TYPE_ERROR 116
#define STAT_BAD_NUMBER_FORMAT 108
#define STAT_INPUT_VALUE_RANGE_ERROR 109
typedef int stat_t;
'''+function(source,r'stat_t str2long\(')+r'''
int main(int argc,char **argv){
 for(int i=1;i<argc;i++) {uint32_t value=1234567;int s=str2long(argv[i],&value);printf("%d %lu\n",s,(unsigned long)value);}
}
'''
        good={'0':0,'8.0':8,'010':10,'08':8,'09':9,'1e3':1000,'1.50e1':15,
              '4294967295':4294967295,'4294967295.0':4294967295,
              '429496729500e-2':4294967295,'0e999999':0,'0.000e-99999':0,
              '1000000000000000000000000000000e-30':1}
        bad=['-1','-0','4294967296','4294967295.1','4294967294.9','1e1000','1e-1000','8.5',
             '0x10','1e','1e+','1.2.3','','.','abc','1x',' 1','1 ']
        rng=random.Random(107)
        for _ in range(300):
            n=rng.randrange(2**32);zeros=rng.randrange(1,12)
            text=str(n)+'0'*zeros+'e-'+str(zeros)
            good[text]=int(Decimal(text))
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'number.c';exe=Path(tmp)/'number';p.write_text(code)
            subprocess.run(['cc','-std=c99','-Wall','-Werror','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(p),'-o',str(exe)],check=True)
            lines=subprocess.check_output([str(exe),*good,*bad],text=True).splitlines()
        for text,line in zip([*good,*bad],lines):
            status,value=map(int,line.split())
            if text in good:self.assertEqual((status,value),(0,good[text]),text)
            else:
                self.assertNotEqual(status,0,text)
                self.assertEqual(value,1234567,text) # failed conversion must not publish a partial result
