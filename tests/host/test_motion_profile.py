"""The separate timing image must not hide a timer overflow as a short sample."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class MotionProfile(unittest.TestCase):
    def test_counter_maximum_and_overflow_rejection(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            (folder/'avr').mkdir()
            (folder/'avr/io.h').write_text('''
#include <stdint.h>
#define TC_CLKSEL_OFF_gc 0
#define TC_CLKSEL_DIV8_gc 4
#define TC1_OVFIF_bm 1
static struct {uint8_t CTRLA,CTRLB,INTCTRLA,INTCTRLB,INTFLAGS;uint16_t CNT,PER;} TCC1;
''')
            code = r'''
#include <assert.h>
#include "motion_profile.h"
volatile motionProfile_t motion_profile;
int main(void) {
    motion_profile_begin();
    assert(TCC1.PER==65535 && TCC1.CNT==0 && TCC1.CTRLA==TC_CLKSEL_DIV8_gc);
    assert(TCC1.INTFLAGS==TC1_OVFIF_bm && !TCC1.INTCTRLA && !TCC1.INTCTRLB);
    /* Model hardware's write-one-to-clear effect, then advance time. */
    TCC1.INTFLAGS=0;TCC1.CNT=12;motion_profile_end(2);
    assert(motion_profile.calls==1 && motion_profile.maximum_ticks==12);
    assert(motion_profile.site==3 && !TCC1.CTRLA && !motion_profile.overflows);
    motion_profile_begin();TCC1.INTFLAGS=0;TCC1.CNT=7;motion_profile_end(2);
    assert(motion_profile.maximum_ticks==12);
    motion_profile_begin();TCC1.INTFLAGS=TC1_OVFIF_bm;TCC1.CNT=2;motion_profile_end(2);
    assert(motion_profile.maximum_ticks==65535 && motion_profile.overflows==1);
    assert(motion_profile.calls==3 && motion_profile.section_at_maximum==2);
}
'''
            source, exe = folder/'check.c', folder/'check'
            source.write_text(code)
            subprocess.run(['cc','-std=gnu99','-Wall','-Werror','-D__AVR__',
                            '-DTINYG_MOTION_PROFILE=3','-I'+str(folder),
                            '-I'+str(ROOT/'firmware/tinyg'),str(source),'-o',str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
