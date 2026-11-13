"""Compare the production event formatter with its previous wire representation."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

FW = Path(__file__).resolve().parents[2] / 'firmware/tinyg'


class OverrideFrameFormat(unittest.TestCase):
    def test_integer_thousandths_preserve_wire_bytes(self):
        source = (FW / 'runtime.c').read_text()
        callback = source.split('stat_t rt_override_callback(void)', 1)[1]
        formatter = re.search(r'snprintf_P\(frame,.*?;', callback, re.S)[0]
        self.assertIn('char frame[112];', callback)
        self.assertNotIn('%.3f', formatter)
        code = r'''
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#define snprintf_P snprintf
#define PSTR(x) x
#define OV_FEED 1
int main(void) {
    struct { uint32_t revision; uint16_t feed, committed_feed;
             unsigned enables, program_feed, capped; } s;
    char frame[112], old[112];
    for (unsigned f=50; f<=2000; ++f) {
        for (unsigned flags=0; flags<8; ++flags) {
            s.revision=UINT32_MAX; s.feed=f; s.committed_feed=2050-f;
            s.enables=flags&1; s.program_feed=(flags>>1)&1; s.capped=flags&4;
''' + formatter + r'''
            snprintf(old,sizeof(old),"{\"ov\":{\"v\":1,\"q\":%lu,\"mfo\":%.3f,\"mfc\":%.3f,\"mfe\":%u,\"capped\":%s}}\n",
                (unsigned long)s.revision,(double)(s.feed/1000.0f),(double)(s.committed_feed/1000.0f),
                (unsigned)((s.enables&OV_FEED) && s.program_feed),s.capped?"true":"false");
            assert(strcmp(frame,old)==0);
            assert(strlen(frame)<sizeof(frame)-1);
        }
    }
    return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix='override-frame-') as tmp:
            c = Path(tmp) / 'frame.c'
            exe = Path(tmp) / 'frame'
            c.write_text(code)
            subprocess.run(['cc', '-Wall', '-Werror', str(c), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
