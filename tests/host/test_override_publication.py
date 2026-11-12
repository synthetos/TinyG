"""The committed factor changes only when exec accepts a private plan."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]


class OverridePublication(unittest.TestCase):
    def test_requested_committed_and_capped_are_separate(self):
        code=r'''
#include <assert.h>
#include "override.h"
int main(void) {
    ovSnapshot_t s;
    ov_init();ov_snapshot(&s);
    assert(s.committed_feed==1000 && !s.capped);
    ov_set_enable(OV_FEED,true,true,true);
    ov_set_factor(OV_FEED,1.5f,true,true);ov_snapshot(&s);
    assert(s.feed==1500 && s.committed_feed==1000 && !s.capped);
    unsigned before=s.revision;
    ov_commit_feed(1500,true);ov_snapshot(&s);
    assert(s.feed==1500 && s.committed_feed==1500 && s.capped);
    assert(s.revision!=before);before=s.revision;
    ov_commit_feed(1500,true);ov_snapshot(&s);assert(s.revision==before);
    ov_set_factor(OV_FEED,.5f,true,true);ov_snapshot(&s);
    assert(s.feed==500 && s.committed_feed==1500 && s.capped);
    ov_commit_feed(500,false);ov_snapshot(&s);
    assert(s.committed_feed==500 && !s.capped);
    ov_init();ov_snapshot(&s);assert(s.committed_feed==1000 && !s.capped);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c,exe=Path(tmp)/'test.c',Path(tmp)/'test'
            c.write_text(code)
            subprocess.run(['cc','-std=gnu99','-Wall','-Werror','-fsanitize=address,undefined',
                            '-I'+str(ROOT/'firmware/tinyg'),str(c),
                            str(ROOT/'firmware/tinyg/override.c'),'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
