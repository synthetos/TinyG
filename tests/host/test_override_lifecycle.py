"""Program permission cannot be bypassed by a live operator request."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]


class OverrideLifecycle(unittest.TestCase):
    def test_permission_request_and_job_reset_are_separate(self):
        code=r'''
#include <assert.h>
#include "override.h"
int main(void){
    ovSnapshot_t s;ov_init();ov_snapshot(&s);
    assert(s.program_feed && (s.enables&OV_FEED) && s.feed==1000 && !s.armed);
    ov_set_factor(OV_FEED,.5f,true,true);
    ov_set_program_feed(false);ov_snapshot(&s);
    assert(s.feed==500 && !s.program_feed);
    unsigned gen=s.request;
    ov_set_factor(OV_FEED,1.5f,true,true);
    ov_set_enable(OV_FEED,true,true,true);ov_snapshot(&s);
    assert(s.feed==1500 && !s.program_feed && s.request!=gen);
    ov_set_program_feed(true);ov_snapshot(&s);assert(s.program_feed && s.feed==1500);
    ov_arm(true,true,true,true);ov_commit_feed(1500,true);
    ov_reset_job();ov_snapshot(&s);
    assert(s.feed==1000 && s.committed_feed==1000 && s.program_feed);
    assert((s.enables&OV_FEED) && !s.armed && !s.capped);
    assert(s.spindle==1000 && s.committed_spindle==1000);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c,exe=Path(tmp)/'test.c',Path(tmp)/'test'
            c.write_text(code)
            subprocess.run(['cc','-std=gnu99','-Wall','-Werror','-fsanitize=address,undefined',
                            '-I'+str(ROOT/'firmware/tinyg'),str(c),
                            str(ROOT/'firmware/tinyg/override.c'),'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
