"""Program permission cannot be bypassed by a live operator request."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function

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
    ovTicket_t ticket={0};ticket.boundary.lead_ticks=32;ov_ticket_stamp(&ticket,100);
    assert(ov_ticket_current(&ticket,101));gen=s.request;
    ov_reset_job();ov_snapshot(&s);
    assert(s.feed==1000 && s.committed_feed==1000 && s.program_feed);
    assert((s.enables&OV_FEED) && !s.armed && !s.capped);
    assert(s.spindle==1000 && s.committed_spindle==1000);
    assert(s.request!=gen && !ov_ticket_current(&ticket,101));
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c,exe=Path(tmp)/'test.c',Path(tmp)/'test'
            c.write_text(code)
            subprocess.run(['cc','-std=gnu99','-Wall','-Werror','-fsanitize=address,undefined',
                            '-I'+str(ROOT/'firmware/tinyg'),str(c),
                            str(ROOT/'firmware/tinyg/override.c'),'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

    def test_program_permission_executes_in_queue_order(self):
        source=(ROOT/'firmware/tinyg/canonical_machine.c').read_text()
        code=r'''
#include <assert.h>
#include "override.h"
typedef unsigned char stat_t;
#define AXES 6
#define STAT_OK 0
#define FLAGS_ONE ((bool[6]){true})
static struct {struct {bool parameter;} gf;struct {float parameter;} gn;} cm;
static void (*pending[4])(float *,bool *);
static float values[4][6];static unsigned queued;
static void mp_queue_command(void (*f)(float *,bool *),float *v,bool *flags) {
    pending[queued]=f;for(unsigned i=0;i<6;i++)values[queued][i]=v[i];queued++;
}
'''
        for name in ['_exec_feed_permission','cm_override_enables','cm_feed_rate_override_enable']:
            code+=function(source, (r"static void " if name.startswith("_") else r"stat_t ")+name+r"\(")+ '\n'
        code+=r'''
int main(void) {
    ovSnapshot_t s;ov_init();ov_set_factor(OV_FEED,.5f,true,true);
    cm_override_enables(false);ov_snapshot(&s);assert(s.program_feed);
    cm.gf.parameter=true;cm.gn.parameter=1;cm_feed_rate_override_enable(true);
    ov_snapshot(&s);assert(s.program_feed && s.feed==500);
    pending[0](values[0],FLAGS_ONE);ov_snapshot(&s);assert(!s.program_feed);
    ov_set_factor(OV_FEED,1.5f,true,true);ov_set_enable(OV_FEED,true,true,true);
    ov_snapshot(&s);assert(!s.program_feed);
    pending[1](values[1],FLAGS_ONE);ov_snapshot(&s);assert(s.program_feed && s.feed==1500);
    cm.gn.parameter=0;cm_feed_rate_override_enable(true);
    pending[2](values[2],FLAGS_ONE);ov_snapshot(&s);assert(!s.program_feed);
    cm.gf.parameter=false;cm_feed_rate_override_enable(true);
    pending[3](values[3],FLAGS_ONE);ov_snapshot(&s);assert(s.program_feed);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c,exe=Path(tmp)/'test.c',Path(tmp)/'test';c.write_text(code)
            subprocess.run(['cc','-std=gnu99','-Wall','-Werror','-fsanitize=address,undefined',
                            '-I'+str(ROOT/'firmware/tinyg'),str(c),
                            str(ROOT/'firmware/tinyg/override.c'),'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
