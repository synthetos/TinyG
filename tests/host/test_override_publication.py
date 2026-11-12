"""The committed factor changes only when exec accepts a private plan."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function

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

    def test_events_coalesce_and_explicit_get_has_no_footer(self):
        source=(ROOT/'firmware/tinyg/runtime.c').read_text()
        code=r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "override.h"
typedef uint8_t stat_t;
#define STAT_OK 0
#define STAT_UNRECOGNIZED_NAME 100
#define STAT_INPUT_VALUE_RANGE_ERROR 110
#define TYPE_INTEGER 2
#define TYPE_BOOLEAN 3
#define JSON_MODE 1
#define RT_TICKS_PER_SECOND 1024UL
#define PSTR(x) x
#define snprintf_P snprintf
typedef struct {uint32_t value_int;uint8_t valuetype;} nvObj_t;
static struct {bool allowed,check_only,ov_events,ov_ask;uint32_t ov_revision,ov_sent_at;} rt;
static struct {uint32_t status_report_interval;} sr;
static struct {uint8_t comm_mode;} cs;
static uint32_t now;
static unsigned sends;
static bool tx=true;
static char sent[248];
static uint32_t SysTickTimer_getValue(void){return now;}
static bool xio_usb_try_send_frame(const char *p){
    assert(strlen(p)<112);if(!tx)return false;
    strcpy(sent,p);sends++;return true;
}
"""
        for name in ('ov_get_capped','rt_get_ov','rt_set_ov','rt_override_callback'):
            code+=function(source,r'stat_t '+name+r'\(')+'\n'
        code+=r"""
int main(void){
    ov_init();cs.comm_mode=JSON_MODE;sr.status_report_interval=100;rt.allowed=true;
    nvObj_t n={.valuetype=TYPE_INTEGER,.value_int=1};
    now=0xfffffff0UL;rt_override_callback();assert(!sends);
    rt.check_only=true;assert(!rt_set_ov(&n));assert(!rt.ov_events);
    rt.check_only=false;assert(!rt_set_ov(&n));rt_override_callback();assert(sends==1);
    assert(strstr(sent,"\"v\":1") && !strstr(sent,"\"f\":") && !strstr(sent,"\"r\":"));
    ov_set_factor(OV_FEED,.5f,true,true);ov_set_factor(OV_FEED,1.5f,true,true);
    now=20;rt_override_callback();assert(sends==1); /* deadline across tick wrap */
    now=100;tx=false;rt_override_callback();assert(sends==1);
    ov_commit_feed(1500,true);tx=true;rt_override_callback();assert(sends==2);
    assert(strstr(sent,"\"mfo\":1.500") && strstr(sent,"\"mfc\":1.500") && strstr(sent,"\"capped\":true"));
    ov_get_capped(&n);assert(n.valuetype==TYPE_BOOLEAN && n.value_int==1);
    now=500;rt_override_callback();assert(sends==2); /* no repeated unchanged event */
    rt_get_ov(&n);rt_override_callback();assert(sends==3); /* explicit connect/mid-job read */
    n.valuetype=TYPE_INTEGER;n.value_int=0;rt_set_ov(&n);
    ov_commit_feed(500,false);now=1000;rt_override_callback();assert(sends==3);
    rt_get_ov(&n);rt_override_callback();assert(sends==4);
    assert(strstr(sent,"\"mfc\":0.500") && strstr(sent,"\"capped\":false"));
    sr.status_report_interval=2147483647UL;
    n.valuetype=TYPE_INTEGER;n.value_int=1;rt_set_ov(&n);rt_override_callback();assert(sends==5);
    ov_commit_feed(1000,false);now+=2147483647UL;rt_override_callback();assert(sends==5);
    now+=51539607UL;rt_override_callback();assert(sends==6);
    rt.allowed=false;assert(rt_get_ov(&n)==100);assert(rt_set_ov(&n)==100);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            c,exe=Path(tmp)/'test.c',Path(tmp)/'test'
            c.write_text(code)
            subprocess.run(['cc','-std=gnu99','-Wall','-Werror','-fsanitize=address,undefined',
                            '-I'+str(ROOT/'firmware/tinyg'),str(c),
                            str(ROOT/'firmware/tinyg/override.c'),'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
