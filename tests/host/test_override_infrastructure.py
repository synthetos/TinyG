"""Override state/byte/ticket infrastructure, without publishing or changing a trajectory."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]

class OverrideInfrastructure(unittest.TestCase):
    def test_state_bytes_tickets_and_window_composition(self):
        code=r'''
#include <assert.h>
#include <math.h>
#include <string.h>
#include "override.h"
static ovSnapshot_t s;
static void snap(void){ov_snapshot(&s);}
static void text(const unsigned char *p,unsigned n){for(unsigned i=0;i<n;i++)assert(!ov_usb_byte(p[i],true,true));}
int main(void){
 ov_init();snap();assert(s.feed==1000&&s.spindle==1000&&s.committed_feed==1000&&!s.armed&&!s.enables);
 for(unsigned i=0;i<256;i++)assert(!ov_usb_byte(i,true,true));
 assert(ov_set_factor(OV_FEED,0.5f,false,true)==OV_OK);snap();assert(s.feed==1000);
 assert(ov_set_factor(OV_FEED,NAN,true,true)==OV_RANGE);
 assert(ov_set_factor(OV_FEED,0.049f,true,true)==OV_RANGE);
 assert(ov_set_factor(OV_FEED,2.001f,true,true)==OV_RANGE);
 assert(ov_set_factor(OV_FEED,0.5f,true,false)==OV_DENIED);
 assert(ov_arm(true,false,true,true)==OV_DENIED);
 assert(ov_arm(true,true,false,true)==OV_DENIED);
 assert(ov_arm(true,true,true,false)==OV_OK);snap();assert(!s.armed);
 assert(ov_arm(true,true,true,true)==OV_OK);
 assert(ov_usb_byte(0x92,true,true));snap();assert(s.feed==900&&(s.enables&OV_FEED));
 uint32_t gen=s.request;
 assert(!ov_usb_byte(0xE2,true,true));assert(!ov_usb_byte(0x80,true,true));assert(!ov_usb_byte(0x9C,true,true));
 snap();assert(s.request==gen&&s.spindle==1000); /* UTF-8 smart quote is never a spindle knob */
 const unsigned char unicode[]={0xC2,0x90,0xF0,0x90,0x80,0x9D};text(unicode,sizeof unicode);
 const unsigned char bad_unicode[]={0xE0,0x80,0x9C};text(bad_unicode,sizeof bad_unicode);
 snap();assert(s.request==gen);assert(!ov_usb_byte('\n',true,true));
 assert(ov_usb_byte(0x9C,true,true));snap();assert(s.spindle==1010);
 assert(ov_usb_byte(0x95,true,true));snap();assert(s.unsupported==1&&s.feed==900&&s.state==OV_UNSUPPORTED);
 assert(ov_usb_byte(0x91,false,true));snap();assert(s.feed==900); /* alarm: consumed but not accepted */
 assert(!ov_usb_byte(0x91,true,false));snap();assert(s.feed==900); /* damaged UART byte is not a knob */
 assert(ov_set_factor(OV_FEED,0.05f,true,true)==OV_OK);
 assert(ov_usb_byte(0x94,true,true));snap();assert(s.feed==50);
 assert(ov_set_factor(OV_SPINDLE,2.0f,true,true)==OV_OK);
 assert(ov_usb_byte(0x9A,true,true));snap();assert(s.spindle==2000);
 uint32_t request=s.request;assert(ov_set_factor(OV_FEED,0.05f,true,true)==OV_OK);snap();assert(s.request==request);
 ov_cancel();snap();assert(s.feed==50&&s.spindle==2000&&s.committed_feed==1000&&s.committed_spindle==1000);
 /* A ticket is a future analytical boundary, never an average-velocity coincidence. */
 ovBoundary_t b={.incarnation=7,.ordinal=100,.stop_epoch=3,.kind=OV_BOUNDARY_HEAD,.eligible=true,.lead_ticks=21,.velocity=10};
 ovTicket_t t;assert(!ov_ticket_make(&b,0xFFFFFFF0u,&t));
 b.kind=OV_BOUNDARY_TAIL;assert(!ov_ticket_make(&b,0xFFFFFFF0u,&t));
 b.kind=OV_BOUNDARY_BODY;assert(ov_ticket_make(&b,0xFFFFFFF0u,&t));
 assert(ov_ticket_valid(&t,&b,0xFFFFFFF1u));assert(!ov_ticket_valid(&t,&b,0x20u)); /* expires across tick wrap */
 b.incarnation++;assert(!ov_ticket_valid(&t,&b,0xFFFFFFF1u));b.incarnation--;
 b.stop_epoch++;assert(!ov_ticket_valid(&t,&b,0xFFFFFFF1u));b.stop_epoch--;
 ov_cancel();assert(!ov_ticket_valid(&t,&b,0xFFFFFFF1u));
 assert(ov_ticket_make(&b,10,&t));assert(ov_set_factor(OV_FEED,0.8f,true,true)==OV_OK);assert(!ov_ticket_valid(&t,&b,11));
 b.kind=OV_BOUNDARY_SECTION_END;assert(ov_ticket_make(&b,10,&t));b.eligible=false;assert(!ov_ticket_valid(&t,&b,11));
 /* A whole window joins both unchanged outer speeds and every internal junction exactly. */
 ovPlan_t p[2]={{.entry=10,.cruise=12,.exit=8,.head=1,.body=2,.tail=1,.incarnation=7,.slot=2},
               {.entry=8,.cruise=9,.exit=5,.head=1,.body=2,.tail=1,.incarnation=8,.slot=3}};
 ovReference_t r[2]={{.length=4,.vmax=15,.incarnation=7,.slot=2},{.length=4,.vmax=15,.incarnation=8,.slot=3}};
 assert(ov_window_valid(p,r,2,10,5));
 p[1].entry=7;assert(!ov_window_valid(p,r,2,10,5));p[1].entry=8; /* <= predecessor exit is NOT enough */
 p[0].entry=9;assert(!ov_window_valid(p,r,2,10,5));p[0].entry=10;
 p[1].exit=6;assert(!ov_window_valid(p,r,2,10,5));p[1].exit=5;
 p[0].body=1;assert(!ov_window_valid(p,r,2,10,5));p[0].body=2;
 p[0].cruise=16;assert(!ov_window_valid(p,r,2,10,5));p[0].cruise=12;
 r[0].incarnation++;assert(!ov_window_valid(p,r,2,10,5));r[0].incarnation--;
 p[0].head=NAN;assert(!ov_window_valid(p,r,2,10,5));
 /* Independently publishing either side of a changed junction is invalid; the whole window is coherent. */
 p[0].head=1;p[0].exit=4;assert(!ov_window_valid(p,r,2,10,5));
 p[1].entry=4;assert(ov_window_valid(p,r,2,10,5));
 p[0].exit=8;assert(!ov_window_valid(p,r,2,10,5));
 /* Slot wrap, all K sizes, and every reference invalidated after computation. */
 for(unsigned n=1;n<=OV_WINDOW_MAX;n++) {
  ovPlan_t w[OV_WINDOW_MAX];ovReference_t ref[OV_WINDOW_MAX];float v=10;
  for(unsigned i=0;i<n;i++) {
   w[i]=(ovPlan_t){.entry=v,.cruise=20,.exit=9+i,.head=1,.body=2,.tail=1,.incarnation=100+i,.slot=(31+i)%32};
   ref[i]=(ovReference_t){.length=4,.vmax=25,.incarnation=100+i,.slot=(31+i)%32};v=w[i].exit;
  }
  assert(ov_window_valid(w,ref,n,10,v));
  for(unsigned i=0;i<n;i++){ref[i].incarnation++;assert(!ov_window_valid(w,ref,n,10,v));ref[i].incarnation--;}
 }
 ov_init();snap();assert(!s.armed&&s.feed==1000&&s.spindle==1000&&!s.enables);
}
'''
        with tempfile.TemporaryDirectory() as d:
            c,exe=Path(d)/'test.c',Path(d)/'test';c.write_text(code)
            subprocess.run(['cc','-std=gnu99','-O1','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fstrict-flex-arrays=3',
                            '-I'+str(ROOT/'firmware/tinyg'),str(c),str(ROOT/'firmware/tinyg/override.c'),'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
