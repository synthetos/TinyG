"""Exercise shipped NV wrappers and USB ISR with the actual record-only override module."""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
from test_rx_overrun_damaged_line import PRELUDE
ROOT=Path(__file__).resolve().parents[2]
FW=ROOT/'firmware/tinyg'
class OverrideBindings(unittest.TestCase):
 def run_c(self,code):
  with tempfile.TemporaryDirectory() as d:
   c,exe=Path(d)/'test.c',Path(d)/'test';c.write_text(code)
   subprocess.run(['cc','-std=gnu99','-O1','-Wall','-Wno-unused-function','-fsanitize=address,undefined','-fstrict-flex-arrays=3',
                   '-I'+str(FW),str(c),str(FW/'override.c'),'-lm','-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
 def test_usb_filters_only_opted_in_standalone_knobs(self):
  code='#define OVERRIDE_FIXTURE_STUBS\n'+PRELUDE+'\n#include "override.h"\n#define USART_RX_DATA_READY_bm 0x80\n#define USART_FERR_bm 0x10\n#define USART_PERR_bm 0x04\n#define STAT_OK 0\nstatic unsigned alarm_checks;static int cm_is_alarmed(void){alarm_checks++;return 0;}\n'
  code=code.replace('static void cm_request_feedhold(void) {}','static void cm_request_feedhold(void) {ov_cancel();}')
  # The function above needs its declaration before the fixture's emergency callbacks.
  code=code.replace('static void cm_request_end_hold(void) {}','static void cm_request_end_hold(void) {resumes++;}')
  code='#include "override.h"\nstatic unsigned resumes;\n'+code
  usart=(FW/'xio/xio_usart.c').read_text();usb=(FW/'xio/xio_usb.c').read_text()
  for source,sig in ((usart,r'buffer_t xio_get_rx_bufcount_usart\('),(usart,r'void xio_xon_usart\('),
                     (usart,r'void xio_fc_usart\('),(usb,r'static void _usb_rx_lost\('),
                     (usb,r'static bool _usb_override_byte\('),(usb,r'ISR\(USB_RX_ISR_vect\)'),(usb,r'void xio_reset_usb_rx_buffers\('),(usb,r'uint8_t xio_usb_input_boundary\(')):
   code+=function(source,sig)+'\n'
  code+='void xio_xoff_usart(xioUsart_t *d){(void)d;}\n'
  code+=r'''
static void send(unsigned char c){uart.DATA=c;USB_RX_ISR_vect();}
int main(void){
 USB.x=&USBu;USB.x_flow=xio_fc_usart;USBu.usart=&uart;USBu.port=&port;
 xio_reset_usb_rx_buffers();ov_init();ovSnapshot_t s;
 send('G');send('1');send('\n');assert(alarm_checks==0); /* disabled path never asks permission */
 xio_reset_usb_rx_buffers();USBu.rx_buf_count=0;
 assert(xio_usb_input_boundary());uart.STATUS=USART_RX_DATA_READY_bm;assert(!xio_usb_input_boundary());uart.STATUS=0;
 send(0x9C);ov_snapshot(&s);assert(s.spindle==1000&&USBu.rx_buf_count==1);
 xio_reset_usb_rx_buffers();USBu.rx_buf_count=0;
 assert(ov_arm(true,true,true,true)==OV_OK);
 for(unsigned knob=0x90;knob<=0x9D;knob++) {
  if(knob==0x98)continue;
  xio_reset_usb_rx_buffers();USBu.rx_buf_count=0;cm.queue_flush_input=false;
  rx_quiet_ticks=RX_STALE_LINE_TICKS;
  send(knob);assert(!rx_in_line&&rx_quiet_ticks==RX_STALE_LINE_TICKS&&USBu.rx_buf_count==0);
  send('%');assert(cm.queue_flush_input&&USBu.rx_buf_count==0);
  unsigned before=resumes;send('~');assert(resumes==before+1&&USBu.rx_buf_count==0);
  send('G');send('1');unsigned count=USBu.rx_buf_count;uint8_t head=USBu.rx_buf_head;
  rx_quiet_ticks=RX_STALE_LINE_TICKS;
  send(knob);assert(rx_in_line&&rx_quiet_ticks==RX_STALE_LINE_TICKS&&USBu.rx_buf_count==count&&USBu.rx_buf_head==head);
  send('{');assert(USBu.rx_buf_count==count+2); /* E195 mark precedes the opening brace */
  assert((uint8_t)USBu.rx_buf[head-1]==RX_STALE_LINE_MARK);
 }
 xio_reset_usb_rx_buffers();USBu.rx_buf_count=0;ov_init();assert(ov_arm(true,true,true,true)==OV_OK);
 send(0xE2);send(0x80);send('!');send(0x9C); /* a hold invalidates tickets, not UTF-8 continuation */
 ov_snapshot(&s);assert(s.spindle==1000&&USBu.rx_buf_count==3);
 send('\n');unsigned kept=USBu.rx_buf_count;
 send(0x9C);ov_snapshot(&s);assert(s.spindle==1010&&USBu.rx_buf_count==kept);
 send('G');send('1');send('\n');assert(USBu.rx_buf_count==kept+3);
 return 0;
}
'''
  self.run_c(code)
 def test_runtime_wrappers_check_without_applying_and_sr_reads(self):
  source=(FW/'runtime.c').read_text()
  code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "override.h"
typedef uint8_t stat_t;
enum {TYPE_INTEGER=2,TYPE_FLOAT=4};
#define STAT_OK 0
#define STAT_UNRECOGNIZED_NAME 100
#define STAT_COMMAND_NOT_ACCEPTED 106
#define STAT_INPUT_VALUE_RANGE_ERROR 110
typedef struct {uint8_t valuetype;union{float value_flt;uint32_t value_int;};int8_t precision;} nvObj_t;
struct {bool allowed,check_only;} rt;
static bool quiet=true,boundary=true,pending=false;static stat_t alarm;
static uint8_t SREG=128;static void cli(void){SREG=0;}
#define __AVR 1
static bool _machine_quiet(void){return quiet;}
static bool xio_rx_data_pending(void){return pending;}
static bool xio_usb_input_boundary(void){return boundary;}
static stat_t cm_is_alarmed(void){return alarm;}
'''
  for name in ('_ov_result','_ov_float','ov_get_mfo','ov_get_sso','ov_get_mfc','ov_get_ssc','ov_get_state',
               '_ov_factor','rt_set_mfo','rt_set_sso','_ov_enable','rt_set_mfe','rt_set_sse','rt_get_mfe','rt_get_sse','rt_get_ovb','rt_set_ovb'):
   code+=function(source,r'(?:static )?stat_t '+name+r'\(')+'\n'
  code+=r'''
int main(void){
 ov_init();nvObj_t n={.valuetype=TYPE_FLOAT,.value_flt=0.8};
 assert(rt_set_mfo(&n)==100);rt.allowed=true;rt.check_only=true;assert(rt_set_mfo(&n)==0);
 rt.check_only=false;ov_get_mfo(&n);assert(n.value_flt==1&&n.precision==3);
 n.value_flt=0.8;assert(rt_set_mfo(&n)==0);ov_get_mfo(&n);assert(fabsf(n.value_flt-0.8f)<1e-6);
 /* A live request does not require a quiet machine: hold/motion may record it,
    but the committed readout remains unchanged until an execution handoff. */
 quiet=false;n.valuetype=TYPE_FLOAT;n.value_flt=0.6f;assert(rt_set_mfo(&n)==0);
 ov_get_mfo(&n);assert(fabsf(n.value_flt-0.6f)<1e-6);
 ov_get_mfc(&n);assert(n.value_flt==1);
 for(unsigned i=0;i<4;i++){
  n.valuetype=TYPE_FLOAT;n.value_flt=(float[]){0.049f,2.001f,NAN,INFINITY}[i];
  assert(rt_set_mfo(&n)==STAT_INPUT_VALUE_RANGE_ERROR);
  ov_get_mfo(&n);assert(fabsf(n.value_flt-0.6f)<1e-6);
 }
 n.valuetype=TYPE_FLOAT;n.value_flt=0.8f;assert(rt_set_mfo(&n)==0);

 quiet=false;n.valuetype=TYPE_INTEGER;n.value_int=1;assert(rt_set_ovb(&n)==106);
 quiet=true;boundary=false;assert(rt_set_ovb(&n)==106);boundary=true;pending=true;assert(rt_set_ovb(&n)==106);
 pending=false;assert(rt_set_ovb(&n)==0);assert(SREG==128);
 alarm=204;n.valuetype=TYPE_FLOAT;n.value_flt=0.5;assert(rt_set_mfo(&n)==204);
 rt.allowed=false;ov_get_mfo(&n);assert(fabsf(n.value_flt-0.8f)<1e-6); /* status report getter */
 ov_get_mfc(&n);assert(n.value_flt==1);ov_get_ssc(&n);assert(n.value_flt==1); /* no engine yet */
}
'''
  self.run_c(code)
 def test_rt_scan_keeps_opt_in_exclusive_and_rejects_duplicates(self):
  source=(FW/'json_parser.c').read_text()
  code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define NUL 0
#define RT_ENVELOPE_NONE 0
#define RT_ENVELOPE_PURE 1
#define RT_ENVELOPE_BAD 2
'''
  for name in ('_is_key_char','_scan_rt'):
   code+=function(source,r'static (?:bool|uint8_t) '+name+r'\(')+'\n'
  code+=r'''
int main(void){
 assert(_scan_rt("{\"rt\":{\"mfo\":0.8,\"mfe\":1,\"sso\":1.2,\"sse\":1}}") == RT_ENVELOPE_PURE);
 assert(_scan_rt("{\"rt\":{\"mfo\":0.8,\"mfo\":0.9}}") == RT_ENVELOPE_BAD);
 assert(_scan_rt("{\"rt\":{\"ovb\":1},\"tid\":7}") == RT_ENVELOPE_PURE);
 assert(_scan_rt("{\"rt\":{\"ovb\":1,\"mfo\":0.8}}") == RT_ENVELOPE_BAD);
 assert(_scan_rt("{\"rt\":{\"mfo\":0.8},\"xvm\":1}") == RT_ENVELOPE_BAD);
 assert(_scan_rt("{\"rtmfo\":0.8}") == RT_ENVELOPE_NONE);
}
'''
  self.run_c(code)
