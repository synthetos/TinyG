"""N11: a full USB TX ring must not delay a requested hold or job kill.

Real putc, hold-start, alarm-entry and sequencing code; sleep models interrupt
progress and releases TX only after checking that motion can decelerate.
"""
from pathlib import Path
import re, subprocess, tempfile, unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]
PRE=r'''
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
typedef int stat_t; typedef uint8_t buffer_t;
#define __AVR
#define PSTR(s) s
#define XIO_OK 0
#define STAT_OK 0
#define STAT_KILL_JOB 207
#define TX_BUFFER_SIZE 8
#define CR '\r'
#define CTRLA_RXON_TXOFF 1
#define CTRLA_RXON_TXON 2
enum {MACHINE_CYCLE,MACHINE_ALARM,MACHINE_PANIC,MACHINE_SHUTDOWN};
enum {MOTION_STOP,MOTION_RUN,MOTION_HOLD};
enum {FEEDHOLD_OFF,FEEDHOLD_REQUESTED,FEEDHOLD_SYNC,FEEDHOLD_HOLD};
enum {FLUSH_OFF,FLUSH_REQUESTED,PREP_BUFFER_OWNED_BY_LOADER};
struct {int machine_state,motion_state,hold_state,queue_flush_state;bool job_kill_requested,job_kill_report_pending,end_hold_requested;} cm;
struct {int buffer_state;} st_pre;
struct {uint8_t CTRLA;} uart;
struct {buffer_t tx_buf_head,tx_buf_tail;char tx_buf[8]; typeof(uart)*usart;} USBu;
struct {bool flag_crlf;} USB;
uint8_t SREG=0x80; static bool usb_activity;
static int reports,off,flushes,sleeps,scenario; static bool in_wait;
#define cli() (SREG &= ~0x80)
static bool mp_has_runnable_buffer(void){return true;}
static bool mp_runtime_is_idle(void){return false;}
static bool st_aline_in_flight(void){return true;}
static void cm_set_motion_state(int s){cm.motion_state=s;}
static void stepper_reset(void){}
static void cm_request_feedhold(void){if(cm.hold_state==FEEDHOLD_OFF)cm.hold_state=FEEDHOLD_REQUESTED;}
static void cm_request_planner_flush(void){cm.queue_flush_state=FLUSH_REQUESTED;}
static void spindle_reset(void){off++;}
static void cm_coolant_off_immediate(void){off++;}
static void rpt_exception(int s,const char*m){assert(!in_wait);assert(s==STAT_KILL_JOB);reports++;}
static void _run_requested_queue_flush(void){assert(!in_wait);flushes++;cm.queue_flush_state=FLUSH_OFF;}
static void _service_input_flush(void){assert(!in_wait);}
static void cm_end_hold(void){assert(!in_wait);}
static void sleep_mode(void){
  sleeps++;
  if(sleeps==3){
    assert(cm.hold_state==FEEDHOLD_SYNC);assert(cm.motion_state==MOTION_HOLD);
    if(scenario){assert(cm.machine_state==MACHINE_ALARM);assert(off==2);}
    assert(!reports && !flushes); USBu.tx_buf_tail=7;
  }
  assert(sleeps<4);
}
'''
class TxHold(unittest.TestCase):
 def test_wait_services_stop_without_output_or_input_reentry(self):
  cm=(ROOT/'firmware/tinyg/canonical_machine.c').read_text()
  usb=(ROOT/'firmware/tinyg/xio/xio_usb.c').read_text()
  code=PRE
  # The baseline has no shared non-reporting alarm entry or blocked-TX callback.
  if re.search(r'static bool _enter_alarm\(void\)',cm):
   code+=function(cm,r'static bool _enter_alarm\(void\)(?=\s*\{)')
  code+=function(cm,r'stat_t cm_alarm\(const stat_t status, const char \*msg\)')
  code+='\nstatic int cm_alarm_P(int s,const char*m){return cm_alarm(s,m);}\n'
  code+=function(cm,r'void cm_start_hold\(\)(?=\s*\{)')
  if 'void cm_tx_blocked_callback(void)' in cm:
   code+=function(cm,r'void cm_tx_blocked_callback\(void\)(?=\s*\{)')
  code+=function(cm,r'stat_t cm_feedhold_sequencing_callback\(\)(?=\s*\{)')
  code+='\nstatic int _putc_usb_raw(const char c);\n'
  if '_putc_usb_wait' in usb: code+=function(usb,r'static int __attribute__\(\(noinline\)\) _putc_usb_wait\(const char c\)')
  code+=function(usb,r'static int _putc_usb_raw\(const char c\)(?=\s*\{)')
  code+=r'''
int main(void){
 for(scenario=0;scenario<2;scenario++)for(int cr=0;cr<2;cr++){
  memset(&cm,0,sizeof cm);off=reports=flushes=sleeps=0;
  cm.motion_state=MOTION_RUN;cm.hold_state=scenario?FEEDHOLD_OFF:FEEDHOLD_REQUESTED;
  cm.job_kill_requested=scenario;USBu.usart=&uart;USB.flag_crlf=cr;
  USBu.tx_buf_head=4;USBu.tx_buf_tail=cr?2:3;in_wait=true;
  assert(_putc_usb_raw(cr?'\n':'a')==0);in_wait=false;
  assert(sleeps==3 && SREG==0x80);
  cm_feedhold_sequencing_callback();cm_feedhold_sequencing_callback();
  assert(reports==scenario);assert(!cm.job_kill_report_pending);
 }
}
'''
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp)/'wait.c';p.write_text(code)
   subprocess.run(['cc','-std=gnu99','-fsanitize=address,undefined',str(p),'-o',str(p.with_suffix(''))],check=True)
   out=subprocess.run([str(p.with_suffix(''))],capture_output=True,text=True)
   self.assertEqual(out.returncode,0,out.stderr)
if __name__=='__main__':unittest.main()
