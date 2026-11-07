"""Deterministic queue/loader handoff checks; no physical latency claim."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT = Path(__file__).resolve().parents[2]

class LateFeedhold(unittest.TestCase):
    def test_empty_queue_and_final_segment_handoff(self):
        source=(ROOT/'firmware/tinyg/canonical_machine.c').read_text()
        code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define __AVR 1
#define STAT_OK 0
#define SR_REQUEST_ASAP 1
enum {FEEDHOLD_OFF,FEEDHOLD_REQUESTED,FEEDHOLD_SYNC,FEEDHOLD_DECEL_TO_ZERO,FEEDHOLD_HOLD};
enum {MOTION_STOP,MOTION_RUN,MOTION_HOLD};
enum {PREP_BUFFER_OWNED_BY_EXEC,PREP_BUFFER_OWNED_BY_LOADER};
enum {FLUSH_OFF,FLUSH_REQUESTED};
enum {BLOCK_IDLE,BLOCK_RUNNING};
struct {int hold_state,motion_state,machine_state,queue_flush_state;bool end_hold_requested;} cm;
struct {int block_state;} mr;
struct {int buffer_state;} st_pre;
uint8_t SREG;
bool runnable,busy;int inject;
void fake_cli(void){
 if((SREG&128)&&inject){runnable=false;st_pre.buffer_state=PREP_BUFFER_OWNED_BY_LOADER;}
 SREG&=~128;
}
#define cli() fake_cli()
bool mp_has_runnable_buffer(void){return runnable;}
bool mp_runtime_is_idle(void){return !busy;}
bool st_runtime_isbusy(void){return busy;}
void cm_set_motion_state(int s){cm.motion_state=s;}
void sr_request_status_report(int v){(void)v;}
'''
        code+=function(source,r'void cm_start_hold\(\)(?=\s*\{)')
        code+=r'''
int main(void){
 for(int irq=0;irq<=128;irq+=128)for(int initial=FEEDHOLD_REQUESTED;initial<=FEEDHOLD_SYNC;initial++){
  for(int active=0;active<3;active++){
   cm.hold_state=initial;cm.motion_state=MOTION_RUN;cm.machine_state=12;cm.end_hold_requested=true;
   runnable=false;busy=(active==1);st_pre.buffer_state=active==2;SREG=irq;inject=0;
   cm_start_hold();assert(SREG==irq);assert(cm.machine_state==12);assert(cm.end_hold_requested);
   if(active)assert(cm.hold_state==initial);
   else {assert(cm.hold_state==FEEDHOLD_HOLD);assert(cm.motion_state==MOTION_HOLD);}
  }
 }
 runnable=true;busy=false;st_pre.buffer_state=0;cm.hold_state=FEEDHOLD_REQUESTED;SREG=128;
 cm_start_hold();assert(cm.hold_state==FEEDHOLD_SYNC&&cm.motion_state==MOTION_HOLD);assert(SREG==128);
 // Pending exec/loader wins just before cli: the final segment must finish before HOLD.
 runnable=true;inject=1;cm.hold_state=FEEDHOLD_REQUESTED;cm.motion_state=MOTION_RUN;SREG=128;
 cm_start_hold();assert(cm.hold_state==FEEDHOLD_REQUESTED);assert(SREG==128);
 inject=0;st_pre.buffer_state=0;cm_start_hold();assert(cm.hold_state==FEEDHOLD_HOLD);
 // Already decelerating or stopped must be owned by the existing hold state machine.
 for(int state=FEEDHOLD_DECEL_TO_ZERO;state<=FEEDHOLD_HOLD;state++){
  cm.hold_state=state;runnable=true;cm_start_hold();assert(cm.hold_state==state);
 }
}
'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'check.c').write_text(code)
            subprocess.run(['cc','-std=c99','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(p/'check.c'),'-o',str(p/'check')],check=True)
            subprocess.run([str(p/'check')],check=True)
