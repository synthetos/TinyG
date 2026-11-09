"""A second latched overflow must not re-enter a stopped DDA/dwell timer at count zero.

Real AVR ISR bodies, with W1C flags and an overflow latched immediately before clock-off.
The callback can also start a new segment: clearing after the load would lose its first event.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function

FW = Path(__file__).resolve().parents[2] / 'firmware/tinyg'


class TerminalTimerIRQ(unittest.TestCase):
    def test_real_isrs_with_a_late_overflow(self):
        src = (FW / 'stepper.c').read_text()
        code = r'''
#include <cassert>
#include <cstdint>
#define __AVR 1
#define ISR(name) void name()
#define TIMER_DDA_ISR_vect dda_isr
#define TIMER_DWELL_ISR_vect dwell_isr
#define STEP_TIMER_DISABLE 0
#define TC0_OVFIF_bm 1
struct Flags {
 unsigned bits=0;
 void operator=(unsigned mask){bits &= ~mask;} // write one to clear, not an ordinary RAM register
};
struct Timer {
 struct Clock {
  unsigned value=1; Timer *owner;
  void operator=(unsigned v) {
   if(v==0 && owner->inject){owner->INTFLAGS.bits |= 1;owner->inject=false;}
   value=v;
  }
  operator unsigned() const {return value;}
 } CTRLA;
 Flags INTFLAGS;
 bool inject=true;
 Timer():CTRLA{1,this}{}
} TIMER_DDA,TIMER_DWELL;
static Timer *active;
static unsigned loads,pulses;
static bool rearm;
struct Motor {int substep_accumulator=0, substep_increment=1;};
struct {uint32_t dda_ticks_downcount=0,dda_ticks_X_substeps=1;Motor mot[4];} st_run;
struct Port {unsigned OUT=0;} ports[4];
#define MOTOR_1 0
#define MOTOR_2 1
#define MOTOR_3 2
#define MOTOR_4 3
#define PORT_MOTOR_1_VPORT ports[0]
#define PORT_MOTOR_2_VPORT ports[1]
#define PORT_MOTOR_3_VPORT ports[2]
#define PORT_MOTOR_4_VPORT ports[3]
#define STEP_BIT_bm 1
#define INCREMENT_ENCODER(m) do {if((m)==0)pulses++;} while(0)
static void _load_move() {
 loads++;
 if(rearm){
  rearm=false;st_run.dda_ticks_downcount=2;active->CTRLA=1;
  active->INTFLAGS.bits |= 1; // a NEW segment's first overflow, after its clock is restarted
 }
}
'''
        code += function(src, r'ISR\(TIMER_DDA_ISR_vect\)') + '\n'
        code += function(src, r'ISR\(TIMER_DWELL_ISR_vect\)') + '\n'
        code += r'''
static void service(void (*isr)()) {
 unsigned entries=0;
 while(active->INTFLAGS.bits & 1){
  assert(++entries<5);active->INTFLAGS.bits &= ~1u; // hardware acknowledges on ISR entry
  isr();
 }
}
int main() {
 for(unsigned dwell=0;dwell<2;dwell++)for(unsigned next=0;next<2;next++) {
  active=dwell?&TIMER_DWELL:&TIMER_DDA;
  active->CTRLA.value=1;active->inject=true;active->INTFLAGS.bits=0;
  loads=pulses=0;rearm=next;st_run.dda_ticks_downcount=1;
  active->INTFLAGS.bits=1;service(dwell?dwell_isr:dda_isr);
  if(next) {
   assert(st_run.dda_ticks_downcount==1); // restarted segment's first event was preserved
   active->INTFLAGS.bits=1;service(dwell?dwell_isr:dda_isr);
  }
  assert(st_run.dda_ticks_downcount==0); // stock underflows to UINT32_MAX, blocking HOLD/flush forever
  assert(active->CTRLA.value==0 && active->INTFLAGS.bits==0);
  assert(loads==1+next);
  if(!dwell)assert(pulses==1+2*next); // no ghost pulse after terminal clock-off
 }
}
'''
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'timer.cpp';path.write_text(code);exe=path.with_suffix('')
            subprocess.run(['c++','-std=c++17','-Wall','-Werror','-fsanitize=address,undefined',
                            '-fstrict-flex-arrays=3',str(path),'-o',str(exe)],check=True)
            result=subprocess.run([str(exe)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
