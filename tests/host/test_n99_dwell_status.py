"""N99: report physical/queued work without changing machine state or planner credits."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = Path(__file__).resolve().parents[2] / 'firmware/tinyg'


class DwellStatus(unittest.TestCase):
    def test_real_combined_state(self):
        header = (ROOT / 'canonical_machine.h').read_text()
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define PSTR(s) s
#define STAT_STATE_MANAGEMENT_ASSERTION_FAILURE 1
'''
        for name in ('cmCombinedState', 'cmMachineState', 'cmCycleState', 'cmMotionState'):
            code += re.search(r'typedef enum \{[^{}]*\}\s*' + name + r';', header).group() + '\n'
        code += r'''
#ifdef TEST_AVR
#define __AVR 1
#endif
static uint8_t SREG;
static bool inject_alarm;
static void fake_cli(void) __attribute__((unused));
#define cli() fake_cli()
enum { PREP_BUFFER_OWNED_BY_EXEC, PREP_BUFFER_OWNED_BY_LOADER };
enum { BLOCK_TYPE_NULL, BLOCK_TYPE_ALINE, BLOCK_TYPE_DWELL, BLOCK_TYPE_COMMAND };
struct { cmMachineState machine_state; cmCycleState cycle_state;
         cmMotionState motion_state; bool program_paused; } cm;
struct { uint8_t buffer_state, block_type; } st_pre;
struct { uint32_t dda_ticks_downcount; } st_run;
struct { uint8_t buffer_state; } block;
struct { typeof(block) *r; } mb = { &block };
static void fake_cli(void) {
 if(inject_alarm && (SREG & 128)){cm.machine_state=MACHINE_ALARM;inject_alarm=false;}
 SREG &= ~128;
}
static void cm_panic_P(int status, const char *message) { (void)status;(void)message;assert(0); }
'''
        code += function((ROOT / 'stepper.c').read_text(), r'uint8_t st_runtime_isbusy\(') + '\n'
        code += function((ROOT / 'plan_line.c').read_text(), r'bool mp_runtime_is_idle\(') + '\n'
        code += function((ROOT / 'planner.c').read_text(), r'bool mp_has_runnable_buffer\(') + '\n'
        code += r'''
static bool snapshot_runtime_idle(void) {
#ifdef __AVR
 assert(!(SREG & 128));
#endif
 return mp_runtime_is_idle();
}
#define mp_runtime_is_idle snapshot_runtime_idle
'''
        code += function((ROOT / 'canonical_machine.c').read_text(), r'cmCombinedState cm_get_combined_state\(')
        code += r'''
static void check(int expected) {
 typeof(cm) before=cm;
 assert(cm_get_combined_state()==expected);
 assert(memcmp(&before,&cm,sizeof(cm))==0); // reporting never starts/ends a cycle
}
int main(void) {
 for(int irq=0;irq<=128;irq+=128) {
  SREG=irq;
  for(int state=MACHINE_READY;state<=MACHINE_PROGRAM_END;state++) {
   if(state==MACHINE_ALARM) continue;
   cm.machine_state=state;cm.cycle_state=CYCLE_OFF;cm.motion_state=MOTION_STOP;cm.program_paused=false;
   for(int source=0;source<4;source++) {
    st_run.dda_ticks_downcount=source==0 ? 0x10000 : 0;
    block.buffer_state=source==1;
    st_pre.buffer_state=source>=2 ? PREP_BUFFER_OWNED_BY_LOADER : PREP_BUFFER_OWNED_BY_EXEC;
    st_pre.block_type=source==2 ? BLOCK_TYPE_DWELL : BLOCK_TYPE_COMMAND;
    check(COMBINED_RUN); // lone dwell, queued command, loader dwell/command, M2-adjacent work
    assert(SREG==irq);
   }
   st_run.dda_ticks_downcount=0;block.buffer_state=0;
   st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;check(state);
   st_pre.buffer_state=PREP_BUFFER_OWNED_BY_LOADER;st_pre.block_type=BLOCK_TYPE_NULL;check(state);
  }
  // An M0 or enabled M1 keeps PROGRAM_STOP even with later work queued.
  cm.machine_state=MACHINE_PROGRAM_STOP;cm.program_paused=true;block.buffer_state=1;
  cm.motion_state=MOTION_HOLD;check(COMBINED_PROGRAM_STOP);
  cm.program_paused=false;cm.machine_state=MACHINE_CYCLE;cm.cycle_state=CYCLE_MACHINING;
  block.buffer_state=0;check(COMBINED_HOLD); // held lone dwell after its timer expired
  cm.motion_state=MOTION_STOP;
  cm.cycle_state=CYCLE_HOMING;check(COMBINED_HOMING);
  cm.cycle_state=CYCLE_PROBE;check(COMBINED_PROBE);
  cm.cycle_state=CYCLE_JOG;check(COMBINED_JOG);
  st_run.dda_ticks_downcount=256;block.buffer_state=1;
  int states[]={MACHINE_INITIALIZING,MACHINE_ALARM,MACHINE_INTERLOCK,MACHINE_SHUTDOWN,MACHINE_PANIC};
  int combined[]={COMBINED_INITIALIZING,COMBINED_ALARM,COMBINED_INTERLOCK,COMBINED_SHUTDOWN,COMBINED_PANIC};
  for(int n=0;n<5;n++){cm.machine_state=states[n];check(combined[n]);}
#ifdef __AVR
  if(irq) {
   cm.machine_state=MACHINE_READY;inject_alarm=true;
   assert(cm_get_combined_state()==COMBINED_ALARM);
   assert(cm.machine_state==MACHINE_ALARM && !inject_alarm && SREG==irq);
  }
#endif
 }
}
'''
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'status.c'
            path.write_text(code)
            for avr in (False, True):
                exe = Path(d) / ('avr' if avr else 'host')
                subprocess.run(['cc', '-std=gnu99', '-Wall', '-Werror',
                                '-fsanitize=address,undefined', '-fstrict-flex-arrays=3',
                                *(['-DTEST_AVR'] if avr else []), str(path), '-o', str(exe)], check=True)
                result = subprocess.run([str(exe)], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
