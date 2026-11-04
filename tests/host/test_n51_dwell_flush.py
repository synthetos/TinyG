"""Flush discards an active/prepared dwell, never a stepping line; real AVR path."""
from pathlib import Path
import subprocess
import tempfile
from source_extract import function, RUNTIME_STUBS
from test_queue_flush_input import PRELUDE
ROOT = Path(__file__).resolve().parents[2] / 'firmware/tinyg'


def test_flush_cancels_dwell_before_waiting_for_runtime():
    code = PRELUDE.replace('bool mp_runtime_is_idle(void) { return true; }',
                           'bool mp_runtime_is_idle(void) { return st_run.dda_ticks_downcount == 0; }')
    code = code.replace('void st_prep_null(void) {}', '''void st_prep_null(void) {
        assert(!(SREG & 128)); st_pre.block_type=BLOCK_TYPE_NULL;
        st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;
    }''')
    declarations = r'''
enum {BLOCK_TYPE_NULL, BLOCK_TYPE_ALINE, BLOCK_TYPE_DWELL};
enum {PREP_BUFFER_OWNED_BY_EXEC=1, PREP_BUFFER_OWNED_BY_LOADER};
#define STEP_TIMER_DISABLE 0
#define TC0_OVFIF_bm 1
struct {unsigned CTRLA,CNT,INTFLAGS;} TIMER_DWELL,TIMER_DDA;
struct {unsigned dda_ticks_downcount;} st_run;
struct {unsigned block_type,buffer_state,dda_ticks;} st_pre;
bool st_aline_in_flight(void) {return TIMER_DDA.CTRLA ||
 (st_pre.buffer_state==PREP_BUFFER_OWNED_BY_LOADER && st_pre.block_type==BLOCK_TYPE_ALINE);}
'''
    code=code.replace('uint8_t SREG = 0x80;',declarations+'\nuint8_t SREG = 0x80;')
    stepper=(ROOT/'stepper.c').read_text()
    if 'void st_cancel_dwell(void)' in stepper:
        code+=function(stepper,r'void st_cancel_dwell\(void\)')
    else:
        code+='void st_cancel_dwell(void) {}\n'  # baseline: no cancellation
    code+=RUNTIME_STUBS
    source=(ROOT/'canonical_machine.c').read_text()
    for sig in [r'void cm_request_queue_flush\(\)',r'void cm_request_planner_flush\(\)',
                r'static void _take_input_flush_request\(\)',r'static void _service_input_flush\(\)',
                r'void cm_queue_flush\(\)',r'static void _run_requested_queue_flush\(\)']:
        code+=function(source,sig+r'(?=\s*\{)')
    code+=r'''
int main(void) {
 // A lone dwell has freed all planner buffers but still owns its timer.
 cm.hold_state=FEEDHOLD_HOLD;TIMER_DWELL.CTRLA=1;TIMER_DWELL.CNT=7;
 st_run.dda_ticks_downcount=30000;st_pre.block_type=BLOCK_TYPE_NULL;
 st_pre.buffer_state=PREP_BUFFER_OWNED_BY_LOADER;
 cm_request_queue_flush();_run_requested_queue_flush();
 assert(!TIMER_DWELL.CTRLA && !st_run.dda_ticks_downcount);
 assert(!TIMER_DWELL.CNT && TIMER_DWELL.INTFLAGS==TC0_OVFIF_bm);
 assert(planner_flushes==1 && input_flushes==1 && cm.queue_flush_state==FLUSH_OFF);
 assert(cm.hold_state==FEEDHOLD_OFF && SREG==128);
 // Alarm-only flush cancels too, while retaining input for alarm rejection.
 cm.hold_state=FEEDHOLD_HOLD;TIMER_DWELL.CTRLA=1;st_run.dda_ticks_downcount=10000;
 cm_request_planner_flush();_run_requested_queue_flush();
 assert(!st_run.dda_ticks_downcount && planner_flushes==2 && input_flushes==1);
 // A staged dwell must never load after a direct flush.
 cm.hold_state=FEEDHOLD_HOLD;st_pre.block_type=BLOCK_TYPE_DWELL;
 st_pre.buffer_state=PREP_BUFFER_OWNED_BY_LOADER;cm_queue_flush();
 assert(st_pre.block_type==BLOCK_TYPE_NULL && st_pre.buffer_state==PREP_BUFFER_OWNED_BY_EXEC);
 // Preserve both interrupt states; do not cancel a moving/prepared ALINE.
 for(unsigned irq=0;irq<=128;irq+=128) {
  SREG=irq;TIMER_DWELL.CTRLA=1;st_run.dda_ticks_downcount=123;
  st_cancel_dwell();assert(!st_run.dda_ticks_downcount && SREG==irq);
  TIMER_DDA.CTRLA=1;st_run.dda_ticks_downcount=456;
  st_cancel_dwell();assert(st_run.dda_ticks_downcount==456 && TIMER_DDA.CTRLA && SREG==irq);
  TIMER_DDA.CTRLA=0;st_pre.block_type=BLOCK_TYPE_ALINE;
  st_pre.buffer_state=PREP_BUFFER_OWNED_BY_LOADER;st_cancel_dwell();
  assert(st_pre.block_type==BLOCK_TYPE_ALINE && st_run.dda_ticks_downcount==456);
  st_pre.block_type=BLOCK_TYPE_NULL;st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;
 }
}
'''
    with tempfile.TemporaryDirectory() as d:
        p=Path(d);(p/'check.c').write_text(code)
        subprocess.run(['cc','-std=c99','-Wall','-fsanitize=address,undefined',str(p/'check.c'),'-o',str(p/'check')],check=True)
        subprocess.run([str(p/'check')],check=True)
