"""The late-overflow hook must wait until the final prepared braking segment has loaded."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function
FW=Path(__file__).resolve().parents[2]/'firmware/tinyg'

class Stimulus(unittest.TestCase):
    def test_pending_with_an_aline_in_prep_does_not_consume_the_hook(self):
        code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define __AVR 1
#define TINYG_TEST_HOOKS 1
#define ISR(x) void x()
#define TIMER_DDA_ISR_vect dda_isr
#define STEP_TIMER_DISABLE 0
#define TC0_OVFIF_bm 1
#define FEEDHOLD_PENDING 5
#define BLOCK_TYPE_NULL 0
#define BLOCK_TYPE_ALINE 1
struct {int hold_state;} cm={5};
struct {uint8_t block_type,buffer_state;} st_pre={1,1};
struct {uint8_t dda_finish_delay,dda_finish_delays,dda_finish_pre_type,dda_finish_flags,
               dda_finish_ctrl,dda_finish_flags_after_stop;
        uint16_t dda_finish_cnt,dda_finish_per;} test_timer;
struct {uint8_t CTRLA,INTFLAGS;uint16_t CNT,PER;} TIMER_DDA={1,0,0,640};
struct Motor {int substep_accumulator,substep_increment;};
struct {uint32_t dda_ticks_downcount,dda_ticks_X_substeps;struct Motor mot[4];} st_run;
struct Port {uint8_t OUT;} ports[4];
#define MOTOR_1 0
#define MOTOR_2 1
#define MOTOR_3 2
#define MOTOR_4 3
#define PORT_MOTOR_1_VPORT ports[0]
#define PORT_MOTOR_2_VPORT ports[1]
#define PORT_MOTOR_3_VPORT ports[2]
#define PORT_MOTOR_4_VPORT ports[3]
#define STEP_BIT_bm 1
#define INCREMENT_ENCODER(m) ((void)0)
static int delayed;
static void _delay_us(int us){assert(us==25);delayed++;TIMER_DDA.INTFLAGS|=1;}
static void _load_move(void){
 if(st_pre.block_type==BLOCK_TYPE_ALINE){st_run.dda_ticks_downcount=2;st_pre.block_type=0;TIMER_DDA.CTRLA=1;}
}
'''+function((FW/'stepper.c').read_text(),r'ISR\(TIMER_DDA_ISR_vect\)')+r'''
int main(void){
 test_timer.dda_finish_delay=1;st_run.dda_ticks_downcount=1;
 dda_isr(); // PENDING is set, but the final braking segment is still in prep.
 assert(!delayed && test_timer.dda_finish_delay==1 && !test_timer.dda_finish_delays);
 assert(st_run.dda_ticks_downcount==2 && st_pre.block_type==0);
 dda_isr();assert(!delayed && st_run.dda_ticks_downcount==1);
 dda_isr();assert(delayed==1 && !test_timer.dda_finish_delay && test_timer.dda_finish_delays==1);
}
'''
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'stim.c';path.write_text(code);exe=path.with_suffix('')
            subprocess.run(['cc','-std=c99','-Wall','-Werror','-fsanitize=address,undefined',
                            '-fstrict-flex-arrays=3',str(path),'-o',str(exe)],check=True)
            r=subprocess.run([str(exe)],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stderr)
