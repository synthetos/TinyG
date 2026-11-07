"""The shipped loader and DDA produce one leading pulse for opposite boot DIR phases."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class FirstSegmentPhase(unittest.TestCase):
    def test_opposite_polarities_converge_after_first_tick(self):
        source = (ROOT / "firmware/tinyg/stepper.c").read_text()
        start = source.index("// mirror the accumulator on a reversal of travel only")
        end = source.index("SET_ENCODER_STEP_SIGN(MOTOR_1", start)
        load = source[start:end].replace("MOTOR_1", "m").replace("PORT_m_VPORT", "port[m]")
        start = source.index("if ((st_run.mot[MOTOR_1].substep_accumulator +=")
        end = source.index("\n\tif ((st_run.mot[MOTOR_2]", start)
        tick = source[start:end].replace("MOTOR_1", "m").replace("PORT_m_VPORT", "port[m]")
        code = r'''
#include <assert.h>
#include <stdint.h>
enum { DIRECTION_CW=0, DIRECTION_CCW=1, DIRECTION_BIT_bm=2, STEP_BIT_bm=1 };
struct {struct {int prev_step_sign,step_sign,direction,prev_direction;} mot[2];} st_pre;
struct {int32_t dda_ticks_X_substeps;struct {int32_t substep_accumulator;uint32_t substep_increment;} mot[2];} st_run;
struct {unsigned OUT;} port[2];
static int pulses[2];
#define INCREMENT_ENCODER(m) pulses[m]++
static void load(int m){
''' + load + "\n}\nstatic void tick(int m){\n" + tick + r'''
}
int main(void){
 st_run.dda_ticks_X_substeps=1000000;
 for(int m=0;m<2;m++){
  st_pre.mot[m].step_sign=1;st_pre.mot[m].direction=m;
  st_run.mot[m].substep_increment=10000;
  load(m);
 }
 assert(st_run.mot[0].substep_accumulator==0);
 assert(st_run.mot[1].substep_accumulator==-1000000);
 tick(0);tick(1);
 assert(pulses[0]==1&&pulses[1]==0);
 assert(st_run.mot[0].substep_accumulator==st_run.mot[1].substep_accumulator);
 for(int i=0;i<10000;i++){
  int before=pulses[0]-pulses[1];
  tick(0);tick(1);
  assert(pulses[0]-pulses[1]==before);
  assert(st_run.mot[0].substep_accumulator==st_run.mot[1].substep_accumulator);
 }
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c, exe = Path(tmp)/"phase.c", Path(tmp)/"phase"
            c.write_text(code)
            subprocess.run(["cc", "-std=c99", "-fsanitize=address,undefined", str(c), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
