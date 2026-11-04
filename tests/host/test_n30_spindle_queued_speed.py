"""N30: S queued before M30 uses the driven mode, despite M30's early model reset."""
import unittest
import test_spindle_coolant_status as fixture

class QueuedSpeed(unittest.TestCase):
    build_run=fixture.SpindleCoolantStatus.build_run
    # Reuse the source-extracted output callback fixture, not its test cases.
    def test_s_uses_output_mode_after_model_reset(self):
        c=fixture.code().replace('static float cm_get_spindle_pwm(uint8_t mode) {',
                         'static uint8_t pwm_mode;\nstatic float cm_get_spindle_pwm(uint8_t mode) { pwm_mode=mode;')
        c+=r'''
static void loader_s_update(void) {}
int main(void) {
 for(int direction=SPINDLE_CW;direction<=SPINDLE_CCW;direction++) {
  float initial=1000, mode=direction, next=5000;
  _exec_spindle_speed(&initial,0);_exec_spindle_control(&mode,0);
  cm.gm.spindle_mode=SPINDLE_OFF; /* cm_program_end parsed while older commands queued */
  _exec_spindle_speed(&next,0);
  assert(pwm_mode==direction);
  assert(spindle_out.mode==direction&&spindle_out.speed==5000);
  assert(cm.gm.spindle_mode==SPINDLE_OFF); /* keep the ahead-of-runtime model reset */
  spindle_reset();next=7000;_exec_spindle_speed(&next,0);
  assert(pwm_mode==SPINDLE_OFF&&spindle_out.speed==0);
 }
}
'''
        self.build_run(c,'queued')
