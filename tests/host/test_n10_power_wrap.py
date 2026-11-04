"""N10: motor idle time is elapsed time, including through 32-bit tick wrap.

TinyG wiki TinyG-Configuration-for-Firmware-Version-0.97, $MT promises power
for the configured time after the last move. Runs the real power callback.
"""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
from test_e184_motor_power_check import POWER_PRELUDE,DEF
ROOT=Path(__file__).resolve().parents[2]/'firmware/tinyg'
class PowerWrap(unittest.TestCase):
    def test_elapsed_countdown(self):
        source=(ROOT/'stepper.c').read_text()
        code=POWER_PRELUDE.replace('uint32_t power_systick;','uint32_t power_systick,power_duration;')
        for sig in (r'static void _deenergize_motor\(',r'static void _energize_motor\(',r'static uint8_t _motor_is_enabled\(',r'stat_t st_motor_power_callback\('):
            code+=function(source,sig+DEF)
        code+=r'''
int main(void) {
 for(int m=0;m<MOTORS;m++) st_cfg.mot[m].power_mode=MOTOR_POWERED_IN_CYCLE;
 uint32_t starts[]={0xfffffff0u,1000};
 for(unsigned i=0;i<2;i++) {
  st_cfg.motor_power_timeout=2; now=starts[i];loader_starts_a_segment();
  st_motor_power_callback();assert(_motor_is_enabled(0));assert(st_runtime_isbusy());
  st_run.dda_ticks_downcount=0;motion_state=MOTION_STOP;
  // Changing mt does not change an already running countdown.
  st_cfg.motor_power_timeout=.1f;
  now=starts[i]+1999u;st_motor_power_callback();assert(_motor_is_enabled(0));
  now=starts[i]+2000u;st_motor_power_callback();assert(!_motor_is_enabled(0));
 }
 // Long legal durations beyond INT32_MAX also work; signed deadline tests do not.
 st_cfg.motor_power_timeout=3000000; now=0xfffffff0u;loader_starts_a_segment();
 st_motor_power_callback();assert(_motor_is_enabled(0));
 st_run.dda_ticks_downcount=0;motion_state=MOTION_STOP;
 now=0xfffffff0u+2999999999u;st_motor_power_callback();assert(_motor_is_enabled(0));
 now++;st_motor_power_callback();assert(!_motor_is_enabled(0));
}
'''
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'power.c';exe=Path(d)/'power';p.write_text(code)
            subprocess.run(['cc','-std=c99','-fsanitize=address,undefined',str(p),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
