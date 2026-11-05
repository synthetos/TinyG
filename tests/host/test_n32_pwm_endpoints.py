"""N32: actual AVR duty setter on a register fixture, including PER=65535."""
from pathlib import Path
import subprocess, tempfile, unittest
from source_extract import function
import test_pwm_speed_range as fixture
ROOT=Path(__file__).resolve().parents[2]
PRE=fixture.PRELUDE
class PwmEndpoints(unittest.TestCase):
 def test_endpoints_and_resume(self):
  code=PRE+function((ROOT/'firmware/tinyg/pwm.c').read_text(),r'stat_t pwm_set_duty\([^;{}]*\)(?=\s*\{)')+r'''
int main(void){
 pwm.p[0].timer=&timer;
 for(unsigned p=256;p<=65535;p+=(p==256?39744:25535)){
  timer.PER=p;timer.CTRLB=0x23;pin=1;
  assert(pwm_set_duty(0,0)==STAT_OK);
  assert(timer.CCB==0 && !(timer.CTRLB&TC0_CCBEN_bm) && pin==0);
  assert((timer.CTRLB&3)==3 && SREG==0x80);
  assert(pwm_set_duty(0,0.5f)==STAT_OK);
  assert(timer.CCB==(uint16_t)(p*0.5f)+1 && (timer.CTRLB&TC0_CCBEN_bm));
  assert(pwm_set_duty(0,1)==STAT_OK);
  assert(timer.CCB==p && !(timer.CTRLB&TC0_CCBEN_bm) && pin==1);
  assert(pwm_set_duty(0,0.25f)==STAT_OK && (timer.CTRLB&TC0_CCBEN_bm));
 }
 assert(pwm_set_duty(PWMS,0.5f)==STAT_NO_SUCH_DEVICE);
 SREG=0;assert(pwm_set_duty(0,0)==STAT_OK && SREG==0);
}
'''
  with tempfile.TemporaryDirectory() as d:
   src=Path(d)/'duty.c';src.write_text(code);exe=src.with_suffix('')
   subprocess.run(['cc','-std=c99','-fsanitize=address,undefined,float-cast-overflow','-fstrict-flex-arrays=3',str(src),'-o',str(exe),'-lm'],check=True)
   p=subprocess.run([str(exe)],capture_output=True,text=True)
   self.assertEqual(p.returncode,0,p.stderr)
if __name__=='__main__':unittest.main()
