"""N32: actual AVR duty setter on a register fixture, including PER=65535."""
from pathlib import Path
import subprocess, tempfile, unittest
from source_extract import function
import test_pwm_speed_range as fixture
ROOT=Path(__file__).resolve().parents[2]
EXTRA=r'''
#define PWMS 1
#define STAT_NO_SUCH_DEVICE 4
#define TC0_CCBEN_bm 0x20
#define PWM1_CTRLB 0x23
#define PWM2_CTRLB 3
#define SPINDLE_PWM 2
static unsigned char SREG=0x80;
static void cli(void){SREG=0;}
static int pin;
static void gpio_set_bit_off(int bit){assert(bit==SPINDLE_PWM);pin=0;}
static void gpio_set_bit_on(int bit){assert(bit==SPINDLE_PWM);pin=1;}
'''
# Keep the older speed-range regression usable before and after the firmware fix.
PRE=fixture.PRELUDE.replace('uint16_t PER, CCB;', 'uint16_t PER, CCB; uint8_t CTRLB;')+EXTRA
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
   subprocess.run(['cc','-std=c99','-fsanitize=address,undefined,float-cast-overflow',str(src),'-o',str(exe),'-lm'],check=True)
   p=subprocess.run([str(exe)],capture_output=True,text=True)
   self.assertEqual(p.returncode,0,p.stderr)
if __name__=='__main__':unittest.main()
