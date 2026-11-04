"""N31: SET and timer programming share representable-period validation.

AVR's timer period is uint16_t. Exercise the original integer prescaler boundaries
and values either side, with float-cast-overflow sanitizer active.
"""
from pathlib import Path
import re,subprocess,tempfile,unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]

class PwmFrequency(unittest.TestCase):
    def test_set_and_register_guard(self):
        source=(ROOT/'firmware/tinyg/pwm.c').read_text()
        row=next(s for s in (ROOT/'firmware/tinyg/config_app.c').read_text().splitlines() if '"p1frq"' in s)
        setter=row.split(',')[6].strip()
        code=r'''
#include <assert.h>
#include <stdint.h>
#include <math.h>
#define __AVR
#define F_CPU 32000000UL
#define PWMS 2
#define STAT_OK 0
#define STAT_NO_SUCH_DEVICE 105
#define STAT_INPUT_LESS_THAN_MIN_VALUE 108
#define STAT_INPUT_EXCEEDS_MAX_VALUE 109
#define STAT_INPUT_VALUE_RANGE_ERROR 110
#define PWM_MAX_FREQ (F_CPU/256)
#define PWM_MIN_FREQ (F_CPU/64/65536)
typedef int stat_t;
typedef struct {float value_flt;} nvObj_t;
enum {TC_CLKSEL_DIV1_gc=1,TC_CLKSEL_DIV2_gc,TC_CLKSEL_DIV4_gc,TC_CLKSEL_DIV8_gc,TC_CLKSEL_DIV64_gc};
struct timer {uint16_t PER;uint8_t CTRLA;};
struct {struct {struct timer *timer;} p[2];} pwm;
static float stored;
static stat_t set_flt(nvObj_t*n){stored=n->value_flt;return 0;}
'''
        for name in ('_pwm_frequency','pwm_set_frq','pwm_set_freq'):
            sig=r'(?:static )?stat_t '+name+r'\([^;{}]*\)(?=\s*\{)'
            if re.search(sig,source):code+=function(source,sig)+'\n'
        code+=r'''
int main(void){
 struct timer a={40000,1},b={40000,1};pwm.p[0].timer=&a;pwm.p[1].timer=&b;
 float bad[]={0,-1,7,7.5f,61,122,244,488,125001,NAN,INFINITY};
 for(unsigned i=0;i<sizeof(bad)/sizeof(*bad);i++){
  nvObj_t nv={bad[i]};stored=800;
  assert(SETTER(&nv)!=0&&stored==800); /* applies equally when config_init loads EEPROM */
  assert(pwm_set_freq(0,bad[i])!=0&&a.PER==40000&&a.CTRLA==1);
 }
 float good[]={8,60,62,121,123,243,245,487,489,800,125000};
 for(unsigned i=0;i<sizeof(good)/sizeof(*good);i++){
  nvObj_t nv={good[i]};assert(SETTER(&nv)==0&&stored==good[i]);
  assert(pwm_set_freq(0,good[i])==0&&a.PER>=256);
  assert(pwm_set_freq(1,good[i])==0&&b.PER==a.PER);
 }
 assert(pwm_set_freq(PWMS,800)==STAT_NO_SUCH_DEVICE);
}
'''.replace('SETTER',setter)
        with tempfile.TemporaryDirectory() as tmp:
            c=Path(tmp)/'freq.c';c.write_text(code);exe=Path(tmp)/'freq'
            subprocess.run(['cc','-std=gnu99','-fsanitize=address,undefined,float-cast-overflow','-fno-sanitize-recover=all',str(c),'-lm','-o',str(exe)],check=True)
            r=subprocess.run([str(exe)],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stderr)
