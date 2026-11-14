"""Exercise live spindle requests, queued output ownership and stale-write cancellation."""
import re
import unittest
import test_spindle_coolant_status as fixture


class LiveSpindle(unittest.TestCase):
    build_run = fixture.SpindleCoolantStatus.build_run

    def test_live_output_and_cancellation(self):
        c = fixture.code()
        for name in ('spindle_override_callback', 'spindle_override_reset', 'spindle_override_master',
                     'spindle_override_program', '_get_factor', 'spindle_get_ssc', 'spindle_get_sso', 'spindle_set_sso',
                     'spindle_get_ssoe', 'spindle_set_ssoe', 'spindle_get_sse', 'spindle_get_scp'):
            c += fixture.fn('spindle.c', name) + '\n'
        c += r'''
static void loader_s_update(void) {}
static void command(void (*f)(float *,bool *),float value){f(&value,0);}
static void factor(float v){nvObj_t n={.valuetype=TYPE_FLOAT,.value_flt=v}; assert(spindle_set_sso(&n)==0);}
static void stop_interrupt(void){spindle_reset();}
static void speed_interrupt(void){float s=6000;_exec_spindle_speed(&s,0);}
static void permission_interrupt(void){spindle_override_program(false);}
int main(void){
 spindle_override_reset();
 factor(.5f); spindle_override_callback(); assert(spindle_out.speed==0 && spindle_out.mode==0);
 command(_exec_spindle_speed,8000);command(_exec_spindle_control,SPINDLE_CW);
 assert(spindle_out.speed==4000 && spindle_commanded==8000);
 nvObj_t report={0};spindle_get_ssc(&report);assert(report.value_flt==.5f);
 factor(2);spindle_override_callback();assert(spindle_out.speed==16000);
 for(int master=0;master<2;master++)for(int op=0;op<2;op++)for(int prog=0;prog<2;prog++){
  spindle_override_master(master);nvObj_t n={.valuetype=TYPE_INTEGER,.value_int=op};
  assert(spindle_set_ssoe(&n)==0);spindle_override_program(prog);spindle_override_callback();
  spindle_get_sse(&n);assert(n.value_int==(master&&op&&prog));
  assert(spindle_out.speed==((master&&op&&prog)?16000:8000));
 }
 spindle_override_reset();factor(2);command(_exec_spindle_speed,20000);
 assert(spindle_out.speed==24000 && spindle_capped && spindle_commanded==20000);
 factor(.5);spindle_override_callback();assert(spindle_out.speed==10000 && !spindle_capped);
 pwm.c[0].ccw_speed_lo=1000;pwm.c[0].ccw_speed_hi=10000;
 command(_exec_spindle_control,SPINDLE_CCW);assert(spindle_out.speed==10000);
 factor(.05);spindle_override_callback();assert(spindle_out.speed==1000);
 /* A future model S must not alter the speed in use. */
 cm.gm.spindle_speed=9000;factor(.25);spindle_override_callback();assert(spindle_out.speed==5000);
 /* Queued writers interrupt the main-loop calculation; never publish its stale result. */
 factor(.5);prepare_interrupt=stop_interrupt;spindle_override_callback();
 assert(spindle_out.mode==0 && spindle_out.speed==0);
 spindle_get_ssc(&report);assert(report.value_flt==1);
 spindle_override_callback();assert(spindle_out.mode==0 && spindle_out.speed==0);
 spindle_get_ssc(&report);assert(report.value_flt==1);
 command(_exec_spindle_control,SPINDLE_CW);factor(1.5);
 prepare_interrupt=speed_interrupt;spindle_override_callback();assert(spindle_out.speed==9000);
 spindle_override_callback();assert(spindle_out.speed==9000);
 factor(.5);prepare_interrupt=permission_interrupt;spindle_override_callback();assert(spindle_out.speed==6000);
 spindle_override_callback();assert(spindle_out.speed==6000);
 cm.alarm=1;spindle_reset();nvObj_t n={.valuetype=TYPE_FLOAT,.value_flt=2};
 assert(spindle_set_sso(&n)==204);spindle_override_callback();assert(spindle_out.speed==0);
 cm.alarm=0;spindle_override_reset();assert(spindle_factor==1000 && spindle_enables==7);
 for(unsigned i=0;i<3;i++){float bad[]={.049f,2.001f,NAN};n.valuetype=TYPE_FLOAT;n.value_flt=bad[i];
  assert(spindle_set_sso(&n)==110 && spindle_factor==1000);}
 rt.check_only=true;factor(.5);assert(spindle_factor==1000);rt.check_only=false;
 for(int mode=1;mode<=2;mode++){
  float applied;bool capped;
  pwm.c[0].cw_speed_lo=pwm.c[0].cw_speed_hi=1000;
  pwm.c[0].ccw_speed_lo=pwm.c[0].ccw_speed_hi=1000;
  assert(_spindle_duty(mode,500,1000,&applied,&capped)==1 && applied==1000 && capped);
 }
 return 0;
}
'''
        self.build_run(c, 'live_spindle')

    def test_ram_only_rows_and_reset_wiring(self):
        table = fixture.src('config_app.c')
        for token in ('sso', 'ssoe', 'sse', 'scp'):
            row = re.search(r'\{\s*"",\s*"'+token+r'",([^}\n]+)',table)[1]
            self.assertRegex(row, r'^_[fi]0,') # no F_INITIALIZE or F_PERSIST
        runtime = fixture.src('runtime.c')
        self.assertIn('spindle_override_reset();', fixture.fn('runtime.c','rt_init'))
        self.assertIn('spindle_override_reset();', fixture.fn('runtime.c','rt_override_reset_job'))
        cm = fixture.src('canonical_machine.c')
        self.assertEqual(cm.count('rt_override_reset_job();'),2) # kill and ordered M2/M30
