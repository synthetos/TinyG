"""Private override plans preserve distance, outer speeds, and the supplied jerk cap."""
from pathlib import Path
import subprocess
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[2]

class PrivatePlan(unittest.TestCase):
    def test_transition_limits_and_rejected_plans_leave_output_untouched(self):
        code=r'''
#include <assert.h>
#include <math.h>
#include <string.h>
#include "override.h"
static void ramp(float a,float b,float duration,float jerk) {
    if(a==b){assert(duration==0);return;}
    assert(duration>0);
    /* Independent analytic derivative of v=a+(b-a)(10u^3-15u^4+6u^5). */
    for(int i=0;i<=1000;i++){
        double u=i/1000.0;
        double j=(b-a)*(60*u-180*u*u+120*u*u*u)/(duration*duration);
        assert(fabs(j)<=jerk*1.00001);
    }
}
int main(void){
    ovTransition_t p,old;
    for(unsigned factor=50;factor<=2000;factor+=50){
        assert(ov_plan_transition(100,600,600,0,1200,1e9f,factor,&p));
        assert(p.entry==600 && p.exit==0 && p.factor_q1000==factor);
        assert(fabsf(p.cruise-600*factor/1000.0f)<.001f);
        assert(fabsf(p.head+p.body+p.tail-100)<.00002f);
        assert(p.body>=0 && p.head>=0 && p.tail>=0);
        ramp(p.entry,p.cruise,p.head_time,1e9f);ramp(p.cruise,p.exit,p.tail_time,1e9f);
    }
    assert(ov_plan_transition(100,600,600,0,800,1e9f,2000,&p));
    assert(p.capped && p.cruise==800 && p.factor_q1000==2000);
    old=p;
    assert(!ov_plan_transition(.00001f,600,600,0,1200,1e9f,500,&p));
    assert(!memcmp(&p,&old,sizeof p));
    assert(!ov_plan_transition(NAN,600,600,0,1200,1e9f,500,&p));
    assert(!ov_plan_transition(100,600,600,0,1200,0,500,&p));
    assert(!ov_plan_transition(100,600,600,0,1200,1e9f,49,&p));
    assert(!ov_plan_transition(100,600,600,0,1200,1e9f,2001,&p));
    assert(!ov_plan_transition(100,600,600,0,500,1e9f,500,&p));
    assert(!memcmp(&p,&old,sizeof p));
    assert(ov_plan_transition(100,600,600,600,1200,1e9f,1000,&p));
    assert(p.head==0 && p.tail==0 && p.body==100);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c=Path(tmp)/'check.c';exe=Path(tmp)/'check';c.write_text(code)
            subprocess.run(['cc','-std=gnu99','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                            '-I'+str(ROOT/'firmware/tinyg'),str(c),str(ROOT/'firmware/tinyg/override.c'),'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
