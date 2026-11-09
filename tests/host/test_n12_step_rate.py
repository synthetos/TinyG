"""N12: Cartesian planner limits motor pulse rate, and prep refuses an impossible segment.

TinyG Configuration wiki (0.97), motor step angle/microsteps/travel per revolution:
steps per unit derive from all three. stepper.h defines a 50 kHz DDA, at most
one pulse per motor per tick. Use 90% for planning, reserving pulse capacity for
rounding and the bounded following-error correction; retain requested F.
"""
from pathlib import Path
import re,subprocess,tempfile,unittest
import test_feed_limit_tags as tags
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]
FREQ=re.search(r'^#define FREQUENCY_DDA.*$',(ROOT/'firmware/tinyg/hardware.h').read_text(),re.M).group()+'\n'
class StepRate(unittest.TestCase):
 def test_real_planner_limits_mapped_motors(self):
  code=tags.CHECK[:tags.CHECK.index('int main(void) {')].replace('float st_get_move_time(const float axis_length[]) { return 0; }','')
  if 'stConfig_t st_cfg;' not in code:code+='\nstConfig_t st_cfg;\n'
  code+='\n'+FREQ
  stepper=(ROOT/'firmware/tinyg/stepper.c').read_text()
  if 'float st_get_move_time(' in stepper:
   code+='static float motor_time_per_unit[AXES];static uint8_t motor_time_axes[AXES],motor_time_groups;\n'+function(stepper,r'void st_refresh_move_time\(')+function(stepper,r'float st_get_move_time\(')
  code+=function(stepper,r'static int8_t _get_motor\(')+function(stepper,r'static void _set_motor_steps_per_unit\(')
  code+='\nstatic uint8_t *set_target;\nstat_t set_ui8(nvObj_t *nv){*set_target=nv->value_int;return STAT_OK;}\nstatic int _get_axis_type(int index){return 0;}\n'
  code+=function((ROOT/'firmware/tinyg/config_app.c').read_text(),r'static stat_t set_ma\(nvObj_t \*nv\)(?=\s*\{)')
  code+=function((ROOT/'firmware/tinyg/canonical_machine.c').read_text(),r'stat_t cm_set_am\(')
  code+=r'''
static void init_rate(void){setup();memset(&st_cfg,0,sizeof st_cfg);for(int m=0;m<MOTORS;m++)st_cfg.mot[m].motor_map=AXES;}
int main(void){
 init_rate();st_cfg.mot[0].motor_map=0;
 st_cfg.mot[0].step_angle=1.8f;st_cfg.mot[0].travel_rev=.01f;st_cfg.mot[0].microsteps=8;
 nvObj_t nv={0};strcpy(nv.token,"1sa");_set_motor_steps_per_unit(&nv);
 float distances[AXES]={1,2,0,0,0,0};float old=st_get_move_time(distances);
 assert(old>0);
 set_target=&st_cfg.mot[0].motor_map;nv.value_int=1;assert(set_ma(&nv)==STAT_OK);assert(fabs(st_get_move_time(distances)-2*old)<1e-5);
 set_target=&cm.a[1].axis_mode;nv.value_int=AXIS_INHIBITED;assert(cm_set_am(&nv)==STAT_OK);assert(st_get_move_time(distances)==0);
 nv.value_int=AXIS_STANDARD;assert(cm_set_am(&nv)==STAT_OK);assert(st_get_move_time(distances)>0);
 st_cfg.mot[0].travel_rev=.02f;_set_motor_steps_per_unit(&nv);assert(fabs(st_get_move_time(distances)-old)<1e-5);
 unsigned rng=1234567;
 for(int trial=0;trial<2000;trial++){
  init_rate();float d[AXES];
  for(int a=0;a<AXES;a++){rng=rng*1664525u+1013904223u;d[a]=(int)(rng%2001)-1000;cm.a[a].axis_mode=(rng&16)?AXIS_STANDARD:AXIS_INHIBITED;}
  float ref=0;
  for(int m=0;m<MOTORS;m++){
   rng=rng*1664525u+1013904223u;unsigned a=rng%7;
   st_cfg.mot[m].motor_map=a;st_cfg.mot[m].steps_per_unit=80u<<(rng%5);
   if(a<AXES && cm.a[a].axis_mode!=AXIS_INHIBITED){
    float t=fabsf(d[a])*st_cfg.mot[m].steps_per_unit*(1.f/(60.f*FREQUENCY_DDA*.9f));
    if(t>ref)ref=t;
   }
  }
  st_refresh_move_time();float actual=st_get_move_time(d);
  assert(fabsf(actual-ref)<=fmaxf(1e-7f,ref*4e-7f));
 }
 for(int rapid=0;rapid<2;rapid++)for(int negative=0;negative<2;negative++){
  init_rate();st_cfg.mot[0].motor_map=0;st_cfg.mot[0].steps_per_unit=160000;
  st_refresh_move_time();mpBuf_t*b=move(rapid?MOTION_MODE_STRAIGHT_TRAVERSE:MOTION_MODE_STRAIGHT_FEED,30,negative?-1:1,0);
  assert(b->cruise_vmax*160000/60 <= FREQUENCY_DDA*.90001f);
  assert(b->gm.feed_rate==30);assert(b->gm.move_time >= 160000/(FREQUENCY_DDA*.90001f*60));
 }
 init_rate();st_cfg.mot[2].motor_map=0;st_cfg.mot[2].steps_per_unit=160000;
 st_cfg.mot[3].motor_map=0;st_cfg.mot[3].steps_per_unit=320000;
 st_refresh_move_time();mpBuf_t*b=move(MOTION_MODE_STRAIGHT_FEED,30,1,1);
 assert(b->cruise_vmax*b->unit[0]*320000/60 <= FREQUENCY_DDA*.90001f);
 init_rate();st_cfg.mot[0].motor_map=0;st_cfg.mot[0].steps_per_unit=80;
 st_refresh_move_time();assert(fabs(move(MOTION_MODE_STRAIGHT_FEED,30,1,0)->cruise_vmax-30)<.001);
 init_rate();st_cfg.mot[0].motor_map=3;st_cfg.mot[0].steps_per_unit=160000;
 GCodeState_t gm={0};gm.motion_mode=MOTION_MODE_STRAIGHT_FEED;gm.feed_rate_mode=INVERSE_TIME_MODE;
 gm.feed_rate=.001f;gm.target[3]=1;mpBuf_t*w=mb.w;st_refresh_move_time();assert(mp_aline(&gm)==STAT_OK);
 assert(w->cruise_vmax*160000/60<=FREQUENCY_DDA*.90001f);
 init_rate();st_cfg.mot[0].motor_map=0;st_cfg.mot[0].steps_per_unit=160000;cm.a[0].axis_mode=AXIS_INHIBITED;
 st_refresh_move_time();assert(fabs(move(MOTION_MODE_STRAIGHT_FEED,30,1,0)->cruise_vmax-30)<.001);
}
'''
  tags.FeedLimitTags.build_run(self,code,None)
 def test_prep_rejects_over_capacity_before_publication(self):
  code=r'''
#include <assert.h>
#include "tinyg.h"
#include "canonical_machine.h"
#include "stepper.h"
#include "feedlimit.h"
#include "runtime.h"
rtSingleton_t rt;

#include "planner.h"
#include "util.h"
stPrepSingleton_t st_pre;stConfig_t st_cfg;volatile uint8_t fl_staged;

static uint16_t _f_to_period(float f){return 640;}
stat_t cm_panic_P(stat_t status,const char *msg){return status;}
'''
  code=FREQ+code
  util=(ROOT/'firmware/tinyg/util.c').read_text()
  code+=function(util,r'float min3\(')+function(util,r'float max3\(')
  code+=function((ROOT/'firmware/tinyg/stepper.c').read_text(),r'stat_t st_prep_line\(')
  code+=r'''
int main(void){
 float err[MOTORS]={0},steps[MOTORS]={0};
 for(int invalid=0;invalid<3;invalid++){
  memset(&st_pre,0,sizeof st_pre);st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;
  steps[0]=invalid==0?NAN:(invalid==1?INFINITY:-INFINITY);
  assert(st_prep_line(steps,err,250.0f/(60*FREQUENCY_DDA))!=STAT_OK);
  assert(st_pre.buffer_state==PREP_BUFFER_OWNED_BY_EXEC);
 }
 for(int boundary=0;boundary<3;boundary++){
  memset(&st_pre,0,sizeof st_pre);st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;
  steps[0]=boundary==0?250.f:(boundary==1?nextafterf(250.f,INFINITY):nextafterf(250.f,0));
  int status=st_prep_line(steps,err,250.0f/(60*FREQUENCY_DDA));
  assert((status==STAT_OK)==(boundary!=1));
 }
 for(int sign=-1;sign<=1;sign+=2){
  memset(&st_pre,0,sizeof st_pre);st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;
  steps[0]=400*sign;assert(st_prep_line(steps,err,250.0f/(60*FREQUENCY_DDA))!=STAT_OK);
  assert(st_pre.buffer_state==PREP_BUFFER_OWNED_BY_EXEC);
  memset(&st_pre,0,sizeof st_pre);st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;
  steps[0]=225*sign;assert(st_prep_line(steps,err,250.0f/(60*FREQUENCY_DDA))==STAT_OK);
  assert(st_pre.mot[0].substep_increment<=st_pre.dda_ticks_X_substeps);
 }
}
'''
  with tempfile.TemporaryDirectory() as d:
   c=Path(d)/'prep.c';c.write_text(code);exe=c.with_suffix('')
   p=subprocess.run(['cc','-std=gnu99','-D_POSIX_C_SOURCE=200809L','-include','tgmath.h','-I'+str(tags.FIXTURE),'-I'+str(tags.FIRMWARE),str(c),'-lm','-o',str(exe)],capture_output=True,text=True)
   self.assertEqual(p.returncode,0,p.stderr)
   p=subprocess.run([str(exe)],capture_output=True,text=True);self.assertEqual(p.returncode,0,p.stderr)
if __name__=='__main__':unittest.main()
