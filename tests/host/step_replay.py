"""N39 investigation: extend corpus replay through real exec/prep and extracted DDA arithmetic.

Hardware registers and scheduling are host fixtures. --batch varies queue occupancy;
this is not a cycle-accurate AVR emulator or a substitute for Saleae verification.
Profile: XY/Z rates 1200, jerk 500, 1280 steps/mm; motors 1/2 both X, 3 Z, 4 A.
An active segment finishes before the newly prepared segment loads. GPIO levels,
actual interrupt timing and transport pacing are not modeled.
SUM reports requested/corrected logical steps; STEPS reports runtime target,
signed virtual pulses, total pulses, and accumulated feedback correction.
"""
from pathlib import Path
import argparse,json,subprocess
import corpus_replay as base
from source_extract import function

def build(root,out):
 base.build(root,out)
 fw=root/'firmware/tinyg';step=(fw/'stepper.c').read_text()
 driver=(out/'driver.c').read_text()
 driver=driver.replace('void mp_exit_hold_state(void){}','')
 driver=driver.replace('void en_set_encoder_steps(uint8_t m,float x){}','')
 driver=driver.replace('void kn_inverse_kinematics(const float travel[],float steps[]){for(int m=0;m<MOTORS;m++)steps[m]=0;}','')
 driver=driver.replace('stat_t mp_exec_aline(mpBuf_t *b){return STAT_OK;}','')
 driver=driver.replace('void st_prep_dwell(float t){}','void host_run_active(void);\nvoid st_prep_dwell(float t){host_run_active();}')
 driver=driver.replace('void st_prep_command(void *b){mp_runtime_command((mpBuf_t*)b);}', 'void host_run_active(void);\nvoid st_prep_command(void *b){host_run_active();mp_runtime_command((mpBuf_t*)b);}')
 start=driver.index('        memcpy(&mr.gm, &b->gm');end=driver.index('    } else if (b->block_type',start)
 driver=driver[:start]+'''        unsigned guard=0;
        while(mb.r==b && b->buffer_state!=MP_BUFFER_EMPTY){
            st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;
            stat_t status=mp_exec_move();
            if(status!=STAT_OK && status!=STAT_EAGAIN && status!=STAT_NOOP && status!=STAT_MINIMUM_TIME_MOVE){fprintf(stderr,"EXEC %u\\n",status);exit(126);}
            if(++guard>1000000)abort();
        }
'''+driver[end:]
 driver=driver.replace('a<3?16000:3600','a<3?1200:3600').replace('st_cfg.mot[m].motor_map=m;','st_cfg.mot[m].motor_map=(m==1?0:m);')
 driver=driver.replace('static void room(void)','static unsigned batch=28;\nstatic void room(void)').replace('mb.buffers_available < PLANNER_BUFFER_HEADROOM','mb.buffers_available < PLANNER_BUFFER_POOL_SIZE-batch+1')
 driver=driver.replace('    setup();','    if(argc>1)batch=atoi(argv[1]);\n    setup();')
 driver=driver.replace('    free(line);','    host_run_active();\n    extern void host_report(void);host_report();\n    free(line);')
 driver=driver.replace('void stepper_reset(void){}','void stepper_reset(void){}\nvoid st_prep_null(void){}')
 (out/'driver.c').write_text(driver)
 code='''#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "planner.h"
#include "stepper.h"
#include "encoder.h"
#include "feedlimit.h"
#include "util.h"
#define _f_to_period(f) (uint16_t)((float)F_CPU/(float)(f))
#define STEP_BIT_bm 1
#define DIRECTION_BIT_bm 2
#define MOTOR_ENABLE_BIT_bm 4
stRunSingleton_t st_run;
enEncoders_t en;
static struct {uint8_t OUT;} host_ports[MOTORS];
static int64_t pulses[MOTORS],total[MOTORS];
static double requested[MOTORS],corrected[MOTORS];
'''
 code+=base.extract(fw,'encoder.c','en_set_encoder_steps')+function((fw/'encoder.c').read_text(),r'float en_read_encoder\([^;{}]*\)(?=\s*\{)')
 dda=step[step.index('if ((st_run.mot[MOTOR_1].substep_accumulator +='):step.index('\n\tif ((st_run.mot[MOTOR_2].substep_accumulator +=')]
 dda=dda.replace('MOTOR_1','motor').replace('PORT_motor_VPORT','host_ports[motor]')
 code+='void host_run_active(void){while(st_run.dda_ticks_downcount){for(int motor=0;motor<MOTORS;motor++){\n'+dda.replace('INCREMENT_ENCODER(motor);','INCREMENT_ENCODER(motor);pulses[motor]+=en.en[motor].step_sign;total[motor]++;')+'}\n st_run.dda_ticks_downcount--;}}\n'
 load=step[step.index('if ((st_run.mot[MOTOR_1].substep_increment ='):step.index('\n#if (MOTORS >= 2)',step.index('if ((st_run.mot[MOTOR_1].substep_increment ='))]
 load=load.replace('MOTOR_1','motor').replace('PORT_motor_VPORT','host_ports[motor]')
 prep=function(step,r'stat_t st_prep_line\([^;{}]*\)(?=\s*\{)').replace('st_prep_line(', 'real_st_prep_line(',1)
 code+=prep+'\nstat_t st_prep_line(float t[],float e[],float dt){for(int m=0;m<MOTORS;m++)requested[m]+=t[m];stat_t status=real_st_prep_line(t,e,dt);for(int m=0;m<MOTORS;m++)corrected[m]+=t[m];if(status)return status;host_run_active();st_run.dda_ticks_downcount=st_pre.dda_ticks;st_run.dda_ticks_X_substeps=st_pre.dda_ticks_X_substeps;for(int motor=0;motor<MOTORS;motor++){\n'+load+'}\nst_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;return STAT_OK;}\n'
 code+='void host_report(void){for(int m=0;m<MOTORS;m++){printf("SUM %d %.9g %.9g\\n",m,requested[m],corrected[m]);printf("STEPS %d %.9g %lld %lld %.9g\\n",m,(double)mr.target_steps[m],(long long)pulses[m],(long long)total[m],(double)st_pre.mot[m].corrected_steps);}}\n'
 (out/'step_engine.c').write_text(base.binary32(code))
 for name in ['plan_exec','kinematics']:
  (out/(name+'.c')).write_text(base.binary32((fw/(name+'.c')).read_text()))
 args=json.loads((out/'command.json').read_text());args[args.index('-lm'):args.index('-lm')]=[str(out/(n+'.c')) for n in ['plan_exec','kinematics','step_engine']];args.insert(1,'-DF_CPU=32000000UL')
 p=subprocess.run(args,capture_output=True,text=True);(out/'step_build.log').write_text(p.stdout+p.stderr)
 if p.returncode:raise RuntimeError(p.stderr[-4000:])
 return out/'replay'
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,default=Path(__file__).resolve().parents[2]);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();print(build(a.source,a.output))
