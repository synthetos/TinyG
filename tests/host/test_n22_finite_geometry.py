"""N22: coordinate offsets and resolved targets must leave finite geometry. TinyG wiki Coordinate-Systems and NIST RS274NGC v3 3.5.1/3.5.2/3.5.5 define offsets and target positions. A 1e18 canonical-unit bound leaves headroom for squaring six differences of two accepted endpoints; reject rather than publish a NaN plan."""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
FW=Path(__file__).resolve().parents[2]/"firmware/tinyg"
PRE='\n#include <assert.h>\n#include <stdbool.h>\n#include <stdint.h>\n#include <string.h>\n#include <math.h>\n#ifndef MAX_GEOMETRY_COORDINATE\n#define MAX_GEOMETRY_COORDINATE 1e18f\n#endif\n#define STAT_OK 0\n#define STAT_INPUT_LESS_THAN_MIN_VALUE 108\n#define STAT_INPUT_VALUE_RANGE_ERROR 110\n#define STAT_UNRECOGNIZED_NAME 100\n#define TYPE_FLOAT 4\n#define MODEL 0\n#define INCHES 0\n#define MILLIMETERS 1\n#define MM_PER_INCH 25.4f\n#define JERK_MULTIPLIER 1000000.f\n#define JERK_MATCH_PRECISION 1000.f\n#define EPSILON .00001f\n#define _to_millimeters(a) (units==INCHES?(a)*MM_PER_INCH:(a))\n#define PSTR(s) s\n#define min(a,b) ((a)<(b)?(a):(b))\n#define max(a,b) ((a)>(b)?(a):(b))\ntypedef int stat_t;\ntypedef struct {float value_flt;uint32_t value_int;int index,precision,valuetype,depth;} nvObj_t;\nstatic float stored=123;\nstatic int units=MILLIMETERS;\n#define table_target (&stored)\n#define table_precision 0\n#define GET_TABLE_WORD(x) table_##x\nstatic int cm_get_units_mode(int which){return units;}\nstatic int _get_axis_type(int index){return index>=3;}\nstatic int _get_axis(int index){return index;}\nstatic struct {struct {float jerk_max,recip_jerk;}a[6];float chordal_tolerance;}cm;\n'
class Regression(unittest.TestCase):
 def test_real_code(self):
  code=PRE
  code+=function((FW/'config.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) set_flt\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  code+=function((FW/'config_app.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) set_flu\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  import re
  s=(FW/'canonical_machine.c').read_text()
  row=next(x for x in (FW/'config_app.c').read_text().splitlines() if '"g54x"' in x and '{' in x)
  setter=row.split(',')[6].strip()
  code+='\n#define F_CONVERT 128\nstatic bool cfg_has_flag(int i,int f){return i<3;}\n'
  if setter!='set_flu':code+=function(s,rf'stat_t {setter}\(nvObj_t \*nv\)')
  code+='\n#define offset_set '+setter+'\n'
  code+='\nint main(void){for(units=0;units<2;units++){\n float bad[]={1e30f,-1e30f,NAN,INFINITY};\n for(int k=0;k<4;k++){stored=123;nvObj_t n={.value_flt=bad[k]};assert(offset_set(&n)==110&&stored==123);}\n nvObj_t n={.value_flt=1000};assert(offset_set(&n)==0&&isfinite(stored));\n}}\n'
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/"check.c";p.write_text(code);exe=Path(d)/"check"
   subprocess.run(["cc","-std=gnu99","-O1","-fsanitize=address,undefined,float-cast-overflow","-fstrict-flex-arrays=3",str(p),"-lm","-o",str(exe)],check=True)
   subprocess.run([str(exe)],check=True)

class PlannerGeometry(unittest.TestCase):
 def test_refuse_before_buffer_or_position_change(self):
  code='\n#include <assert.h>\n#include <math.h>\n#ifndef MAX_GEOMETRY_COORDINATE\n#define MAX_GEOMETRY_COORDINATE 1e18f\n#endif\n#include <stdio.h>\n#include <avr/interrupt.h>\n#include "tinyg.h"\n#include "config.h"\n#include "controller.h"\n#include "canonical_machine.h"\n#include "planner.h"\n#include "plan_arc.h"\n#include "kinematics.h"\n#include "stepper.h"\n#include "encoder.h"\n#include "report.h"\n#include "util.h"\n#include "xmega/xmega_rtc.h"\n\ncmSingleton_t cm;\nstPrepSingleton_t st_pre;\nrtClock_t rtc;\ncontroller_t cs;\nuint8_t SREG;\nchar text_item[256];\nint spoil_plan;\nvoid spoil(mpBuf_t *p){if(spoil_plan && p->gm.target[0]==20){p->tail_length=NAN;}}\n\nuint8_t cm_get_path_control(const GCodeState_t *gcode_state) { return (PATH_CONTINUOUS); }\nvoid cm_set_motion_state(const cmMotionState motion_state) { cm.motion_state = motion_state; }\nvoid cm_cycle_end(void) {}\nvoid cm_abort_arc(void) {} void cm_run_short_move(void) {}\nstat_t cm_is_alarmed(void) { return cm.machine_state == MACHINE_ALARM; }\nstat_t cm_panic_P(const stat_t status, const char *msg_P) { return (status); }\nvoid en_set_encoder_steps(uint8_t motor, float steps) {}\nvoid kn_inverse_kinematics(const float travel[], float steps[]) {}\nstat_t rpt_exception(stat_t status, const char *msg) { return (status); }\nstat_t rpt_exception_P(stat_t status, const char *msg) { return (status); }\nvoid qr_request_queue_report(int8_t buffers) {}\nuint8_t st_runtime_isbusy(void) { return true; }       /* the exec is busy: blocks stay queued */\nvoid st_request_exec_move(void) {}\nvoid st_prep_null(void) {}\nvoid st_prep_command(void *bf) {}\nvoid st_prep_dwell(float microseconds) {}\nfloat st_get_move_time(const float axis_length[]) { return 0; }\nvoid stepper_reset(void) {}\nstat_t mp_exec_aline(mpBuf_t *bf) { return STAT_OK; }\nint holds;void cm_request_feedhold(void) {holds++;}\nstat_t mp_exec_move(void) { return STAT_NOOP; }\n\n\nint main(void){\n planner_init();SREG=0x95;cm.junction_acceleration=1;st_pre.buffer_state=PREP_BUFFER_OWNED_BY_LOADER;\n for(int a=0;a<AXES;a++){cm.a[a].velocity_max=cm.a[a].feedrate_max=800;cm.a[a].jerk_max=20;cm.a[a].recip_jerk=1.f/20000000;cm.a[a].junction_dev=.05;}\n GCodeState_t gm={0};gm.feed_rate_mode=UNITS_PER_MINUTE_MODE;gm.feed_rate=100;gm.target[0]=10;gm.motion_mode=MOTION_MODE_STRAIGHT_FEED;\n gm.target[0]=1e30f;\n assert(mp_aline(&gm)==STAT_INPUT_VALUE_RANGE_ERROR);\n assert(mb.buffers_available==PLANNER_BUFFER_POOL_SIZE&&mm.position[0]==0);\n gm.target[0]=INFINITY;assert(mp_aline(&gm)==STAT_INPUT_VALUE_RANGE_ERROR);\n gm.target[0]=10;assert(mp_aline(&gm)==STAT_OK);\n mpBuf_t *first=mb.r;uint8_t before=mb.buffers_available;\n spoil_plan=1;gm.target[0]=20;assert(mp_aline(&gm)==STAT_INPUT_VALUE_RANGE_ERROR);\n assert(mb.buffers_available==before && mm.position[0]==10 && holds==1);\n assert(first->exit_velocity==0 && isfinite(first->tail_length));\n assert(mb.w==mb.q && mb.w->buffer_state==MP_BUFFER_EMPTY);\n spoil_plan=0;assert(mp_aline(&gm)==STAT_OK);assert(mm.position[0]==20);\n}\n'
  root=FW.parent.parent.parent;fixture=Path(__file__).parent/'planner_startup'
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'plan.c';p.write_text(code);exe=Path(d)/'plan'
   line=Path(d)/'plan_line.c'
   source=(FW/'plan_line.c').read_text().replace('mp_calculate_trapezoid(&plan);','mp_calculate_trapezoid(&plan); extern void spoil(mpBuf_t *); spoil(&plan);')
   line.write_text(source)
   subprocess.run(['cc','-std=gnu99','-O1','-funsigned-char','-D_POSIX_C_SOURCE=200809L','-DHAS_STARTUP','-include','tgmath.h','-I'+str(fixture),'-I'+str(FW),str(p),*[str(line if x=='plan_line.c' else FW/x) for x in ['planner.c','plan_line.c','plan_zoid.c','util.c']],'-lm','-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)

class ResolvedTargets(unittest.TestCase):
 def test_target_refusal_restores_previous_endpoint(self):
  source=(FW/'canonical_machine.c').read_text()
  code=r'''
#include <math.h>
#include <assert.h>
#include <string.h>
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
cmSingleton_t cm;
#define copy_vector(a,b) memcpy(a,b,sizeof(float)*AXES)
#define _to_millimeters(x) (x)
static float offset;
float cm_get_active_coord_offset(uint8_t axis){return axis==0 ? offset : 0;}
'''
  code+=function(source,r'static float _calc_ABC\(')
  code+='\n'+''.join(function((FW/'util.h').read_text(),r'static inline '+sig+r'\(') for sig in ['uint32_t fp_bits','bool fp_bounded','bool fp_bounded_at'])
  code+=function(source,r'(?:void|stat_t) cm_set_model_target\(')
  body='return cm_set_model_target(t,f);' if 'stat_t cm_set_model_target(' in source else 'cm_set_model_target(t,f);return STAT_OK;'
  code+='\nstat_t convert(float*t,bool*f){'+body+'}\n'
  code+=r'''
int main(void){
 float t[AXES]={1e30f};bool f[AXES]={true};cm.a[0].axis_mode=AXIS_STANDARD;
 cm.gmx.position[0]=2;cm.gm.distance_mode=INCREMENTAL_MODE;
 assert(convert(t,f)==STAT_INPUT_VALUE_RANGE_ERROR&&cm.gm.target[0]==2);
 t[0]=5;assert(convert(t,f)==STAT_OK&&cm.gm.target[0]==7);
 cm.gm.distance_mode=ABSOLUTE_MODE;offset=1e30f;
 assert(convert(t,f)==STAT_INPUT_VALUE_RANGE_ERROR&&cm.gm.target[0]==2);
 offset=10;assert(convert(t,f)==STAT_OK&&cm.gm.target[0]==15);
}
'''
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'target.c';p.write_text(code);exe=p.with_suffix('')
   subprocess.run(['cc','-std=gnu99','-D_POSIX_C_SOURCE=200809L','-I'+str(Path(__file__).parent/'planner_startup'),'-I'+str(FW),str(p),'-lm','-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
