"""N20: axis rate caps must support a representable move duration. TinyG wiki TinyG-Configuration-for-Firmware-Version-0.97, xVM/xFR; NIST RS274NGC v3 3.5.1/3.5.2. Refuse rates below 0.00001 canonical units/minute (the existing numeric zero tolerance); refuse move durations beyond the segment counter range before queuing."""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
FW=Path(__file__).resolve().parents[2]/"firmware/tinyg"
PRE='\n#include <assert.h>\n#include <stdbool.h>\n#include <stdint.h>\n#include <string.h>\n#include <math.h>\n#define STAT_OK 0\n#define STAT_INPUT_LESS_THAN_MIN_VALUE 108\n#define STAT_INPUT_VALUE_RANGE_ERROR 110\n#define STAT_UNRECOGNIZED_NAME 100\n#define TYPE_FLOAT 4\n#define MODEL 0\n#define INCHES 0\n#define MILLIMETERS 1\n#define MM_PER_INCH 25.4f\n#define JERK_MULTIPLIER 1000000.f\n#define JERK_MATCH_PRECISION 1000.f\n#define EPSILON .00001f\n#define _to_millimeters(a) (units==INCHES?(a)*MM_PER_INCH:(a))\n#define PSTR(s) s\n#define min(a,b) ((a)<(b)?(a):(b))\n#define max(a,b) ((a)>(b)?(a):(b))\ntypedef int stat_t;\ntypedef struct {float value_flt;uint32_t value_int;int index,precision,valuetype,depth;} nvObj_t;\nstatic float stored=123;\nstatic int units=MILLIMETERS;\n#define table_target (&stored)\n#define table_precision 0\n#define GET_TABLE_WORD(x) table_##x\nstatic int cm_get_units_mode(int which){return units;}\nstatic int _get_axis_type(int index){return index>=3;}\nstatic int _get_axis(int index){return index;}\nstatic struct {struct {float jerk_max,recip_jerk;}a[6];float chordal_tolerance;}cm;\n'
class Regression(unittest.TestCase):
 def test_real_code(self):
  code=PRE
  code+=function((FW/'config.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) set_flt\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  code+=function((FW/'config_app.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) set_flu\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  source=(FW/'canonical_machine.c').read_text()
  row=next(l for l in (FW/'config_app.c').read_text().splitlines() if '"xvm"' in l and '{' in l)
  if row.split(',')[6].strip()=='cm_set_axis_rate':
   code+=function((FW/'canonical_machine.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) cm_set_axis_rate\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  else:code+='\n#define cm_set_axis_rate set_flu\n'
  code+='\nint main(void){for(int a=0;a<6;a++)for(units=0;units<2;units++){\n stored=123;nvObj_t n={.index=a,.value_flt=1e-38f};assert(cm_set_axis_rate(&n)==108&&stored==123);\n n.value_flt=.00001f;assert(cm_set_axis_rate(&n)==0&&stored>0);\n}}\n'
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/"check.c";p.write_text(code);exe=Path(d)/"check"
   subprocess.run(["cc","-std=gnu99","-O1","-fsanitize=address,undefined,float-cast-overflow","-fstrict-flex-arrays=3",str(p),"-lm","-o",str(exe)],check=True)
   subprocess.run([str(exe)],check=True)

class PlannerDuration(unittest.TestCase):
 def test_refuse_before_buffer_or_position_change(self):
  code='\n#include <assert.h>\n#include <math.h>\n#include <stdio.h>\n#include <avr/interrupt.h>\n#include "tinyg.h"\n#include "config.h"\n#include "controller.h"\n#include "canonical_machine.h"\n#include "planner.h"\n#include "plan_arc.h"\n#include "kinematics.h"\n#include "stepper.h"\n#include "encoder.h"\n#include "report.h"\n#include "util.h"\n#include "xmega/xmega_rtc.h"\n\ncmSingleton_t cm;\nstPrepSingleton_t st_pre;\nrtClock_t rtc;\ncontroller_t cs;\nuint8_t SREG;\nchar text_item[256];\n\nuint8_t cm_get_path_control(const GCodeState_t *gcode_state) { return (PATH_CONTINUOUS); }\nvoid cm_set_motion_state(const cmMotionState motion_state) { cm.motion_state = motion_state; }\nvoid cm_cycle_end(void) {}\nvoid cm_abort_arc(void) {} void cm_run_short_move(void) {}\nstat_t cm_is_alarmed(void) { return cm.machine_state == MACHINE_ALARM; }\nstat_t cm_panic_P(const stat_t status, const char *msg_P) { return (status); }\nvoid en_set_encoder_steps(uint8_t motor, float steps) {}\nvoid kn_inverse_kinematics(const float travel[], float steps[]) {}\nstat_t rpt_exception(stat_t status, const char *msg) { return (status); }\nstat_t rpt_exception_P(stat_t status, const char *msg) { return (status); }\nvoid qr_request_queue_report(int8_t buffers) {}\nuint8_t st_runtime_isbusy(void) { return true; }       /* the exec is busy: blocks stay queued */\nvoid st_request_exec_move(void) {}\nvoid st_prep_null(void) {}\nvoid st_prep_command(void *bf) {}\nvoid st_prep_dwell(float microseconds) {}\nfloat st_get_move_time(const float axis_length[]) { return 0; }\nvoid stepper_reset(void) {}\nstat_t mp_exec_aline(mpBuf_t *bf) { return STAT_OK; }\nvoid cm_request_feedhold(void) {}\nstat_t mp_exec_move(void) { return STAT_NOOP; }\n\n\nint main(void){\n planner_init();SREG=0x95;cm.junction_acceleration=1;st_pre.buffer_state=PREP_BUFFER_OWNED_BY_LOADER;\n for(int a=0;a<AXES;a++){cm.a[a].velocity_max=cm.a[a].feedrate_max=800;cm.a[a].jerk_max=20;cm.a[a].recip_jerk=1.f/20000000;cm.a[a].junction_dev=.05;}\n GCodeState_t gm={0};gm.feed_rate_mode=UNITS_PER_MINUTE_MODE;gm.feed_rate=100;gm.target[0]=10;gm.motion_mode=MOTION_MODE_STRAIGHT_FEED;\n cm.a[0].feedrate_max=1e-38f;\n assert(mp_aline(&gm)==STAT_INPUT_VALUE_RANGE_ERROR);\n assert(mb.buffers_available==PLANNER_BUFFER_POOL_SIZE&&mm.position[0]==0);\n cm.a[0].feedrate_max=.00001f;assert(mp_aline(&gm)==STAT_INPUT_VALUE_RANGE_ERROR);\n cm.a[0].feedrate_max=800;assert(mp_aline(&gm)==STAT_OK);\n}\n'
  root=FW.parent.parent.parent;fixture=Path(__file__).parent/'planner_startup'
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'plan.c';p.write_text(code);exe=Path(d)/'plan'
   subprocess.run(['cc','-std=gnu99','-O1','-funsigned-char','-D_POSIX_C_SOURCE=200809L','-DHAS_STARTUP','-include','tgmath.h','-I'+str(fixture),'-I'+str(FW),str(p),*[str(FW/x) for x in ['planner.c','plan_line.c','plan_zoid.c','util.c']],'-lm','-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
