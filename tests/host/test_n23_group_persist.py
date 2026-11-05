"""N23: failed group children must not be persisted or reported as accepted. TinyG wiki JSON-Details and TinyG-Configuration-for-Firmware-Version-0.97: grouped settings use the same setting validation and response status as single settings. Earlier successful children remain applied; groups are not transactions."""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
FW=Path(__file__).resolve().parents[2]/"firmware/tinyg"
PRE='\n#include <assert.h>\n#include <stdbool.h>\n#include <stdint.h>\n#include <string.h>\n#include <math.h>\n#define STAT_OK 0\n#define STAT_INPUT_LESS_THAN_MIN_VALUE 108\n#define STAT_INPUT_VALUE_RANGE_ERROR 110\n#define STAT_UNRECOGNIZED_NAME 100\n#define TYPE_FLOAT 4\n#define MODEL 0\n#define INCHES 0\n#define MILLIMETERS 1\n#define MM_PER_INCH 25.4f\n#define JERK_MULTIPLIER 1000000.f\n#define JERK_MATCH_PRECISION 1000.f\n#define EPSILON .00001f\n#define _to_millimeters(a) (units==INCHES?(a)*MM_PER_INCH:(a))\n#define PSTR(s) s\n#define min(a,b) ((a)<(b)?(a):(b))\n#define max(a,b) ((a)>(b)?(a):(b))\ntypedef int stat_t;\ntypedef struct {float value_flt;uint32_t value_int;int index,precision,valuetype,depth;} nvObj_t;\nstatic float stored=123;\nstatic int units=MILLIMETERS;\n#define table_target (&stored)\n#define table_precision 0\n#define GET_TABLE_WORD(x) table_##x\nstatic int cm_get_units_mode(int which){return units;}\nstatic int _get_axis_type(int index){return index>=3;}\nstatic int _get_axis(int index){return index;}\nstatic struct {struct {float jerk_max,recip_jerk;}a[6];float chordal_tolerance;}cm;\n'
class Regression(unittest.TestCase):
 def test_real_code(self):
  code=PRE
  code+=function((FW/'config.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) set_flt\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  code+=function((FW/'config_app.c').read_text(),r'stat_t set_flu\(')+"\n"
  code+=function((FW/'canonical_machine.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) cm_set_axis_jerk\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  code+=function((FW/'canonical_machine.c').read_text(encoding="latin1"), r"\b(?:static )?(?:stat_t|float|void|bool) cm_set_xjm\([^;{}]*\)[^;{}]*(?=\{)")+"\n"
  code+='\n#define STAT_INTERNAL_RANGE_ERROR 17\n#define F_PERSIST 32\n#define NV_MAX_OBJECTS 8\n#define TEXT_MODE 0\n#define TYPE_EMPTY 0\n#define TYPE_NULL 1\n#define ritorno(x) do {stat_t s=(x);if(s!=STAT_OK)return s;}while(0)\nstatic struct{int comm_mode;}cs={1};\nstatic nvObj_t nodes[4];\nstatic int writes,sets,gets,persist_error,get_error;\nstatic float disk;\nstatic bool nv_index_lt_groups(int i){return true;}\nstatic bool cfg_has_flag(int i,int f){return true;}\nstatic nvObj_t *nv_next(nvObj_t*n){return n==nodes+3?0:n+1;}\nstatic stat_t nv_get(nvObj_t*n){gets++;return get_error;}\nstatic stat_t write_persistent_value(nvObj_t*n){writes++;if(persist_error)return persist_error;disk=n->value_flt;return 0;}\nstatic stat_t nv_set(nvObj_t*n){sets++;return cm_set_xjm(n);}\n'
  code+=function((FW/'config.c').read_text(),r'stat_t nv_persist\(')
  code+='\nstatic bool _refused_in_cycle(nvObj_t *nv){(void)nv;return false;}  /* N60: these cases run idle */\nstatic int nv_index_max(void){return 255;}\n#define STAT_SETTING_REFUSED_IN_CYCLE 117\n'
  code+=function((FW/'config.c').read_text(),r'stat_t set_grp\(')
  code+='\nstatic void setup(void){memset(nodes,0,sizeof(nodes));nodes[1].valuetype=TYPE_FLOAT;nodes[1].value_flt=0;stored=disk=800;writes=sets=gets=persist_error=get_error=0;}\nint main(void){\n setup();assert(set_grp(nodes)==108);assert(stored==800&&disk==800&&writes==0);\n setup();nodes[1].value_flt=900;nodes[2].valuetype=TYPE_FLOAT;nodes[2].value_flt=0;\n assert(set_grp(nodes)==108);assert(stored==900&&disk==900&&writes==1&&sets==2);\n setup();nodes[1].value_flt=900;persist_error=17;\n assert(set_grp(nodes)==17&&stored==900&&disk==800&&writes==1);\n setup();nodes[1].valuetype=TYPE_NULL;get_error=100;\n assert(set_grp(nodes)==100&&gets==1&&writes==0);\n setup();nodes[1].value_flt=900;assert(set_grp(nodes)==0&&stored==900&&disk==900&&writes==1);\n}\n'
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/"check.c";p.write_text(code);exe=Path(d)/"check"
   subprocess.run(["cc","-std=gnu99","-O1","-fsanitize=address,undefined,float-cast-overflow",str(p),"-lm","-o",str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
