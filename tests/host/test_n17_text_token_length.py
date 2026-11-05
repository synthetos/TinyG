"""N17: a text setting/action name must match in full, before execution.

TinyG wiki Text-Mode names commands by their exact token, not a prefix.
Use real text_parser and nv_get_index, including the five-character action
clear; short keys such as xvm must retain their existing exact matching.
"""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
from test_text_numeric_validation import PRELUDE
ROOT=Path(__file__).resolve().parents[2]/'firmware/tinyg'
class TextTokenLength(unittest.TestCase):
 def test_no_truncated_action_or_setting(self):
  cfg=(ROOT/'config.c').read_text();src=(ROOT/'text_parser.c').read_text()
  code=PRELUDE.replace('static struct {const char *group;} cfgArray[]={{"x"},{"1"}};',r'''
typedef int index_t;
#define GROUP_LEN 3
#define GET_TOKEN_BYTE(b) cfgArray[i].b
static int nv_index_max(void){return 4;}
static struct {const char *group;char token[6];} cfgArray[]={{"","clear"},{"x","xvm"},{"","shutd"},{"","abcde"}};
''')
  start=code.index('static int nv_get_index(');end=code.index('\n',start)
  code=code[:start]+function(cfg,r'index_t nv_get_index\(')+code[end:]
  code+=function(src,r'static stat_t _text_parser_kernal\(char \*str, nvObj_t \*nv\)\s*\n')
  code+=function(src,r'stat_t text_parser\(char \*str\)\s*\n')
  code+=r'''
static void refused(const char *input){char line[128];strcpy(line,input);gets=sets=persists=0;
 assert(text_parser(line)==STAT_UNRECOGNIZED_NAME);assert(!gets&&!sets&&!persists);}
int main(void){
 const char *bad[]={"$clearanything","$clearanything=1","$shutdextra","$abcdex=1","$xvmzz=1","$xvmzzz=1"};
 for(unsigned i=0;i<sizeof(bad)/sizeof(*bad);i++)refused(bad[i]);
 cm.machine_state=MACHINE_ALARM;refused("$clearanything");cm.machine_state=0;
 char valid[]="$clear";assert(text_parser(valid)==STAT_OK);assert(gets==1);
 assert(nv_get_index("","abcde")==3);assert(nv_get_index("","abcdex")==NO_MATCH);
 assert(nv_get_index("abc","dex")==NO_MATCH);assert(nv_get_index("toolong","x")==NO_MATCH);
 assert(nv_get_index("x","vm")==1);
 char set[]="$xvm=600";gets=sets=persists=0;assert(text_parser(set)==0);assert(sets==1&&persists==1);
}
'''
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'text.c';p.write_text(code);exe=p.with_suffix('')
   subprocess.run(['cc','-std=c99','-Wall','-Werror','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(p),'-o',str(exe)],check=True)
   q=subprocess.run([str(exe)],capture_output=True,text=True);self.assertEqual(q.returncode,0,q.stderr)
