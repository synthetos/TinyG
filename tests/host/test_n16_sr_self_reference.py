"""N16: status reports select scalar reads, never report/action getters.

TinyG wiki Status Reports describes a list of reportable values. Recursive
report/group commands are not values. Exercise selection, boot repair and
both real population paths; a synthetic getter tests the recursion backstop.
"""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]/'firmware/tinyg'
class StatusSelection(unittest.TestCase):
 def test_selection_load_and_recursion(self):
  r=(ROOT/'report.c').read_text();c=(ROOT/'config_app.c').read_text()
  ctl=(ROOT/'controller.c').read_text()
  self.assertLess(ctl.index('DISPATCH(_controller_state());'),ctl.index('sr_report_load_repair();'))
  code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define STAT_OK 0
#define STAT_INTERNAL_ERROR 1
#define STAT_INPUT_VALUE_RANGE_ERROR 110
#define STAT_VALUE_TYPE_ERROR 111
#define STAT_INPUT_EXCEEDS_MAX_LENGTH 112
#define STAT_BAD_STATUS_REPORT_SETTING 35
#define STAT_BUFFER_FULL_FATAL 12
#define NO_MATCH -1
#define NUL 0
#define NV_STATUS_REPORT_LEN 30
#define SR_WORKING_LIST_LEN 60
#define TOKEN_LEN 5
#define SR_OFF 0
#define SR_MATCH_PRECISION .001f
#define COMBINED_PROGRAM_STOP 3
#define COMBINED_PROGRAM_END 4
#define PSTR(s) s
#define strcpy_P strcpy
#define pgm_read_word(p) (*(p))
#define pgm_read_byte(p) (*(p))
#define TYPE_FLOAT 2
#define TYPE_INTEGER 3
#define TYPE_BOOLEAN 4
#define TYPE_SIGNED 5
#define TYPE_DATA 6
#define TYPE_NULL 7
#define TYPE_STRING 8
#define TYPE_EMPTY 0
#define TYPE_PARENT 9
typedef int stat_t;typedef int index_t;typedef int valueType;
typedef struct {int index,valuetype,precision;char token[6],group[4];char *str;union {uint32_t value_int;float value_flt;};} nvObj_t;
typedef stat_t(*fptrCmd)(nvObj_t*);
static nvObj_t nodes[38];
static struct {int status_report_list[30],stat_index,status_report_request;union {float value_flt[30];uint32_t value_int[30];};} sr;
static bool sr_populating, sr_load_repaired;
static const char SR_DEFAULTS[]="stat,posx";
static unsigned persisted,reads,depth,nested,exceptions;
static int read_fail=-1;
static uint32_t disk[30];
void sr_init_status_report_P(const char*);
static stat_t sr_get(nvObj_t*);static stat_t srs_get(nvObj_t*);
static stat_t _populate_unfiltered_status_report(char*);
static bool _populate_filtered_status_report(void);
static stat_t scalar(nvObj_t*n){n->value_flt=7;return 0;}
static stat_t qr_get(nvObj_t*n){return scalar(n);}
static stat_t get_rx(nvObj_t*n){return scalar(n);}
static stat_t help_config(nvObj_t*n){abort();}
static stat_t help_defa(nvObj_t*n){abort();}
static stat_t help_test(nvObj_t*n){abort();}
static stat_t help_boot(nvObj_t*n){abort();}
static stat_t st_set_md(nvObj_t*n){abort();}
static stat_t get_nul(nvObj_t*n){return 0;}
static stat_t rt_get_fs(nvObj_t*n){abort();}
static stat_t get_grp(nvObj_t*n){abort();}
static stat_t recurse(nvObj_t*n){nested++;assert(sr_get(n)==35);return scalar(n);}
enum {I_STAT,I_POS,I_SR,I_SRS,I_QR,I_RX,I_HELP,I_DEFA,I_TEST,I_BOOT,I_MD,I_RTFS,I_RECURSE,I_SE,I_GROUP};
static struct {char token[6],group[4];int type;fptrCmd get;} cfgArray[]={
 {"stat","",TYPE_INTEGER,scalar},{"posx","",TYPE_FLOAT,scalar},
 {"sr","",TYPE_BOOLEAN,sr_get},{"srs","",TYPE_NULL,srs_get},
 {"qr","",TYPE_NULL,qr_get},{"rx","",TYPE_NULL,get_rx},
 {"help","",TYPE_NULL,help_config},{"defa","",TYPE_BOOLEAN,help_defa},
 {"test","",TYPE_INTEGER,help_test},{"boot","",TYPE_BOOLEAN,help_boot},
 {"md","",TYPE_INTEGER,st_set_md},{"rtfs","rt",TYPE_INTEGER,rt_get_fs},
 {"loop","",TYPE_FLOAT,recurse},{"se00","",TYPE_SIGNED,scalar},
 {"sys","",TYPE_NULL,get_grp}};
static int nv_index_max(void){return sizeof(cfgArray)/sizeof(*cfgArray);}
static bool nv_index_is_single(index_t i){return i>=0&&i<I_GROUP;}
static int cfg_is_type(index_t i){assert(i>=0&&i<nv_index_max());return cfgArray[i].type;}
static bool cfg_get_is_action(fptrCmd p){return p==st_set_md;}
static int nv_get_index(const char*g,const char*t){for(int i=0;i<nv_index_max();i++)if(!strcmp(t,cfgArray[i].token))return i;return -1;}
static nvObj_t *nv_next(nvObj_t*n){return n&&n<nodes+37?n+1:NULL;}
static nvObj_t *nv_reset_nv_list(char*k){memset(nodes,0,sizeof nodes);return nodes;}
static void cm_position_snapshot_begin(void){assert(++depth<5);}
static void cm_position_snapshot_end(void){assert(depth>0);depth--;}
static void nv_populate_nv_by_index(nvObj_t*n,index_t i){n->index=i;n->valuetype=cfg_is_type(i);strcpy(n->token,cfgArray[i].token);n->group[0]=0;cfgArray[i].get(n);}
static stat_t cm_panic_P(int s,const char*m){abort();}
static stat_t nv_persist(nvObj_t*n){persisted++;disk[n->index-I_SE]=n->value_int;return 0;}
static stat_t nv_set(nvObj_t*n){sr.status_report_list[n->index-I_SE]=n->value_int;return 0;}
static stat_t set_u16(nvObj_t*n){sr.status_report_list[0]=n->value_int;return 0;}
static stat_t read_persistent_value(nvObj_t*n){n->value_int=disk[n->index-I_SE];reads++;return n->index-I_SE==read_fail?1:0;}
static stat_t rpt_exception(int s,const char*m){assert(s==35);exceptions++;return s;}
#define rpt_exception_P rpt_exception
'''
  def add(source,name):
   return function(source,r'(?:static )?(?:stat_t|void|bool) '+name+r'\([^;{}]*\)[^\n{]*(?=\s*\{)')+'\n'
  if 'bool cfg_is_reportable(' in c:code+=add(c,'cfg_is_reportable')
  for name in ['_valid_status_report_index','_persist_status_report_list','_set_sr','_set_srs','sr_init_status_report_P','sr_set_se','_populate_unfiltered_status_report','_populate_filtered_status_report','sr_get','srs_get']:
   body=add(r,name).replace('stat_t sr_get(', 'static stat_t sr_get(').replace('stat_t srs_get(','static stat_t srs_get(')
   code+=body
  code+=function(r,r'void __attribute__\(\(noinline\)\) sr_report_load_repair_pending\(void\)')
  code+=function((ROOT/'report.h').read_text(),r'static inline void sr_report_load_repair\(void\)')
  code+=r'''
static void baseline(void){for(int i=0;i<30;i++)sr.status_report_list[i]=-1;sr.status_report_list[0]=I_STAT;nv_reset_nv_list("");}
int main(void){
 int rejected[]={I_SR,I_SRS,I_QR,I_RX,I_HELP,I_DEFA,I_TEST,I_BOOT,I_MD,I_RTFS,I_GROUP};
 for(unsigned j=0;j<sizeof(rejected)/sizeof(*rejected);j++){int bad=rejected[j];
  baseline();nodes[1].index=I_POS;nodes[1].valuetype=TYPE_BOOLEAN;nodes[1].value_int=1;
  nodes[2].index=bad;nodes[2].valuetype=TYPE_BOOLEAN;nodes[2].value_int=1;
  unsigned before=persisted;assert(_set_sr(nodes)==110);assert(sr.status_report_list[0]==I_STAT&&sr.status_report_list[1]==-1);assert(persisted==before);
  baseline();nodes[1].token[0]='s';nodes[2].index=bad;nodes[2].valuetype=TYPE_BOOLEAN;nodes[2].value_int=1;
  assert(_set_srs(nodes)==110);assert(sr.status_report_list[0]==I_STAT);
  nvObj_t n={.value_int=bad};assert(sr_set_se(&n)==110);assert(sr.status_report_list[0]==I_STAT);
  sr.status_report_list[0]=bad;assert(_populate_unfiltered_status_report("sr")==35);assert(depth==0);
  assert(!_populate_filtered_status_report());assert(depth==0);
 }
 baseline();nodes[1].index=I_POS;nodes[1].valuetype=TYPE_BOOLEAN;nodes[1].value_int=1;
 assert(_set_sr(nodes)==0);assert(sr.status_report_list[0]==I_POS);
 assert(sr_get(nodes)==0);assert(depth==0);
 // Load a corrupt persisted list: keep valid entries on both sides, compact,
 // persist the repair, and emit one exception for all rejected entries.
 for(int i=0;i<30;i++)disk[i]=UINT32_MAX;
 disk[0]=I_STAT;disk[1]=I_SR;disk[2]=I_POS;disk[3]=I_SRS;disk[4]=9999;
 unsigned saved=persisted;sr_init_status_report_P("");
 assert(reads==30&&exceptions==0&&persisted==saved+30&&sr_load_repaired);
 sr_report_load_repair();assert(exceptions==1&&!sr_load_repaired);
 sr_report_load_repair();assert(exceptions==1);
 assert(sr.status_report_list[0]==I_STAT&&sr.status_report_list[1]==I_POS);
 for(int i=0;i<30;i++)assert(disk[i]==(uint32_t)sr.status_report_list[i]);
 for(int i=2;i<30;i++)assert(sr.status_report_list[i]==NO_MATCH);
 assert(sr_get(nodes)==0&&depth==0);
 // A second boot neither reports nor rewrites an already repaired list.
 saved=persisted;sr_init_status_report_P("");
 assert(exceptions==1&&persisted==saved&&reads==60);
 // An all-invalid list becomes empty, not a default list with new fields.
 for(int i=0;i<30;i++)disk[i]=I_SR;
 sr_init_status_report_P("");assert(exceptions==1);
 sr_report_load_repair();assert(exceptions==2);
 for(int i=0;i<30;i++)assert(sr.status_report_list[i]==NO_MATCH&&disk[i]==UINT32_MAX);
 assert(sr_get(nodes)==0&&depth==0);
 // Failed EEPROM reads are discarded even if the returned bits look valid.
 disk[0]=I_STAT;disk[1]=I_POS;read_fail=0;
 sr_init_status_report_P("");assert(exceptions==2&&sr.status_report_list[0]==I_POS);
 sr_report_load_repair();assert(exceptions==3);
 assert(sr.status_report_list[1]==NO_MATCH);read_fail=-1;
 // Unknown future scalar getter accidentally calls sr: no nested population.
 baseline();sr.status_report_list[0]=I_RECURSE;assert(sr_get(nodes)==0);assert(nested==1&&depth==0);
 (void)_populate_filtered_status_report();assert(nested==2&&depth==0);
 baseline();assert(sr_get(nodes)==0);assert(depth==0);
}
'''
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'check.c';p.write_text(code);exe=p.with_suffix('')
   subprocess.run(['cc','-std=c99','-Wall','-Wno-unused-function','-Wno-unused-variable','-fsanitize=address,undefined',str(p),'-o',str(exe)],check=True)
   q=subprocess.run([str(exe)],capture_output=True,text=True,timeout=5);self.assertEqual(q.returncode,0,q.stderr)
