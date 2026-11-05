
#include <assert.h>
#include <avr/interrupt.h>
#include <errno.h>
#include <ctype.h>
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "controller.h"
#include "json_parser.h"
#include "runtime.h"
#include "report.h"
#include "settings.h"
#include "planner.h"
#include "stepper.h"
#include "spindle.h"
#include "util.h"
#include "text_parser.h"
#include "help.h"
#define memcpy_P memcpy

#define NUL 0
#define SPC 32
#define DEL 127
jsSingleton_t js;
nvList_t nvl;
nvStr_t nvStr;
int json_status;
srSingleton_t sr;
extern unsigned source_line;
extern bool host_config_dispatch;
static stat_t get_query(nvObj_t *n) {
    printf("REPORT_STUB %u %s\n", source_line, cfgArray[n->index].token);
    return STAT_OK;
}
stat_t set_grp(nvObj_t *n){return rt_set_grp(n);}
#define ROW(tok,typ,get,set,target) {"",tok,typ,3,0,get,set,(uint32_t*)(target),0}
const cfgItem_t cfgArray[]={
 ROW("xjm",TYPE_FLOAT,get_flt,cm_set_xjm,&cm.a[0].jerk_max),
 ROW("ja",TYPE_FLOAT,get_flt,@JA_SET@,&cm.junction_acceleration),
 ROW("si",TYPE_INTEGER,get_query,sr_set_si,&sr.status_report_interval),
 ROW("xvm",TYPE_FLOAT,get_flt,@VM_SET@,&cm.a[0].velocity_max),
 ROW("rt",TYPE_NULL,get_query,rt_set_grp,0),
 ROW("rtfi",TYPE_INTEGER,rt_get_fi,rt_set_fi,0),
 ROW("rthb",TYPE_INTEGER,rt_get_hb,rt_set_hb,0),
 ROW("rtfs",TYPE_NULL,rt_get_fs,set_nul,0),
 ROW("rtup",TYPE_INTEGER,rt_get_up,set_nul,0),
 ROW("rtflk",TYPE_INTEGER,rt_get_flk,set_nul,0),ROW("rtflm",TYPE_INTEGER,rt_get_flm,set_nul,0),ROW("rtflp",TYPE_INTEGER,rt_get_flp,set_nul,0),ROW("rtflf",TYPE_FLOAT,rt_get_flf,set_nul,0),
 ROW("sps",TYPE_FLOAT,get_query,set_nul,0),ROW("spe",TYPE_INTEGER,get_query,set_nul,0),ROW("spc",TYPE_INTEGER,get_query,set_nul,0),ROW("cof",TYPE_INTEGER,get_query,set_nul,0),ROW("com",TYPE_INTEGER,get_query,set_nul,0),
 ROW("h",TYPE_NULL,get_query,get_query,0),ROW("help",TYPE_NULL,get_query,get_query,0),
 ROW("sr",TYPE_NULL,get_query,set_nul,0),ROW("qr",TYPE_NULL,get_query,set_nul,0),ROW("stat",TYPE_INTEGER,get_query,set_nul,0),ROW("hold",TYPE_INTEGER,get_query,set_nul,0),
};
index_t nv_index_max(void){return sizeof(cfgArray)/sizeof(*cfgArray);}
bool nv_index_is_single(index_t i){return i!=4;}
@FIRMWARE_TOKENS@
index_t nv_get_index(const char *g,const char *t){char full[30];snprintf(full,sizeof(full),"%s%s",g,t);for(index_t i=0;i<nv_index_max();i++)if(!strcmp(full,cfgArray[i].token))return i;for (unsigned i=0; i<sizeof(firmware_tokens)/sizeof(*firmware_tokens); i++) {
 if (host_config_dispatch && !strcmp(full,firmware_tokens[i])) {
  printf("UNMODELED_CONFIG %u %s\n",source_line,full);break;
 }
}
return NO_MATCH;}
valueType cfg_is_type(index_t i){return cfgArray[i].flags & F_TYPE_MASK;}
bool cfg_has_flag(index_t i,uint8_t flag){return (cfgArray[i].flags&flag)!=0;}
bool cfg_get_is_action(fptrCmd get){return false;}
bool nv_group_is_prefixed(char *t){return !strcmp(t,"rt");}
bool nv_group_is_typesafe(char *t){return strcmp(t,"sr")!=0;}
static nvObj_t *real_reset_nv_list(char *parent);
nvObj_t *nv_reset_nv_list(char *p){return real_reset_nv_list(p?p:"");}
stat_t controller_dispatch_txt_container(nvObj_t *n,char *s){return STAT_FUNCTION_IS_STUBBED;}
void json_print_response(uint8_t status){json_status=status;}
