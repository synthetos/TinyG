/* Parser dependencies only: no device, storage, or motion is touched. */
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "controller.h"
#include "json_parser.h"
#define NUL 0
#define SPC 32
#define DEL 127
jsSingleton_t js;
nvList_t nvl;
nvStr_t nvStr;
stat_t status_code;
cmSingleton_t cm;
static unsigned executed, get_count, sets;
static int last_status;
static nvObj_t observed;
static const char *names[] = {"fb", "xvm", "1mi", "1", "ee", "gc", "txt", "tid", "sr", "js"};
static const valueType types[] = {TYPE_FLOAT, TYPE_FLOAT, TYPE_INTEGER, TYPE_NULL, TYPE_BOOLEAN,
    TYPE_STRING, TYPE_TXTCON, TYPE_INTEGER, TYPE_NULL, TYPE_INTEGER};
index_t nv_get_index(const char *g, const char *t) {
    char full[GROUP_LEN+TOKEN_LEN+1]; snprintf(full,sizeof(full),"%s%s",g,t);
    for (unsigned i=0;i<sizeof(names)/sizeof(*names);i++)
        if (!strcmp(full,names[i]) || !strcmp(t,names[i])) return i;
    return NO_MATCH;
}
valueType cfg_is_type(index_t i) { assert(i<sizeof(types)/sizeof(*types));return types[i]; }
bool nv_group_is_prefixed(char *t) { return !strcmp(t,"1"); }
bool nv_group_is_typesafe(char *t) { return strcmp(t,"sr")!=0; }
/* The first real configuration row is fb, whose setter is set_nul. */
static void observe(nvObj_t *n) { observed=*n;executed++; }
static stat_t capture_get(nvObj_t *n) { observe(n);get_count++;return STAT_OK; }
static stat_t capture_set(nvObj_t *n) { observe(n);sets++;return STAT_OK; }
static fptrCmd table_get(index_t i) { (void)i;return capture_get; }
static fptrCmd table_set(index_t i) { return i ? capture_set : set_nul; }
static int table_precision(index_t i) { (void)i;return 3; }
#undef GET_TABLE_WORD
#define GET_TABLE_WORD(a) table_##a(nv->index)
index_t nv_index_max(void) { return sizeof(names)/sizeof(*names); }
bool nv_index_is_single(index_t i) { return i<nv_index_max(); }
static nvObj_t *real_reset_nv_list(char *parent);
nvObj_t *nv_reset_nv_list(char *parent) {
    /* AVR address zero reads as the empty parent; desktop NULL is not readable. */
    return real_reset_nv_list(parent ? parent : "");
}
stat_t nv_persist(nvObj_t *n) { (void)n;return STAT_OK; }
stat_t controller_dispatch_txt_container(nvObj_t *n,char *s) { (void)n;(void)s;return STAT_OK; }
void json_print_response(uint8_t status) { last_status=status; }
stat_t sr_request_status_report(uint8_t request) { (void)request;return STAT_OK; }
#define SR_REQUEST_ASAP 1
static void reset(void) {
    nvl.magic_start=0x1357;nvl.magic_end=0x2468;executed=get_count=sets=0;last_status=-1;memset(&observed,0,sizeof(observed));memset(&js,0,sizeof(js));
}
