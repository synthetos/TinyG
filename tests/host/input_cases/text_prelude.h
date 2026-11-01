/* Execute the real text parser; configuration I/O is captured, never performed. */
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <errno.h>
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#define NUL 0
#define strcpy_P strcpy
#define memcpy_P memcpy
#define TEXT_RESPONSE 0
#define JSON_RESPONSE 0
cmSingleton_t cm;
stat_t status_code;
const cfgItem_t cfgArray[] = {{.group="x"}, {.group="1"}, {.group=""}};
static nvObj_t obj;
static unsigned get_count, sets, persists;
nvObj_t *nv_reset_nv_list(char *parent) { (void)parent;memset(&obj,0,sizeof(obj));return &obj; }
void sr_run_text_status_report(void) {}
void help_general(nvObj_t *nv) { (void)nv; }
stat_t nv_copy_string(nvObj_t *nv,const char *s) { (void)nv;(void)s;return STAT_OK; }
index_t nv_get_index(const char *g,const char *s) {
    (void)g;return !strcmp(s,"xvm")?0:!strcmp(s,"1mi")?1:!strcmp(s,"sys")?2:NO_MATCH;
}
valueType cfg_is_type(index_t i) { return i==0?TYPE_FLOAT:TYPE_INTEGER; }
stat_t nv_get(nvObj_t *nv) { (void)nv;get_count++;return STAT_OK; }
stat_t nv_set(nvObj_t *nv) { (void)nv;sets++;return STAT_OK; }
stat_t nv_persist(nvObj_t *nv) { (void)nv;persists++;return STAT_OK; }
void nv_print_list(stat_t s,uint8_t a,uint8_t b) { (void)s;(void)a;(void)b; }
