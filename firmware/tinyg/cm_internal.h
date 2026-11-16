/* Private interfaces between canonical-machine implementation files.
 * Include tinyg.h and config.h first. Not part of the public machine API. */
#ifndef CM_INTERNAL_H
#define CM_INTERNAL_H

int8_t cm_cfg_axis_index(const index_t index);
int8_t cm_cfg_axis_type(const index_t index);
#ifdef __TEXT_MODE
extern const char *const msg_am[] PROGMEM;
#endif

#endif
