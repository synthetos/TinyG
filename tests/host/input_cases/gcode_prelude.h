// Host-only dependency stubs, adapted from test_modal_groups.py. Firmware functions are extracted at runtime.

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "controller.h"
#define NUL 0
cmSingleton_t cm;
controller_t cs;
stat_t status_code;
struct { uint8_t modals[MODAL_GROUP_COUNT]; } gp;
static int executions;
uint8_t cm_get_motion_mode(const GCodeState_t *state) { return state->motion_mode; }
stat_t cm_alarm(const stat_t status, const char *message) { return status; }
static stat_t _execute_gcode_block(void) { executions++; return STAT_OK; }
