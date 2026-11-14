/*
 * spindle.h - spindle driver
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef SPINDLE_H_ONCE
#define SPINDLE_H_ONCE

/*
 * Global Scope Functions
 */

typedef struct spindleConfig {
    uint8_t enable_polarity;
    uint8_t direction_polarity;
} spindleConfig_t;
extern spindleConfig_t spindle_cfg;

void spindle_config_init(void);
void spindle_init_outputs(void);
stat_t spindle_set_polarity(nvObj_t *nv);
stat_t spindle_get_spe(nvObj_t *nv);
stat_t spindle_get_spd(nvObj_t *nv);
stat_t spindle_get_spc(nvObj_t *nv);
stat_t spindle_get_sps(nvObj_t *nv);
void spindle_get_runtime(uint8_t *mode, float *speed);
void spindle_control_immediate(uint8_t mode, float speed);
void spindle_override_reset(void);
void spindle_override_master(bool enabled);
void spindle_override_program(bool enabled);
stat_t spindle_override_callback(void);
stat_t spindle_get_sso(nvObj_t *nv);
stat_t spindle_get_ssc(nvObj_t *nv);
stat_t spindle_set_sso(nvObj_t *nv);
stat_t spindle_get_ssoe(nvObj_t *nv);
stat_t spindle_set_ssoe(nvObj_t *nv);
stat_t spindle_get_sse(nvObj_t *nv);
stat_t spindle_get_scp(nvObj_t *nv);
#ifdef __TEXT_MODE
void spindle_print_sso(nvObj_t *nv);
void spindle_print_ssoe(nvObj_t *nv);
void spindle_print_sse(nvObj_t *nv);
void spindle_print_scp(nvObj_t *nv);
#else
#define spindle_print_sso tx_print_stub
#define spindle_print_ssoe tx_print_stub
#define spindle_print_sse tx_print_stub
#define spindle_print_scp tx_print_stub
#endif
void spindle_init();
void spindle_reset();
stat_t cm_set_spindle_speed(float speed);			// S parameter
void cm_exec_spindle_speed(float speed);			// callback for above
stat_t cm_spindle_control(uint8_t spindle_mode);	// M3, M4, M5 integrated spindle control
void cm_exec_spindle_control(uint8_t spindle_mode);	// callback for above

#endif	// End of include guard: SPINDLE_H_ONCE
