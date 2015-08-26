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

typedef enum {				        // how spindle controls are presented by the Gcode parser
    SPINDLE_CONTROL_OFF = 0,        // M5
    SPINDLE_CONTROL_CW,             // M3
    SPINDLE_CONTROL_CCW             // M4
} cmSpindleControl;

typedef enum {
    SPINDLE_OFF = 0,
    SPINDLE_ON,
    SPINDLE_PAUSE                   // meaning it was on and now it's off
} cmSpindleEnable;

typedef enum {				        // spindle direction state
    SPINDLE_CW = 0,
    SPINDLE_CCW
} cmSpindleDir;

typedef enum {
    SPINDLE_ACTIVE_LOW = 0,
    SPINDLE_ACTIVE_HIGH
} cmSpindlePolarity;

typedef enum {
    ESC_ONLINE = 0,
    ESC_OFFLINE,
    ESC_LOCKOUT,
    ESC_REBOOTING,
    ESC_LOCKOUT_AND_REBOOTING,
} cmESCState;

#define SPINDLE_OVERRIDE_ENABLE     false
#define SPINDLE_OVERRIDE_FACTOR     1.00
#define SPINDLE_OVERRIDE_MIN        0.05 // 5%
#define SPINDLE_OVERRIDE_MAX        2.00 // 200%

/*
 * Spindle control structure
 */

typedef struct cmSpindleSingleton {

    float speed;                        // S in RPM
    cmSpindleEnable enable;             // OFF, ON, PAUSE
    cmSpindleDir direction;             // CW, CCW

    bool pause_on_hold;                 // pause on feedhold
    cmSpindlePolarity enable_polarity;  // 0=active low, 1=active high
    cmSpindlePolarity dir_polarity;     // 0=clockwise low, 1=clockwise high
    float dwell_seconds;                // dwell on spindle resume

    bool sso_enable;                    // TRUE = spindle speed override enabled (see also m48_enable in canonical machine)
    float sso_factor;                   // 1.0000 x S spindle speed. Go up or down from there

    cmESCState esc_state;               // state management for ESC controller
    uint32_t esc_boot_timer;            // When the ESC last booted up
    uint32_t esc_lockout_timer;         // When the ESC lockout last triggered

} cmSpindleton_t;
extern cmSpindleton_t spindle;

/*
 * Global Scope Functions
 */

void spindle_init();
void spindle_reset();
stat_t cm_set_spindle_speed(float speed);			    // S parameter
stat_t cm_spindle_control(uint8_t control);             // M3, M4, M5 integrated spindle control
void cm_spindle_off_immediate(void);
void cm_spindle_optional_pause(bool option);            // stop spindle based on system options selected
void cm_spindle_resume(float dwell_seconds);            // restart spindle after pause based on previous state

//stat_t cm_spindle_override_enable(uint8_t flag);    // M51
//stat_t cm_spindle_override_factor(uint8_t flag);    // M51.1

stat_t cm_set_dir(nvObj_t *nv);
stat_t cm_set_sso(nvObj_t *nv);

/*--- text_mode support functions ---*/

#ifdef __TEXT_MODE

    void cm_print_spep(nvObj_t *nv);
    void cm_print_spdp(nvObj_t *nv);
    void cm_print_spph(nvObj_t *nv);
    void cm_print_spdw(nvObj_t *nv);
	void cm_print_ssoe(nvObj_t *nv);
	void cm_print_sso(nvObj_t *nv);
    void cm_print_spe(nvObj_t *nv);
    void cm_print_spd(nvObj_t *nv);
    void cm_print_sps(nvObj_t *nv);

#else

    #define cm_print_spep tx_print_stub
    #define cm_print_spdp tx_print_stub
    #define cm_print_spph tx_print_stub
    #define cm_print_spdw tx_print_stub
    #define cm_print_ssoe tx_print_stub
    #define cm_print_spe tx_print_stub
    #define cm_print_sso tx_print_stub
    #define cm_print_spd tx_print_stub
    #define cm_print_sps tx_print_stub

#endif // __TEXT_MODE

#endif	// End of include guard: SPINDLE_H_ONCE
