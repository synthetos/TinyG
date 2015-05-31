/*
 * hardware.c - general hardware support functions
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

#ifdef __AVR
#include <avr/wdt.h>			// used for software reset
#endif

#include "tinyg.h"		// #1
#include "config.h"		// #2
#include "hardware.h"
#include "switch.h"
#include "controller.h"
#include "text_parser.h"
#ifdef __AVR
#include "xmega/xmega_init.h"
#include "xmega/xmega_rtc.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif

/*
 * _port_bindings  - bind XMEGA ports to hardware - these changed at board revision 7
 * hardware_init() - lowest level hardware init
 */

static void _port_bindings(float hw_version)
{
#ifdef __AVR
	hw.st_port[0] = &PORT_MOTOR_1;
	hw.st_port[1] = &PORT_MOTOR_2;
	hw.st_port[2] = &PORT_MOTOR_3;
	hw.st_port[3] = &PORT_MOTOR_4;

	hw.sw_port[0] = &PORT_SWITCH_X;
	hw.sw_port[1] = &PORT_SWITCH_Y;
	hw.sw_port[2] = &PORT_SWITCH_Z;
	hw.sw_port[3] = &PORT_SWITCH_A;

	if (hw_version > 6.9) {
		hw.out_port[0] = &PORT_OUT_V7_X;
		hw.out_port[1] = &PORT_OUT_V7_Y;
		hw.out_port[2] = &PORT_OUT_V7_Z;
		hw.out_port[3] = &PORT_OUT_V7_A;
		} else {
		hw.out_port[0] = &PORT_OUT_V6_X;
		hw.out_port[1] = &PORT_OUT_V6_Y;
		hw.out_port[2] = &PORT_OUT_V6_Z;
		hw.out_port[3] = &PORT_OUT_V6_A;
	}
#endif
}

void hardware_init()
{
#ifdef __AVR
	xmega_init();							// set system clock
	_port_bindings(TINYG_HARDWARE_VERSION);
	rtc_init();								// real time counter
#endif
}

/*
 * _get_id() - get a human readable signature
 *
 * FOR AVR:
 *	Produce a unique deviceID based on the factory calibration data.
 *		Format is: 123456-ABC
 *
 *	The number part is a direct readout of the 6 digit lot number
 *	The alpha is the low 5 bits of wafer number and XY coords in printable ASCII
 *	Refer to NVM_PROD_SIGNATURES_t in iox192a3.h for details.
 *
 * FOR ARM:
 *	Currently not implemented
 */

/* UNUSED
static uint8_t _read_calibration_byte(uint8_t index)
{
	NVM_CMD = NVM_NV_READ_CALIB_ROW_gc; 	// Load NVM Command register to read the calibration row
	uint8_t result = pgm_read_byte(index);
	NVM_CMD = NVM_NV_NO_OPERATION_gc; 	 	// Clean up NVM Command register
	return(result);
}
*/

enum {
	LOTNUM0=8,  // Lot Number Byte 0, ASCII
	LOTNUM1,    // Lot Number Byte 1, ASCII
	LOTNUM2,    // Lot Number Byte 2, ASCII
	LOTNUM3,    // Lot Number Byte 3, ASCII
	LOTNUM4,    // Lot Number Byte 4, ASCII
	LOTNUM5,    // Lot Number Byte 5, ASCII
	WAFNUM =16, // Wafer Number
	COORDX0=18, // Wafer Coordinate X Byte 0
	COORDX1,    // Wafer Coordinate X Byte 1
	COORDY0,    // Wafer Coordinate Y Byte 0
	COORDY1,    // Wafer Coordinate Y Byte 1
};

static void _get_id(char_t *id)
{
#ifdef __AVR
	char printable[33] = {"ABCDEFGHJKLMNPQRSTUVWXYZ23456789"};
	uint8_t i;

	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc; 	// Load NVM Command register to read the calibration row

	for (i=0; i<6; i++) {
		id[i] = pgm_read_byte(LOTNUM0 + i);
	}
	id[i++] = '-';
	id[i++] = printable[(pgm_read_byte(WAFNUM) & 0x1F)];
	id[i++] = printable[(pgm_read_byte(COORDX0) & 0x1F)];
//	id[i++] = printable[(pgm_read_byte(COORDX1) & 0x1F)];
	id[i++] = printable[(pgm_read_byte(COORDY0) & 0x1F)];
//	id[i++] = printable[(pgm_read_byte(COORDY1) & 0x1F)];
	id[i] = 0;

	NVM_CMD = NVM_CMD_NO_OPERATION_gc; 	 	// Clean up NVM Command register
#endif
}

/*
 * Hardware Reset Handlers
 *
 * hw_request_hard_reset()
 * hw_hard_reset()			- hard reset using watchdog timer
 * hw_hard_reset_handler()	- controller's rest handler
 */
void hw_request_hard_reset() { cs.hard_reset_requested = true; }

void hw_hard_reset(void)			// software hard reset using the watchdog timer
{
#ifdef __AVR
	wdt_enable(WDTO_15MS);
	while (true);					// loops for about 15ms then resets
#endif
}

stat_t hw_hard_reset_handler(void)
{
	if (cs.hard_reset_requested == false)
        return (STAT_NOOP);
	hw_hard_reset();				// hard reset - identical to hitting RESET button
	return (STAT_EAGAIN);
}

/*
 * Bootloader Handlers
 *
 * hw_request_bootloader()
 * hw_request_bootloader_handler() - executes a software reset using CCPWrite
 */

void hw_request_bootloader() { cs.bootloader_requested = true;}

stat_t hw_bootloader_handler(void)
{
#ifdef __AVR
	if (cs.bootloader_requested == false)
        return (STAT_NOOP);
	cli();
	CCPWrite(&RST.CTRL, RST_SWRST_bm);  // fire a software reset
#endif
	return (STAT_EAGAIN);				// never gets here but keeps the compiler happy
}

/***** END OF SYSTEM FUNCTIONS *****/


/***********************************************************************************
 * CONFIGURATION AND INTERFACE FUNCTIONS
 * Functions to get and set variables from the cfgArray table
 ***********************************************************************************/

/*
 * hw_get_id() - get device ID (signature)
 */

stat_t hw_get_id(nvObj_t *nv)
{
	char_t tmp[SYS_ID_LEN];
	_get_id(tmp);
	nv->valuetype = TYPE_STRING;
	ritorno(nv_copy_string(nv, tmp));
	return (STAT_OK);
}

/*
 * hw_run_boot() - invoke boot form the cfgArray
 */
stat_t hw_run_boot(nvObj_t *nv)
{
	hw_request_bootloader();
	return(STAT_OK);
}

/*
 * hw_set_hv() - set hardware version number
 */
stat_t hw_set_hv(nvObj_t *nv)
{
	if (nv->value > TINYG_HARDWARE_VERSION_MAX)
        return (STAT_INPUT_EXCEEDS_MAX_VALUE);
	set_flt(nv);					// record the hardware version
	_port_bindings(nv->value);		// reset port bindings
	switch_init();					// re-initialize the GPIO ports
//++++	gpio_init();				// re-initialize the GPIO ports
	return (STAT_OK);
}

/***********************************************************************************
 * TEXT MODE SUPPORT
 * Functions to print variables from the cfgArray table
 ***********************************************************************************/

#ifdef __TEXT_MODE

static const char fmt_fb[] PROGMEM = "[fb]  firmware build%18.2f\n";
static const char fmt_fv[] PROGMEM = "[fv]  firmware version%16.2f\n";
static const char fmt_hp[] PROGMEM = "[hp]  hardware platform%15.2f\n";
static const char fmt_hv[] PROGMEM = "[hv]  hardware version%16.2f\n";
static const char fmt_id[] PROGMEM = "[id]  TinyG ID%30s\n";

void hw_print_fb(nvObj_t *nv) { text_print_flt(nv, fmt_fb);}
void hw_print_fv(nvObj_t *nv) { text_print_flt(nv, fmt_fv);}
void hw_print_hp(nvObj_t *nv) { text_print_flt(nv, fmt_hp);}
void hw_print_hv(nvObj_t *nv) { text_print_flt(nv, fmt_hv);}
void hw_print_id(nvObj_t *nv) { text_print_str(nv, fmt_id);}

#endif //__TEXT_MODE

#ifdef __cplusplus
}
#endif
