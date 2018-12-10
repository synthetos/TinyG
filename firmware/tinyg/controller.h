/*
 * controller.h - tinyg controller and main dispatch loop
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2016 Robert Giseburt
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
#ifndef CONTROLLER_H_ONCE
#define CONTROLLER_H_ONCE

#define SAVED_BUFFER_LEN 128            // saved buffer size (for reporting only)
#define OUTPUT_BUFFER_LEN 512           // buffer for serialized JSON output & text output
// see also: error.h STATUS_MESSAGE_LEN and config.h NV_ lengths

#define LED_NORMAL_TIMER 1000			// blink rate for normal operation (in ms)
#define LED_ALARM_TIMER 100				// blink rate for alarm state (in ms)

typedef enum {				            // manages startup lines
    CONTROLLER_INITIALIZING = 0,		// controller is initializing - not ready for use
    CONTROLLER_NOT_CONNECTED,			// controller has not yet detected connection to USB (or other comm channel)
    CONTROLLER_CONNECTED,				// controller has connected to USB (or other comm channel)
    CONTROLLER_STARTUP,					// controller is running startup messages and lines
    CONTROLLER_READY					// controller is active and ready for use
} cmControllerState;

typedef struct controllerSingleton {	// main TG controller struct
	magic_t magic_start;				// magic number to test memory integrity
	uint32_t null;						// dumping ground for items with no target

	// system identification values
	float fw_build;						// tinyg firmware build number
	float fw_version;					// tinyg firmware version number
	uint8_t hw_platform;                // tinyg hardware compatibility - platform type
	uint8_t hw_version;                 // tinyg hardware compatibility - platform revision

	// communications state variables
	uint8_t primary_src;				// primary input source device
	uint8_t secondary_src;				// secondary input source device
	uint8_t default_src;				// default source device

    uint8_t comm_mode;					// 0=text mode, 1=JSON mode, 2=JSON in txt override

	// system state variables
	cmControllerState controller_state;
	uint8_t led_state;		// LEGACY	// 0=off, 1=on
	int32_t led_counter;	// LEGACY	// a convenience for flashing an LED
	uint32_t led_timer;					// used by idlers to flash indicator LED
	uint8_t limit_switch_asserted;      // non-zero input number indicates limit condition
	bool hard_reset_requested;		    // flag to perform a hard reset
	bool bootloader_requested;		    // flag to enter the bootloader

	int32_t job_id[4];					// uuid to identify the job

	// controller serial buffers
	char *bufp;                         // pointer to primary or secondary input buffer
	uint16_t linelen;					// length of currently processing line
	char out_buf[OUTPUT_BUFFER_LEN];	// output buffer for serialized JSON and text output
	char saved_buf[SAVED_BUFFER_LEN];	// buffer for saving the input buffer (reporting only)

	magic_t magic_end;
} controller_t;

extern controller_t cs;					// controller state structure

/**** function prototypes ****/

void controller_init(uint8_t std_in, uint8_t std_out, uint8_t std_err);
void controller_init_assertions(void);
stat_t controller_test_assertions(void);

void controller_run(void);
void controller_dispatch_txt_container (nvObj_t *nv, char *str);
void controller_reset_source(void);
void controller_set_primary_source(uint8_t dev);
void controller_set_secondary_source(uint8_t dev);
void controller_assert_limit_condition(uint8_t input);
void controller_request_enquiry(void);

#endif // End of include guard: CONTROLLER_H_ONCE
