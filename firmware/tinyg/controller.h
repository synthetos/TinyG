/*
 * controller.h - tinyg controller and main dispatch loop
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2015 Robert Giseburt
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

#define INPUT_BUFFER_LEN 255			// text buffer size (255 max)
#define SAVED_BUFFER_LEN 100			// saved buffer size (for reporting only)
#define OUTPUT_BUFFER_LEN 512			// text buffer size
// see also: tinyg.h MESSAGE_LEN and config.h NV_ lengths

#define LED_NORMAL_TIMER 1000			// blink rate for normal operation (in ms)
#define LED_ALARM_TIMER 100				// blink rate for alarm state (in ms)

#define LED_NORMAL_BLINK_RATE 3000      // blink rate for normal operation (in ms)
#define LED_ALARM_BLINK_RATE 750        // blink rate for alarm state (in ms)
#define LED_SHUTDOWN_BLINK_RATE 300     // blink rate for shutdown state (in ms)
#define LED_PANIC_BLINK_RATE 100        // blink rate for panic state (in ms)

typedef enum {                          // manages startup lines
    CONTROLLER_INITIALIZING = 0,        // controller is initializing - not ready for use
    CONTROLLER_NOT_CONNECTED,           // has not yet detected connection to USB (or other comm channel)
    CONTROLLER_CONNECTED,               // has connected to USB (or other comm channel)
    CONTROLLER_STARTUP,                 // is running startup messages and lines
    CONTROLLER_READY,                   // is active and ready for use
    CONTROLLER_PAUSED                   // is paused - presumably in preparation for queue flush
} csControllerState;

typedef struct controllerSingleton {	// main TG controller struct
	magic_t magic_start;				// magic number to test memory integrity
	float null;							// dumping ground for items with no target

	float fw_build;						// tinyg firmware build number
	float fw_version;					// tinyg firmware version number
	float hw_platform;					// tinyg hardware compatibility - platform type
	float hw_version;					// tinyg hardware compatibility - platform revision

	uint32_t led_timer;                 // used to flash indicator LED
	uint32_t led_blink_rate;            // used to flash indicator LED

	// communications state variables
	uint8_t comm_mode;                  // TG_TEXT_MODE or TG_JSON_MODE
#ifdef __ARM
	uint8_t state_usb0;
	uint8_t state_usb1;
//	bool shared_buf_overrun;            // flag for shared string buffer overrun condition
#endif

#ifdef __AVR
	uint8_t primary_src;                // primary input source device
	uint8_t secondary_src;              // secondary input source device
	uint8_t default_src;                // default source device

	uint8_t usb_baud_flag;              // running a USB baudrate update sequence
	uint16_t linelen;					// length of currently processing line
	uint16_t read_index;				// length of line being read

	uint8_t led_state;		            // used by AVR IndicatorLed_toggle() in gpio.c
#endif

	// system state variables
	int32_t led_counter;	// LEGACY	// a convenience for flashing an LED
	uint8_t hard_reset_requested;       // flag to perform a hard reset
	uint8_t bootloader_requested;       // flag to enter the bootloader
	bool shared_buf_overrun;            // flag for shared string buffer overrun condition

	uint8_t state;						// controller state
	csControllerState controller_state;

//	uint8_t sync_to_time_state;
//	uint32_t sync_to_time_time;

	// controller serial buffers
	char *bufp;						// pointer to primary or secondary in buffer
	char in_buf[INPUT_BUFFER_LEN];	// primary input buffer
	char out_buf[OUTPUT_BUFFER_LEN];	// output buffer
	char saved_buf[SAVED_BUFFER_LEN];	// save the input buffer
	magic_t magic_end;
} controller_t;

extern controller_t cs;					// controller state structure

/**** function prototypes ****/

void controller_init(uint8_t std_in, uint8_t std_out, uint8_t std_err);
void controller_init_assertions(void);
stat_t controller_test_assertions(void);
void controller_run(void);
//void controller_reset(void);

void tg_reset_source(void);
void tg_set_primary_source(uint8_t dev);
void tg_set_secondary_source(uint8_t dev);

#endif // End of include guard: CONTROLLER_H_ONCE
