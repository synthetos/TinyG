/*
 * tg_controller.h - tinyg controller and main dispatch loop
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
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
#ifndef controller_h
#define controller_h

#include <stdio.h>						// needed for FILE def'n

//#define TG_FLAG_PROMPTS_bm (1<<0)		// prompt enabled if set
#define INPUT_BUFFER_LEN 255			// text buffer size (255 max)
#define SAVED_BUFFER_LEN 100			// saved buffer size (for reporting only)
#define OUTPUT_BUFFER_LEN 512			// text buffer size
#define STATUS_MESSAGE_LEN 32			// status message string storage allocation
#define APPLICATION_MESSAGE_LEN 64		// application message string storage allocation

struct controllerSingleton {			// main TG controller struct
	uint16_t magic_start;				// magic number to test memory integity	
	float null;							// dumping ground for items with no target
	float fw_build;						// tinyg firmware build number
	float fw_version;					// tinyg firmware version number
	float hw_version;					// tinyg hardware compatibility
	uint8_t test;
	uint8_t primary_src;				// primary input source device
	uint8_t secondary_src;				// secondary input source device
	uint8_t default_src;				// default source device
	uint8_t network_mode;				// 0=master, 1=repeater, 2=slave
	uint8_t linelen;					// length of currently processing line
	uint8_t led_state;					// 0=off, 1=on
	int32_t led_counter;				// a convenience for flashing an LED
	uint8_t reset_requested;			// flag to perform a software reset
	uint8_t bootloader_requested;		// flag to enter the bootloader
	char *bufp;							// pointer to primary or secondary in buffer
	char in_buf[INPUT_BUFFER_LEN];		// primary input buffer
	char out_buf[OUTPUT_BUFFER_LEN];	// output buffer
	char saved_buf[SAVED_BUFFER_LEN];	// save the input buffer
	uint16_t magic_end;
};
struct controllerSingleton tg;			// controller state structure

void tg_init(uint8_t std_in, uint8_t std_out, uint8_t std_err);
void tg_request_reset(void);
void tg_request_bootloader(void);
void tg_reset(void);
void tg_controller(void);
void tg_application_startup(void);
void tg_reset_source(void);
void tg_set_primary_source(uint8_t dev);
void tg_set_secondary_source(uint8_t dev);
void tg_text_response(const uint8_t status, const char *buf);

#endif
