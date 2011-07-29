/*
 * tg_controller.h - tinyg controller and top level parsers
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef controller_h
#define controller_h

#include <stdio.h>				// needed for FILE def'n

/* Global structures and definitions */

enum tgMode {
	TG_GCODE_MODE,				// gcode interpreter
	TG_DIRECT_DRIVE_MODE,		// direct drive motors
	TG_TEST_MODE,				// run tests
	TG_MAX_MODE
};

#define TG_FLAG_PROMPTS_bm (1<<0)// prompt enabled if set
#define CHAR_BUFFER_SIZE 80		// common text buffer size (255 max)

struct controllerSingleton {	// main TG controller struct
	uint8_t status;				// return status (controller level)
	uint8_t prompt_disabled;	// TRUE = disables prompts
	uint8_t prompted;			// TRUE = prompted
	uint8_t xoff_enabled;		// TRUE = enable XON/XOFF flow control
	uint8_t xoff_active;		// TRUE = in XOFF mode right now
	uint8_t mode;				// current operating mode (tgMode)
	uint8_t src;				// active source device
	uint8_t default_src;		// default source device
	double position[4];			// buffer for current position from gcode.c
	char buf[CHAR_BUFFER_SIZE];	// text buffer
};
struct controllerSingleton tg;			// controller state structure

void tg_init(uint8_t default_src);
void tg_alive(void); 
void tg_controller(void);
void tg_application_startup(void);
void tg_reset_source(void);
void tg_print_status(const uint8_t status_code, const char *textbuf);

#endif
