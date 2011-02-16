/*
 * tg_controller.h - tinyg controller and top level parsers
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with TinyG  
 * If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef controller_h
#define controller_h

/* Global structures and definitions */

enum tgControllerState {		// command execution state
	TG_READY_UNPROMPTED,		// ready for input, no prompt sent
	TG_READY_PROMPTED,			// ready for input, prompt has been sent
	TG_STATE_MAX
};

enum tgMode {
	TG_IDLE_MODE,				// idle mode only. No other modes active
	TG_CONFIG_MODE,				// read and set configurations
	TG_GCODE_MODE,				// gcode interpreter
	TG_DIRECT_DRIVE_MODE,		// direct drive motors
	TG_MAX_MODE
};

#define TG_FLAG_PROMPTS_bm (1<<0)// prompt enabled if set
#define CHAR_BUFFER_SIZE 80		// common text buffer size (255 max)

struct tgController {					// main controller struct
	uint8_t state;						// controller state (tgControllerState)
	uint8_t flags;						// control flags
	uint8_t status;						// return status (controller level)
	uint8_t mode;						// current operating mode (tgMode)
	uint8_t src;						// active source device
	uint8_t default_src;				// default source device
	uint8_t i;							// temp for indexes
	char buf[CHAR_BUFFER_SIZE];			// text buffer
};

/*
 * Global Scope Functions
 */

void tg_init(void);
void tg_alive(void); 
void tg_controller(void);
void tg_reset_source(void);
uint8_t tg_application_startup(void);
void tg_print_status(const uint8_t status_code, const char *textbuf);

//int tg_parser(char * buf);
//int tg_read_next_line(void);
//void tg_reset_source();

#endif
