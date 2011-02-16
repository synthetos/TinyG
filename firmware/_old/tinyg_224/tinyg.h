/*
 * tinyg.h - tinyg main header - Application GLOBALS 
 *			 (see also system.h and settings.h)
 * Part of TinyG project
 *
 * Copyright (c) 2010 Alden S. Hart, Jr.
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

#ifndef tinyg_h
#define tinyg_h

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#ifndef max
#define max(a, b) (((a)>(b))?(a):(b))
#endif
#ifndef min
#define min(a, b) (((a)<(b))?(a):(b))
#endif

// found in main.c
void tg_system_init(void);
void tg_application_init(void);
void tg_unit_tests(void);

// Operating Mode: (chose only one)
#define __NORMAL_MODE					// normal operation - receive from USB
//#define __RELAY_MODE					// receive from USB, relay to rs485
//#define __SLAVE_MODE					// receive from rs485


// Global Settings
//#define TINYG_VERSION "build 210 - \"aphasia\""
//#define TINYG_VERSION "build 213 - \"bezoar\""
//#define TINYG_VERSION "build 214 - \"chapped lips\""
//#define TINYG_VERSION "build 215 - \"dropsy\""
//#define TINYG_VERSION "build 216 - \"eczema\""
//#define TINYG_VERSION "build 217 - \"fainting spells\""
//#define TINYG_VERSION "build 220 - \"gout\""
//#define TINYG_VERSION "build 221 - \"hacking cough\""
//#define TINYG_VERSION "build 222 - \"turning japanese\""
#define TINYG_VERSION "build 223 - \"impetigo\""


// Constants
#define ONE_MINUTE_OF_MICROSECONDS 60000000.0	// from GRBL
#define TICKS_PER_MICROSECOND (F_CPU/1000000)	// from GRBL
#define MM_PER_INCH 25.4						// from GRBL
#define RADIAN 57.2957795

/* Define axes 
 *
 * Note that just defing "AXES" to be the number you want doesn't mean
 * you don;t have to change a lot of code. You do. Just less that would
 * other wise be required.
 */

#define AXES 4			// number of axes supported in this version

enum tgAxisNum {		// define axis numbers and array indexes from 0 to 3
		X,
		Y,
		Z,
		A,
		B,
		C
};

/* TinyG return codes
 * The following return codes are unified for various TinyG functions.
 * The first codes (up to the line) are aligned with the XIO codes.
 * Please don't change them without checking the corresponding values in xio.h
 * If you mess with this be sure to change the strings in tg_print_status
 */

//----- codes must align with xio.h and tg_print_status strings...
enum tgCodes {
	TG_OK = 0,					// function completed OK
	TG_ERR,						// generic error return (EPERM)
	TG_EAGAIN,					// function would block here (call again)
	TG_NOOP,					// function had no-operation
	TG_COMPLETE,				// operation is complete
	TG_EOL,						// function returned end-of-line
	TG_EOF,						// function returned end-of-file 
	TG_FILE_NOT_OPEN,
	TG_FILE_SIZE_EXCEEDED,
	TG_NO_SUCH_DEVICE,
	TG_BUFFER_EMPTY,
	TG_BUFFER_FULL_FATAL, 
	TG_BUFFER_FULL_NON_FATAL,
//----- ...to here
    TG_QUIT,					// function returned QUIT
	TG_UNRECOGNIZED_COMMAND,	// parser didn't recognize the command
	TG_EXPECTED_COMMAND_LETTER,	// malformed line to parser
	TG_UNSUPPORTED_STATEMENT,	// a different kind of malformed line
	TG_PARAMETER_OVER_RANGE,	// parameter is too large
	TG_BAD_NUMBER_FORMAT,		// number format error
	TG_FLOATING_POINT_ERROR,	// number conversion error
	TG_MOTION_CONTROL_ERROR,	// motion control failure
	TG_ARC_SPECIFICATION_ERROR,	// arc specification error
	TG_ZERO_LENGTH_MOVE,		// XYZA move is zero length
	TG_MAX_FEED_RATE_EXCEEDED,
	TG_MAX_SEEK_RATE_EXCEEDED,
	TG_MAX_TRAVEL_EXCEEDED,
	TG_MAX_SPINDLE_SPEED_EXCEEDED
};

/*
 * Common typedefs (see xio.h for some others)
 */										// pointers to functions:
typedef void (*fptr_void_uint8) (void); // returns void, unit8_t arg (poll_func)
typedef char (*fptr_char_void) (void); 	// returns char, void args
typedef int (*fptr_int_uint8)(uint8_t s);// returns int, unit8_t arg (signal handler) 
typedef int (*fptr_int_char_p) (char *b);// returns int, character pointer (line handler)

/*
 * Various debug and other compile-time switches
 */

#ifdef __SLAVE_MODE
#define DEFAULT_SOURCE XIO_DEV_RS485
#else
#define DEFAULT_SOURCE XIO_DEV_USB		// default source device
#endif

//#define __UNIT_TESTS		// uncomment to compile the unit tests into the code
//#define __DEBUG			// uncomment to enable debug logging
//#define __ECHO TRUE		// set to echo Gcode commands. If false, only prompts returned
//#define __SIMULATION_MODE	// settings to enable faster and more accurate simulation

#endif
