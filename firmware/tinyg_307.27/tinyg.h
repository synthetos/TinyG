/*
 * tinyg.h - tinyg main header - Application GLOBALS 
 *			 (see also system.h and settings.h)
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

#ifndef tinyg_h
#define tinyg_h

#include <stdio.h>		// for uint8_t definition

/* global variables and prototypes */

void tg_system_init(void);
void tg_application_init(void);
void tg_application_startup(void);
void tg_trap(char *msg);

char trap_msg[32];

/* operating variables */

// Operating Mode: (chose only one)
#define __STANDALONE_MODE		// normal operation - receive from USB
//#define __MASTER_MODE			// receive from USB, relay to rs485
//#define __SLAVE_MODE			// receive from rs485

#ifdef __SLAVE_MODE
#define STD_INPUT XIO_DEV_RS485
#define STD_ERROR XIO_DEV_USB
#else 
#define STD_INPUT XIO_DEV_USB
#define STD_ERROR XIO_DEV_USB
#endif

// OPERATING SETTINGS
//#define __SIMULATION_MODE		// enables faster simulation
#define __UNFORGIVING			// fails hard versus introduce errors
//#define __TRAPS				// enables trap generation
//#define __DEBUG				// enables debug statements (see below)
//#define __NO_EEPROM				// disables EEPROM initialization
//#define __UNIT_TESTS			// includes unit tests in the build

#ifdef __DEBUG					// DEBUG SETTINGS
#define __dbECHO_INPUT_LINE		// echos input lines 				(controller.c:268)
#define __dbECHO_GCODE_BLOCK	// echos input to Gcode interpreter	(gcode.c)
#define __dbALINE_CALLED		// shows call to mp_aline() 		(planner.c)
#define __dbSHOW_QUEUED_LINE	// shows line being queued 			(motor_queue.c)
#define __dbSHOW_LIMIT_SWITCH	// shows switch closures 			(limit_switches.c)
//#define __dbSHOW_LOAD_MOVE	// shows move being loaded 
#endif

// some convenient macros for generating program traps
// USAGE	TRAP(m->length, PSTR("Line length %f"))
// USAGE: 	TRAP_GT(m->length, 100, PSTR("Length: %f"))
#ifdef __TRAPS
#define TRAP(a,m) { sprintf_P(trap_msg,m,a); tg_trap(trap_msg);}
#define TRAP_GT(a,b,m)	{if (a>b) { sprintf_P(trap_msg,m,a); tg_trap(trap_msg);}}
#define TRAP_LT(a,b,m)	{if (a<b) { sprintf_P(trap_msg,m,a); tg_trap(trap_msg);}}
#define TRAP_EQ(a,b,m)	{if (a=b) { sprintf_P(trap_msg,m,a); tg_trap(trap_msg);}}
#define TRAP_ZERO(a,m)	{if (a<ROUNDING_ERROR) { sprintf_P(trap_msg,m,a); tg_trap(trap_msg);}}
#else
#define TRAP(a,m)
#define TRAP_GT(a,b,m)
#define TRAP_LT(a,b,m)
#define TRAP_EQ(a,b,m)
#define TRAP_ZERO(a,m)
#endif

/* general utility */

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

// Constants
#define ONE_MINUTE_OF_MICROSECONDS 60000000
#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define MM_PER_INCH 25.4
#define RADIAN 57.2957795
#define ROUNDING_ERROR 0.0001		// for float compares
#define MAX_LONG 2147483647
#define MAX_ULONG 4294967295

/* Define axes and motors 
 *
 * Note that just define "AXES" to be the number you want doesn't mean
 * you don't have to change a lot of code. You do. Just less that would
 * otherwise be required.
 */

#define AXES 4			// number of axes supported in this version
#define MOTORS 4		// number of motors on the board

enum tgAxisNum {		// define axis numbers and array indexes
	NON_AXIS = -1,		// value for non-axis		
		X,				// X = 0
		Y,
		Z,
		A,
		B,
		C,
		U,
		V,
		W
};
#define NON_AXIS (-1)	// value for non-axis


/* TinyG return codes
 * The following return codes are unified for various TinyG functions.
 * The first codes (up to the line) are aligned with the XIO codes.
 * Please don't change them without checking the corresponding values in xio.h
 * If you mess with this be sure to change the print strings in 
 * tg_print_status found in controller.c
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
	TG_PARAMETER_NOT_FOUND,		// parameter not located
	TG_PARAMETER_UNDER_RANGE,	// parameter is too small
	TG_PARAMETER_OVER_RANGE,	// parameter is too large
	TG_BAD_NUMBER_FORMAT,		// number format error
	TG_FLOATING_POINT_ERROR,	// number conversion error
	TG_MOTION_CONTROL_ERROR,	// motion control failure
	TG_ARC_SPECIFICATION_ERROR,	// arc specification error
	TG_ZERO_LENGTH_MOVE,		// XYZA move is zero length
	TG_MAX_FEED_RATE_EXCEEDED,
	TG_MAX_SEEK_RATE_EXCEEDED,
	TG_MAX_TRAVEL_EXCEEDED,
	TG_MAX_SPINDLE_SPEED_EXCEEDED,
	TG_FAILED_TO_CONVERGE
};

/*
 * Common typedefs (see xio.h for some others)
 */										// pointers to functions:
typedef void (*fptr_void_uint8) (void); // returns void, unit8_t arg (poll_func)
typedef char (*fptr_char_void) (void); 	// returns char, void args
typedef int (*fptr_int_uint8)(uint8_t s);// returns int, unit8_t arg (signal handler) 
typedef int (*fptr_int_char_p) (char *b);// returns int, character pointer (line handler)


/* Version String */
//#define TINYG_VERSION "build 210 - \"Aphasia\""
//#define TINYG_VERSION "build 213 - \"Bezoar\""
//#define TINYG_VERSION "build 214 - \"Chapped lips\""
//#define TINYG_VERSION "build 215 - \"Dropsy\""
//#define TINYG_VERSION "build 216 - \"Eczema\""
//#define TINYG_VERSION "build 217 - \"Fainting spells\""
//#define TINYG_VERSION "build 220 - \"Gout\""
//#define TINYG_VERSION "build 221 - \"Hacking cough\""
//#define TINYG_VERSION "build 222 - \"turning Japanese\""
//#define TINYG_VERSION "build 223 - \"Impetigo\""
//#define TINYG_VERSION "build 226 - \"Jaundice\""
//#define TINYG_VERSION "build 227 - \"Krupka\""
//#define TINYG_VERSION "build 228 - \"Lumbago\""
//#define TINYG_VERSION "build 229 - \"Mumps\""
//#define TINYG_VERSION "build 230 - \"Neutropenia\""
//#define TINYG_VERSION "build 234 - \"Oral leukoplakia\""
//#define TINYG_VERSION "build 302 - \"Pneumonia\""
//#define TINYG_VERSION "build 303 - \"Q fever\""
//#define TINYG_VERSION "build 304 - \"Radiophobia\""
//#define TINYG_VERSION "build 305 - \"Shisto\""
//#define TINYG_VERSION "build 306 - \"Teratoma\""
#define TINYG_VERSION "build 307 - \"Uremia\""
//#define TINYG_VERSION "build 308 - \"Valvano\""


/*** things that are actually allocated ***/

/*
 * ritorno - ritorno is Italian for return - it returns only if an error occurred
 */
uint8_t ritcode;	// defined once globally for ritorno
#define ritorno(a) if((ritcode=a) != TG_OK) { return(ritcode); }

/*
 * Device structure - structure to allow iteration through shared devices
 */
struct deviceSingleton { 
	struct PORT_struct *port[MOTORS];// motor control port
};
struct deviceSingleton device;

#endif
