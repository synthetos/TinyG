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

// NOTE: This header requires <stdio.h> be included previously

/*************************************************************************
 * operating variables
 * :020000020100FB
 */

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

// RUNTIME SETTINGS
#define __UNFORGIVING			// fails hard versus introduce errors
#define __INFO					// enables exception logging (see util.h)

/*************************************************************************
 * TinyG application-specific prototypes, defines and globals
 */

void tg_system_init(void);
void tg_application_init(void);
void tg_application_startup(void);

typedef void (*fptr_void_uint8) (void); // returns void, unit8_t arg (poll_func)
typedef char (*fptr_char_void) (void); 	// returns char, void args
typedef int (*fptr_int_uint8)(uint8_t s);// returns int, unit8_t arg (signal handler) 
typedef int (*fptr_int_char_p) (char *b);// returns int, character pointer (line handler)

#define AXES 6					// number of axes supported in this version
#define MOTORS 4				// number of motors on the board

enum tgAxisNum {		// define axis numbers and array indexes
		X = 0,				// X = 0
		Y,
		Z,
		A,
		B,
		C,
		U,				// I don't actually intend to implement UVW
		V,				//...but they are reserved just in case
		W
};

/* TinyG return codes
 * The following return codes are unified for various TinyG functions.
 * The first codes (up to the line) are aligned with the XIO codes.
 * Please don't change them without checking the corresponding values in xio.h
 * If you mess with this be sure to change the print strings in 
 * tg_print_status found in controller.c
 */

/* Device structure - structure to allow iteration through shared devices */
struct deviceSingleton { 
	struct PORT_struct *port[MOTORS];// motor control port
};
struct deviceSingleton device;

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
	TG_INPUT_ERROR,				// input variables are incorrect
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
//#define TINYG_VERSION "build 307 - \"Uremia\""
//#define TINYG_VERSION "build 308.13 - \"Valvano\""
//#define TINYG_VERSION "build 311.06 - \"Whooping Cough\""
//#define TINYG_VERSION "build 312.04 - \"Xenophobia\""
//#define TINYG_VERSION "build 313.03 - \"Yellow Fever\""
//#define TINYG_VERSION "build 314.03 - \"Zygomycosis\""
//#define TINYG_VERSION "build 316.20 - \"Anaphylaxis\""
//#define TINYG_VERSION "build 319.32 - \"Croup\""
//#define TINYG_VERSION "build 320.16 - \"Dermatitis\""
//#define TINYG_VERSION "build 321.03 - \"Elephantitis\""
//#define TINYG_VERSION "build 322.02 - \"Filariasis\""
#define TINYG_VERSION "build 323.19 - \"Giardia\""

#endif
