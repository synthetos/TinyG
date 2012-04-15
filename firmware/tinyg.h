/*
 * tinyg.h - tinyg main header - Application GLOBALS 
 *			 (see also system.h and settings.h)
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
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

#define TINYG_VERSION_NUMBER	0.93
#define TINYG_BUILD_NUMBER   	337.09

/****** DEVELOPMENT SETTINGS ******/

#define __CANNED_STARTUP			// run any canned startup moves
#define __DISABLE_EEPROM_INIT		// disable EEPROM init for faster simulation
//#define __DISABLE_TRANSMIT		// disable serial tranmission (TX)
//#define __DISABLE_STEPPERS		// disable steppers for faster simulation
//#define __SEGMENT_LOGGER			// enable segment logging to memory array
//#define __DEBUG					// enable debug (see util.c /.h)
// See the end of module header files to enable UNIT_TESTS

/****** OPERATING SETTINGS *******/

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

//++++ Diagnostic variables +++++
double x_step_counter;
double y_step_counter;
double z_step_counter;

/***** Boolean Comparisons *****/

#ifndef false
#define false 0
#endif
#ifndef true
#define true 1
#endif

#ifndef FALSE			// deprecated, use lowercase forms
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

/*************************************************************************
 * TinyG application-specific prototypes, defines and globals
 */

void tg_system_reset(void);
void tg_application_reset(void);
void tg_application_startup(void);

// global typedefs Used for accessing func pointers in PROGMEM & others
typedef void (*fptr_void_uint8)(void);	 	// returns void, unit8_t arg (poll_func)
typedef char (*fptr_char_void)(void); 		// returns char, void args
typedef int (*fptr_int_uint8)(uint8_t s);	// returns int, unit8_t arg (signal handler) 
typedef int (*fptr_int_char_p)(char *b);	// returns int, character pointer (line handler)
typedef void (*fptr_void_double)(double); 	// returns void, double arg (config bindings)

#define AXES 6					// number of axes supported in this version
#define MOTORS 4				// number of motors on the board
#define COORDS 6				// number of supported coordinate systems (1-6)
// if you change COORDS you must adjust the entries in cfgArray table in config.c

enum tgAxisNum {				// define axis numbers and array indexes
		X = 0,					// X = 0
		Y,
		Z,
		A,
		B,
		C,
		U,						// I don't actually intend to implement UVW
		V,						//...but they are reserved just in case
		W
};

enum tgMotorNum {				// define motor numbers and array indexes
		MOTOR_1 = 0,
		MOTOR_2,
		MOTOR_3,
		MOTOR_4
};

// Device structure - structure to allow iteration through shared devices
struct deviceSingleton {
	struct PORT_struct *port[MOTORS];// motor control port
};
struct deviceSingleton device;

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
	TG_ERROR,					// generic error return (EPERM)
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
//----- ...to here				// XIO codes only run to here
    TG_QUIT,					// QUIT current mode
	TG_UNRECOGNIZED_COMMAND,	// parser didn't recognize the command
	TG_RANGE_ERROR,				// number is out-of-range
	TG_EXPECTED_COMMAND_LETTER,	// malformed line to parser
	TG_JSON_SYNTAX_ERROR,		// JSON string is not well formed
	TG_INPUT_EXCEEDS_MAX_LENGTH,// input string is too long
	TG_OUTPUT_EXCEEDS_MAX_LENGTH,// output string is too long
	TG_INTERNAL_ERROR,			// an internal error occurred
	TG_BAD_NUMBER_FORMAT,		// number format error
	TG_FLOATING_POINT_ERROR,	// number conversion error
	TG_ARC_SPECIFICATION_ERROR,	// arc specification error
	TG_ZERO_LENGTH_MOVE,		// move is zero length
	TG_GCODE_BLOCK_SKIPPED,		// block is too short - was skipped
	TG_GCODE_INPUT_ERROR,		// general error for gcode input 
	TG_GCODE_FEEDRATE_ERROR,	// move has no feedrate
	TG_GCODE_AXIS_WORD_MISSING,	// command requires at least one axis present
	TG_MODAL_GROUP_VIOLATION,	// gcode modal group error
	TG_HOMING_CYCLE_FAILED,		// homing cycle did not complete
	TG_MAX_TRAVEL_EXCEEDED,
	TG_MAX_SPINDLE_SPEED_EXCEEDED,
};

/* Version value and strings */

//#define TINYG_VERSION_NAME	  	"Argyle Socks"	// 0.911 build 324.15
//#define TINYG_VERSION_NAME	  	"Butt Slogan"	// 0.911 build 325.07
//#define TINYG_VERSION_NAME	  	"Crocs"			// 0.92	build 326.06
//#define TINYG_VERSION_NAME	  	"Daisy Dukes"
//#define TINYG_VERSION_NAME	  	"Elastic Belt"
#define TINYG_VERSION_NAME	  	"Fanny Pack"
//#define TINYG_VERSION_NAME	  	"GoGo Boots"
//#define TINYG_VERSION_NAME	  	"Hoodie"
//#define TINYG_VERSION_NAME	  	"Ironic Hipster Fashion"
//#define TINYG_VERSION_NAME	  	"Jumpsuit"
//#define TINYG_VERSION_NAME	  	"Kulats"
//#define TINYG_VERSION_NAME	  	"Leisure Suit"
//#define TINYG_VERSION_NAME	  	"Mullet"
//#define TINYG_VERSION_NAME	  	"Nehru Jacket"
//#define TINYG_VERSION_NAME	  	"Overalls"
//#define TINYG_VERSION_NAME	  	"Platform SHoes"
//#define TINYG_VERSION_NAME	  	"Qu"
//#define TINYG_VERSION_NAME	  	"Romper"
//#define TINYG_VERSION_NAME	  	"Speedo"
//#define TINYG_VERSION_NAME	  	"Track Suit"
//#define TINYG_VERSION_NAME	  	"Ugg Boots"
//#define TINYG_VERSION_NAME	  	"Visible Thong"

/*
http://www.badfads.com/pages/fashion.html
http://www.divinecaroline.com/22255/107557-fashion-fails-twelve-styles-decade

 Add-a-Bead Necklace
 Burka
 Chate, Coonskin Cap
 Ed Hardy - by Christian Audigier 
 Fedoras
 Grecian Draping, Grills
 Harem Pants
 Ironed Hair
 Jellies, Juicy Tracksuit
 Man-from-Atlantis Sunglasses
 PVC Dress, Platform Sneakers, Parachute Pants
 Stretch Pants, Spandex Bodysuit
 Trucker Hat
 Zoot Suit, Zubaz
*/

#endif
