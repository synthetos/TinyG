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

void tg_system_init(void);
void tg_application_init(void);
void tg_application_startup(void);

// function pointer templates. Used for accessing func pointers in PROGMEM
typedef void (*fptr_void_uint8)(void);	 	// returns void, unit8_t arg (poll_func)
typedef char (*fptr_char_void)(void); 		// returns char, void args
typedef int (*fptr_int_uint8)(uint8_t s);	// returns int, unit8_t arg (signal handler) 
typedef int (*fptr_int_char_p)(char *b);	// returns int, character pointer (line handler)
typedef void (*fptr_void_double)(double); 	// returns void, double arg (config bindings)

#define AXES 6					// number of axes supported in this version
#define MOTORS 4				// number of motors on the board

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
    TG_QUIT,					// QUIT current mode
	TG_UNRECOGNIZED_COMMAND,	// parser didn't recognize the command
	TG_EXPECTED_COMMAND_LETTER,	// malformed line to parser
	TG_JSON_SYNTAX_ERROR,		// JSON string is not well formed
	TG_INPUT_EXCEEDS_MAX_LENGTH,// input string is too long
	TG_OUTPUT_EXCEEDS_MAX_LENGTH,// output string is too long
	TG_INTERNAL_ERROR,			// an internal error occurred
	TG_BAD_NUMBER_FORMAT,		// number format error
	TG_FLOATING_POINT_ERROR,	// number conversion error
	TG_ARC_SPECIFICATION_ERROR,	// arc specification error
	TG_ZERO_LENGTH_MOVE,		// XYZA move is zero length
	TG_MAX_TRAVEL_EXCEEDED,
	TG_MAX_SPINDLE_SPEED_EXCEEDED,
};

/* Version value and strings */

#define TINYG_VERSION_NUMBER	0.93
#define TINYG_BUILD_NUMBER   	329.38
#define TINYG_VERSION_NAME	  	"Fanny Pack"

//#define TINYG_VERSION "build 324.15 - \"Argyles\""
//#define TINYG_VERSION "0.911 (build 325.07 - \"Butt Slogan\")"
//#define TINYG_VERSION "0.92 (build 326.06 - \"Crocs\")"
//#define TINYG_VERSION "build 327.22 - \"Daisy Dukes\""
//#define TINYG_VERSION "build 328.05 - \"Elastic Belt\""
//#define TINYG_VERSION "build 329.37 - \"Fanny Pack\""
//#define TINYG_VERSION "build 330.01 - \"GoGo Boots\""
//#define TINYG_VERSION "build 331.01 - \"Hoodie\""
//#define TINYG_VERSION "build 332.01 - \"Ironic Hipster\""
//#define TINYG_VERSION "build 333.01 - \"Jumpsuit\""
//#define TINYG_VERSION "build 334.01 - \"Kulats\""
//#define TINYG_VERSION "build 335.01 - \"Leisure Suit\""
//#define TINYG_VERSION "build 336.01 - \"Mullet\""
//#define TINYG_VERSION "build 337.01 - \"Nehru Jacket\""
//#define TINYG_VERSION "build 338.01 - \"Overalls\""
//#define TINYG_VERSION "build 339.01 - \"Platform Shoes\""
//#define TINYG_VERSION "build 340.01 - \"Qu\""
//#define TINYG_VERSION "build 341.01 - \"Romper\""
//#define TINYG_VERSION "build 342.01 - \"Speedo\""
//#define TINYG_VERSION "build 343.01 - \"Track Suit\""
//#define TINYG_VERSION "build 344.01 - \"Ugg Boots\""
//#define TINYG_VERSION "build 345.01 - \"Visible Thong\""
//#define TINYG_VERSION "build 346.01 - \"W\""
//#define TINYG_VERSION "build 347.01 - \"X\""
//#define TINYG_VERSION "build 348.01 - \"Y\""
//#define TINYG_VERSION "build 349.01 - \"Zoot Suit\""

/*
http://www.badfads.com/pages/fashion.html
http://www.divinecaroline.com/22255/107557-fashion-fails-twelve-styles-decade

 Argyle Socks, Add-a-Bead Necklace
 Burka, Butt Slogans
 Crocs, Chate, Coonskin Cap
 Daisy Dukes
 Elastic Belt, Ed Hardy - by Christian Audigier 
 Fanny Pack, Fedoras
 GoGo Boots, Grecian Draping, Grills
 Hoodie, Harem Pants
 Ironic Hipster Fashions, Ironed Hair
 Jellies, Jumpsuits, Juicy Tracksuit
 Leisure Suit
 Mullet, Man-from-Atlantis Sunglasses
 Nehru Jacket
 Overalls
 PVC Dress, Platform Shoes, Platform Sneakers, Parachute Pants
 Rompers  
 Stretch Pants, Spandex Bodysuit, Speedos
 Track Suit, Trucker Hat
 Ugg Boots
 Visible Thongs 
 Zoot Suit, Zubaz
*/

#endif
