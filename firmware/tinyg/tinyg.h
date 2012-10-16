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
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef tinyg_h
#define tinyg_h

// NOTE: This header requires <stdio.h> be included previously

#define TINYG_VERSION_NUMBER	0.95
#define TINYG_BUILD_NUMBER   	342.08

/****** DEVELOPMENT SETTINGS ******/

//#define __CANNED_STARTUP			// run any canned startup moves
//#define __DISABLE_EEPROM_INIT		// disable EEPROM init for faster simulation
//#define __DISABLE_TRANSMIT		// disable serial tranmission (TX)
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
#define __UNFORGIVING			// fail hard versus introducing motion errors

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
#define PWMS 2					// number of supported PWM channels

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

enum tgPWMnum {				// define motor numbers and array indexes
		PWM_1 = 0,
		PWM_2
};


/* TinyG status codes
 * The first code range (0-19) is aligned with the XIO codes and must be so.
 * Please don't change them without checking the corresponding values in xio.h
 *
 * Any changes to the ranges also require changing the message strings and 
 * string array in controller.c
 */
 
// OS, communications and low-level status (must align with XIO_xxxx codes in xio.h)
#define	TG_OK 0							// function completed OK
#define	TG_ERROR 1						// generic error return (EPERM)
#define	TG_EAGAIN 2						// function would block here (call again)
#define	TG_NOOP 3						// function had no-operation
#define	TG_COMPLETE 4					// operation is complete
#define TG_TERMINATE 5					// operation terminated (gracefully)
#define TG_ABORT 6						// operaation aborted
#define	TG_EOL 7						// function returned end-of-line
#define	TG_EOF 8						// function returned end-of-file 
#define	TG_FILE_NOT_OPEN 9
#define	TG_FILE_SIZE_EXCEEDED 10
#define	TG_NO_SUCH_DEVICE 11
#define	TG_BUFFER_EMPTY 12
#define	TG_BUFFER_FULL_FATAL 13 
#define	TG_BUFFER_FULL_NON_FATAL 14		// NOTE: XIO codes align to here
#define	TG_ERROR_15 15
#define	TG_ERROR_16 16
#define	TG_ERROR_17 17
#define	TG_ERROR_18 18
#define	TG_ERROR_19 19

// Internal errors and startup messages
#define	TG_INTERNAL_ERROR 20			// unrecoverable internal error
#define	TG_INTERNAL_RANGE_ERROR 21		// number range other than by user input
#define	TG_FLOATING_POINT_ERROR 22		// number conversion error
#define	TG_DIVIDE_BY_ZERO 23
#define	TG_ERROR_24 24
#define	TG_ERROR_25 25
#define	TG_ERROR_26 26
#define	TG_ERROR_27 27
#define	TG_ERROR_28 28
#define	TG_ERROR_29 29
#define	TG_ERROR_30 30
#define	TG_ERROR_31 31
#define	TG_ERROR_32 32
#define	TG_ERROR_33 33
#define	TG_ERROR_34 34
#define	TG_ERROR_35 35
#define	TG_ERROR_36 36
#define	TG_ERROR_37 37
#define	TG_ERROR_38 38
#define	TG_ERROR_39 39

// Input errors (400's, if you will)
#define	TG_UNRECOGNIZED_COMMAND 40		// parser didn't recognize the command
#define	TG_EXPECTED_COMMAND_LETTER 41	// malformed line to parser
#define	TG_BAD_NUMBER_FORMAT 42			// number format error
#define	TG_INPUT_EXCEEDS_MAX_LENGTH 43	// input string is too long 
#define	TG_INPUT_VALUE_TOO_SMALL 44		// input error: value is under minimum
#define	TG_INPUT_VALUE_TOO_LARGE 45		// input error: value is over maximum
#define	TG_INPUT_VALUE_RANGE_ERROR 46	// input error: value is out-of-range
#define	TG_INPUT_VALUE_UNSUPPORTED 47	// input error: value is not supported
#define	TG_JSON_SYNTAX_ERROR 48			// JSON string is not well formed
#define	TG_JSON_TOO_MANY_PAIRS 49		// JSON string or has too many JSON pairs
#define	TG_NO_BUFFER_SPACE 50			// Buffer pool is full and cannot perform this operation
#define	TG_ERROR_51 51
#define	TG_ERROR_52 52
#define	TG_ERROR_53 53
#define	TG_ERROR_54 54
#define	TG_ERROR_55 55
#define	TG_ERROR_56 56
#define	TG_ERROR_57 57
#define	TG_ERROR_58 58
#define	TG_ERROR_59 59

// Gcode and machining errors
#define	TG_ZERO_LENGTH_MOVE 60			// move is zero length
#define	TG_GCODE_BLOCK_SKIPPED 61		// block is too short - was skipped
#define	TG_GCODE_INPUT_ERROR 62			// general error for gcode input 
#define	TG_GCODE_FEEDRATE_ERROR 63		// move has no feedrate
#define	TG_GCODE_AXIS_WORD_MISSING 64	// command requires at least one axis present
#define	TG_MODAL_GROUP_VIOLATION 65		// gcode modal group error
#define	TG_HOMING_CYCLE_FAILED 66		// homing cycle did not complete
#define	TG_MAX_TRAVEL_EXCEEDED 67
#define	TG_MAX_SPINDLE_SPEED_EXCEEDED 68
#define	TG_ARC_SPECIFICATION_ERROR 69	// arc specification error


/* Version value and strings */

//#define TINYG_VERSION_NAME	  	"Argyle Socks"	// 0.911 build 324.15
//#define TINYG_VERSION_NAME	  	"Butt Slogan"	// 0.911 build 325.07
//#define TINYG_VERSION_NAME	  	"Crocs"			// 0.92	build 326.06
//#define TINYG_VERSION_NAME	  	"Daisy Dukes"
//#define TINYG_VERSION_NAME	  	"Elastic Belt"
//#define TINYG_VERSION_NAME	  	"Fanny Pack"
//#define TINYG_VERSION_NAME	  	"GoGo Boots"
#define TINYG_VERSION_NAME	  	"Hoodie"
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
