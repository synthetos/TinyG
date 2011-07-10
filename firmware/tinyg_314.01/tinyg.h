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

// NOTE: This header requries <stdio.h> be included previously

/*************************************************************************
 * operating variables
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
#define __TRAPS					// enables exception logging (read down)

// DEVELOPMENT SETTINGS
//#define __CANNED_STARTUP		// run any canned startup moves
//#define __DISABLE_EEPROM		// disable EEPROM init for faster simulation
//#define __DISABLE_STEPPERS	// disable steppers for faster simulation
//#define __DISABLE_LIMITS		// disable limit switches for faster simulation
//#define __UNIT_TESTS			// include unit tests in the build
//#define __DEBUG				// enable debug (see below & end-file notes)

#ifdef __DEBUG					// debug details - see end of file for more
#define __dbECHO_INPUT_LINE		// echos input lines 				(controller.c:268)
#define __dbECHO_GCODE_BLOCK	// echos input to Gcode interpreter	(gcode.c)
#define __dbALINE_CALLED		// shows call to mp_aline() 		(planner.c)
#define __dbSHOW_QUEUED_LINE	// shows line being queued 			(motor_queue.c)
#define __dbSHOW_LIMIT_SWITCH	// shows switch closures 			(limit_switches.c)
#define __dbSHOW_CONFIG_STATE	// shows config settings			(config.c)
#define __dbCONFIG				// enable config.c debug statements
//#define __dbSHOW_LOAD_MOVE	// shows move being loaded 
#endif

// global allocation of debug control variables
	uint8_t dbECHO_INPUT_LINE;
	uint8_t dbECHO_GCODE_BLOCK;
	uint8_t dbALINE_CALLED;
	uint8_t dbSHOW_QUEUED_LINE;
	uint8_t dbSHOW_LIMIT_SWITCH;
	uint8_t dbSHOW_CONFIG_STATE;
	uint8_t dbCONFIG;
	uint8_t dbSHOW_LOAD_MOVE;

/*************************************************************************
 * general utility defines
 */

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

#ifndef EPSILON
#define EPSILON 0.0001						// rounding error for floats
#endif
#ifndef FLOAT_EQ
#define FLOAT_EQ(a,b) (fabs(a-b) < EPSILON)	// requires math.h to be included in each file used
#endif
#ifndef FLOAT_NE
#define FLOAT_NE(a,b) (fabs(a-b) > EPSILON)	// requires math.h to be included in each file used
#endif

#define MAX_LONG 2147483647
#define MAX_ULONG 4294967295

#define square(a) ((a)*(a))
#define cube(a) ((a)*(a)*(a))
#define cubert(a) pow((a), 0.33333333333333)
#define radical3 (1.73205080756888)
//for PI use M_PI as defined in math.h

/*************************************************************************
 * TinyG application-specific prototypes, defines and globals
 */

void tg_system_init(void);
void tg_application_init(void);
void tg_application_startup(void);

/* ritorno - ritorno is Italian for return - it returns only if an error occurred */
uint8_t ritcode;	// defined once globally for ritorno
#define ritorno(a) if((ritcode=a) != TG_OK) { return(ritcode); }

typedef void (*fptr_void_uint8) (void); // returns void, unit8_t arg (poll_func)
typedef char (*fptr_char_void) (void); 	// returns char, void args
typedef int (*fptr_int_uint8)(uint8_t s);// returns int, unit8_t arg (signal handler) 
typedef int (*fptr_int_char_p) (char *b);// returns int, character pointer (line handler)

#define AXES 6					// number of axes supported in this version
#define MOTORS 4				// number of motors on the board

enum tgAxisNum {		// define axis numbers and array indexes
	NON_AXIS = -1,		// value for non-axis		
		X,				// X = 0
		Y,
		Z,
		A,
		B,
		C,
		U,				// I don't actually intend to implement UVW
		V,				//...but they are reserved just in case
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
#define TINYG_VERSION "build 314.01 - \"Zygomycosis\""

/*************************************************************************
 * TRAPS and DEBUG - conditionally compiled exception logging
 *
 *	TRAPs are exception statements you may want to enable at run time.
 *	DEBUGs are statements you only want enabled during debugging, and 
 *	then probably only for one section of the code or another.
 *
 *	Both are controlled by setting (or not) certain #defines and are 
 *	coded so they occupy no RAM or program space if they are not enabled.
 *
 *	All TRAPs are enabled if __TRAPS is defined.
 *	Format strings should be in program memory, so use the PSTR macro.
 *	TRAP_IF_TRUE can take any valid expression that will evaluate to 
 *	0 or 1. The expression should be in parens.
 *	A closing semicolon is not required but is recommended for style.
 *
 *	TRAP usage examples:
 *		TRAP(PSTR("Line length is too short"));
 *		TRAP1(PSTR("Line length is too short: %f"), m->length);
 *		TRAP2(PSTR("Line length failed division: %f / %f"), m->length, m->divisor);
 *		TRAP_IF_TRUE((m->divisor == 0), PSTR("Dividing by zero, Bucko: %f"), m->divisor);
 */
#ifdef __TRAPS 
#define TRAP(msg) { fprintf_P(stderr,PSTR("#### TRAP #### ")); \
					fprintf_P(stderr,msg); \
					fprintf_P(stderr,PSTR("\n")); \
				  }

#define TRAP1(msg,a) { fprintf_P(stderr,PSTR("#### TRAP #### ")); \
					  fprintf_P(stderr,msg,a); \
					  fprintf_P(stderr,PSTR("\n")); \
					}

#define TRAP2(msg,a,b) { fprintf_P(stderr,PSTR("#### TRAP #### ")); \
						 fprintf_P(stderr,msg,a,b); \
						 fprintf_P(stderr,PSTR("\n")); \
					   }

#define TRAP_IF_TRUE(expr,msg,a) { if (expr == TRUE) { \
									   fprintf_P(stderr,PSTR("#### TRAP #### ")); \
									   fprintf_P(stderr,msg,a); \
									   fprintf_P(stderr,PSTR("\n")); \
								  }}

#else
#define TRAP(msg,a)
#define TRAP2(msg,a,b)
#define TRAP_IF_TRUE(expr,msg,a)
#endif

/*	DEBUG is more complicated. 
 *	DEBUG logging is enabled if __DEBUG is defined.
 *	__DEBUG enables a set of arbitrary __dbXXXXXX defines that control 
 *	various debug regions, e.g. __dbCONFIG to enable debugging in config.c.
 *	Each __dbXXXXXX pairs with a dbXXXXXX global variable used as a flag.
 *	Each dbXXXXXX is initialized to TRUE or FALSE at startup in main.c.
 *	dbXXXXXX is used as a condition to enable or disable logging.
 *	No varargs, so you must use the one with the right number of variables.
 *	A closing semicolon is not required but is recommended for style.
 *
 *	DEBUG usage examples:
 *		DEBUG0(dbCONFIG, PSTR("String with no variables"));
 *		DEBUG1(dbCONFIG, PSTR("String with one variable: %f"), double_var);
 *		DEBUG2(dbCONFIG, PSTR("String with two variables: %4.2f, %d"), double_var, int_var);
 */
#define DEBUG0(dbXXXXXX,msg) { if (dbXXXXXX == TRUE) { \
								fprintf_P(stderr,PSTR("DEBUG: ")); \
								fprintf_P(stderr,msg); \
								fprintf_P(stderr,PSTR("\n"));}}

#define DEBUG1(dbXXXXXX,msg,a) { if (dbXXXXXX == TRUE) { \
								fprintf_P(stderr,PSTR("DEBUG: ")); \
								fprintf_P(stderr,msg,a); \
								fprintf_P(stderr,PSTR("\n"));}}

#define DEBUG2(dbXXXXXX,msg,a,b) { if (dbXXXXXX == TRUE) { \
								fprintf_P(stderr,PSTR("DEBUG: ")); \
								fprintf_P(stderr,msg,a,b); \
								fprintf_P(stderr,PSTR("\n"));}}

#define DEBUG3(dbXXXXXX,msg,a,b,c) { if (dbXXXXXX == TRUE) { \
								fprintf_P(stderr,PSTR("DEBUG: ")); \
								fprintf_P(stderr,msg,a,b,c); \
								fprintf_P(stderr,PSTR("\n"));}}

#endif
