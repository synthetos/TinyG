/*
 * util.h - a random assortment of useful functions
 * Part of TinyG project
 *
 * Copyright (c) 2011 Alden S. Hart Jr.
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
/* util.c/.h contains a dog's breakfast of supporting functions that are 
 * not specific to tinyg: including:
 *
 *	  - math and min/max utilities and extensions 
 *	  - vector manipulation utilities
 *	  - support for INFO traps
 *	  - support for debugging routines
 */  

#ifndef util_h
#define util_h

/****** DEVELOPMENT SETTINGS ******/ // See tinyg.h for runtime settings

#define __CANNED_STARTUP			// run any canned startup moves
#define __DISABLE_EEPROM_INIT		// disable EEPROM init for faster simulation
//#define __DISABLE_TRANSMIT		// disable serial tranmission (TX)
//#define __DISABLE_STEPPERS		// disable steppers for faster simulation
//#define __SEGMENT_LOGGER			// enable segment logging to memory array
//#define __DEBUG					// enable debug (see below & end-file notes)
// See the end of module header files to enable UNIT_TESTS

/****** Global Scope Variables and Functions ******/

double vector[AXES];				// vector of axes for passing to subroutines

uint8_t isnumber(char c);
uint8_t read_double(char *buf, uint8_t *i, double *double_ptr);

void copy_vector(double dest[], const double src[], uint8_t length);
void copy_axis_vector(double dest[], const double src[]);
void set_unit_vector(double unit[], double target[], double position[]);
double get_axis_vector_length(const double a[], const double b[]);
double *set_vector(double x, double y, double z, double a, double b, double c);
double *set_vector_by_axis(double value, uint8_t axis);
#define clear_vector(a) memset(a,0,sizeof(a))

// ritorno is a handy way to provide exception returns - it returns only 
// if an error occurred. (ritorno is Italian for return) 
uint8_t errcode;
#define ritorno(a) if((errcode=a) != TG_OK) { return(errcode); }

/***** Math Support *****/

// side-effect safe forms of min and max
#ifndef max
#define max(a,b) \
   ({ __typeof__ (a) A = (a); \
      __typeof__ (b) B = (b); \
      A>B ? A:B; })
#endif

#ifndef min
#define min(a,b) \
   ({ __typeof__ (a) A = (a); \
      __typeof__ (b) B = (b); \
      A<B ? A:B; })
#endif

#define min3(a,b,c) (min(min(a,b),c))
#define min4(a,b,c,d) (min(min(a,b),min(c,d)))
#define max3(a,b,c) (max(max(a,b),c))
#define max4(a,b,c,d) (max(max(a,b),max(c,d)))

#ifndef avg
#define avg(a,b) ((a+b)/2)
#endif
#define square(a) ((a)*(a))
#define cube(a) ((a)*(a)*(a))
#define cubert(a) pow((a), 0.33333333333333)

#ifndef EPSILON
#define EPSILON 0.0001					// rounding error for floats
#endif
#ifndef EQ
#define EQ(a,b) (fabs(a-b) < EPSILON)	// requires math.h to be included in each file used
#endif
#ifndef NE
#define NE(a,b) (fabs(a-b) > EPSILON)	// requires math.h to be included in each file used
#endif
#ifndef EQ_ZERO
#define EQ_ZERO(a) (fabs(a) < EPSILON)	// requires math.h to be included in each file used
#endif
#ifndef NE_ZERO
#define NE_ZERO(a) (fabs(a) > EPSILON)	// requires math.h to be included in each file used
#endif

// Constants
#define MAX_LONG (2147483647)
#define MAX_ULONG (4294967295)
#define MM_PER_INCH (25.4)
#define INCH_PER_MM (1/25.4)
#define MICROSECONDS_PER_MINUTE 60000000
#define uSec(a) (a * MICROSECONDS_PER_MINUTE)

#define RADIAN (57.2957795)
//		M_PI is pi as defined in math.h
//		M_SQRT2 is radical2 as defined in math.h
#define M_SQRT3 (1.73205080756888)


/***** INFO trap support ******
 *
 *	INFO traps are exception statements that can be enabled or disabled.
 *
 *	All INFOs are enabled if __INFO is defined (see tinyg.h runtime settings)
 *	INFOs are coded so they occupy no RAM or program space if not enabled.
 *	Format strings should be in program memory, so use the PSTR macro.
 *	A closing semicolon is not required but is recommended for style.
 *
 *	INFO usage examples:
 *		INFO(PSTR("Line length is too short"));
 *		INFO1(PSTR("Line length is too short: %f"), m->length);
 *		INFO2(PSTR("Line length failed division: %f / %f"), m->length, m->divisor);
 */
#ifdef __INFO 	// Note: __INFO is defined in tinyg.h as a runtime setting
#define INFO(msg) { fprintf_P(stderr,PSTR("#### INFO #### ")); \
					fprintf_P(stderr,msg); \
					fprintf_P(stderr,PSTR("\n")); \
				  }

#define INFO1(msg,a) { fprintf_P(stderr,PSTR("#### INFO #### ")); \
					  fprintf_P(stderr,msg,a); \
					  fprintf_P(stderr,PSTR("\n")); \
					}

#define INFO2(msg,a,b) { fprintf_P(stderr,PSTR("#### INFO #### ")); \
						 fprintf_P(stderr,msg,a,b); \
						 fprintf_P(stderr,PSTR("\n")); \
					   }

#define INFO3(msg,a,b,c) { fprintf_P(stderr,PSTR("#### INFO #### ")); \
						   fprintf_P(stderr,msg,a,b,c); \
						   fprintf_P(stderr,PSTR("\n")); \
						 }

#define INFO4(msg,a,b,c,d) { fprintf_P(stderr,PSTR("#### INFO #### ")); \
						 	 fprintf_P(stderr,msg,a,b,c,d); \
						 	 fprintf_P(stderr,PSTR("\n")); \
						   }
#else
#define INFO(msg)
#define INFO1(msg,a)
#define INFO2(msg,a,b)
#define INFO3(msg,a,b,c)
#define INFO4(msg,a,b,c,d)
#endif	// __INFO

/***** DEBUG support ******
 *
 *	DEBUGs are print statements you probably only want enabled during 
 *	debugging, and then probably only for one section of code or another.
 *
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
 *
 *	DEBUG print statements are coded so they occupy no program space if 
 *	they are not enabled. If you also use __dbXXXX defines to enable debug
 *	code these will - of course - be in the code regardless.
 *
 *	There are also a variety of module-specific diagnostic print statements 
 *	that are enabled or not depending on whether __DEBUG is defined
 */
 
#ifdef __DEBUG
void dump_everything(void);
void roll_over_and_die(void);
void print_scalar(char *label, double value);
void print_vector(char *label, double vector[], uint8_t length);

#define __dbECHO_GCODE_BLOCK	// echos input to Gcode interpreter	(gcode.c)
//#define __dbALINE_CALLED		// shows call to mp_aline() 		(planner.c)
//#define __dbSHOW_QUEUED_LINE	// shows line being queued 			(motor_queue.c)
//#define __dbSHOW_LIMIT_SWITCH	// shows switch closures 			(limit_switches.c)
//#define __dbSHOW_CONFIG_STATE	// shows config settings			(config.c)
//#define __dbCONFIG_DEBUG_ENABLED// enable config.c debug statements
//#define __dbSHOW_LOAD_MOVE	// shows move being loaded 

// global allocation of debug control variables
	uint8_t dbECHO_GCODE_BLOCK;
	uint8_t dbALINE_CALLED;
	uint8_t dbSHOW_QUEUED_LINE;
	uint8_t dbSHOW_LIMIT_SWITCH;
	uint8_t dbSHOW_CONFIG_STATE;
	uint8_t dbCONFIG_DEBUG_ENABLED;
	uint8_t dbSHOW_LOAD_MOVE;

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
#else
#define DEBUG0(dbXXXXXX,msg)
#define DEBUG1(dbXXXXXX,msg,a)
#define DEBUG2(dbXXXXXX,msg,a,b)
#define DEBUG3(dbXXXXXX,msg,a,b,c)
#endif	// __DEBUG

/***** Runtime Segment Data Logger Stuff *****
 *
 * This is independent of __DEBUG and does not need __DEBUG defined
 */
#ifdef __SEGMENT_LOGGER
#define SEGMENT_LOGGER_MAX 256

// segment logger structure and index
struct mpSegmentLog {
	uint8_t move_state;
	uint32_t linenum;
	uint32_t segments;
	double velocity;
	double microseconds;
//	double position_x;
//	double target_x;
//	double step_x;
//	double move_time;
//	double accel_time;
};
struct mpSegmentLog sl[SEGMENT_LOGGER_MAX];
uint16_t sl_index;

// function prototype and calling macro
void segment_logger(uint8_t move_state, 
					uint32_t linenum,
					uint32_t segments, 
					uint32_t segment_count, 
					double velocity,
					double microseconds,
//					double position_x, 
//					double target_x,
//					double step_x, 
//					double move_time,
//					double accel_time
					);

#define SEGMENT_LOGGER segment_logger(bf->move_state, \
									  mr.linenum, mr.segments, mr.segment_count, \
									  mr.segment_velocity, \
									  mr.microseconds);
/*
									  mr.microseconds, \
									  mr.position[X], \
									  mr.target[X], \
									  steps[X], \
									  mr.segment_move_time, \
									  mr.segment_accel_time);
*/
#else
#define SEGMENT_LOGGER
#endif	// __SEGMENT_LOGGER

#endif	// util_h
