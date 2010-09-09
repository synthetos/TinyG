/*
  config.h - configuration subsystem prototypes and 
  			 gcode, motion_control and stepper defaults
  
  Copyright (c) 2010 Alden S. Hart, Jr.

  This file has been completely re-written.from the original Grbl settings code
  TinyG configurations are held in the config struct (cfg)

 	Config				example	description
	-------------------	-------	---------------------------------------------
	(non-axis configs)
	config_version		1.00	config version
	mm_arc_segment		0.1		arc drawing resolution in millimeters per segment 

	(axis configs - one per axis - only X axis is shown)
	x_seek_steps_sec	1800	max seek whole steps per second for X axis
	x_feed_steps_sec	1200	max feed whole steps per second for X axis
	x_degree_per_step	1.8		degrees per whole step for X axis
	x_mm_per_rev		2.54	millimeters of travel per revolution of X axis
	x_mm_travel			406		millimeters of travel in X dimension (total)
  	x_microstep			8		microsteps to apply for X axis steps
	x_low_pwr_idle		1		1=low power idle mode, 0=full power idle mode 
	x_limit_enable		1		1=max limit switch enabled, 0=not enabled
*/

#ifndef config_h
#define config_h

/* config Function Prototypes */

//void config_init(void);	// init configuration subsystem (load settings from EEPROM)
void reset_settings(void);	// reset the struct only (no eeprom write)
void dump_settings(void);	// print current settings
int read_settings(void);	// read settings from eeprom
void write_settings(void);	// write settings to eeprom
void store_setting(int parameter, double value);  // helper method to set new settings from command line

void config_init(void);		// initialize config struct by reading from EEPROM
void config_reset(void);	// reset config values to defaults
int config_parse(char *text); // parse a tag=value config string
int config_read(void);		// read config record from EEPROM
void config_write(void);	// write config record to EEPROM
void config_test(void);		// unit tests for config routines


/* Base Configuration Values */

// variables
#define MM_PER_ARC_SEGMENT 0.1

// constants (from Grbl)
#define ONE_MINUTE_OF_MICROSECONDS 60000000.0
#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define CLK_MHZ = (F_CPU/1000000)				// clock in Mhz
#define INCHES_PER_MM (1.0/25.4) 				// A conversion

/*	Version of the EEPROM data. 
 *	Used to migrate existing data from older versions during firmware upgrades
 *	Stored in EEPROM byte 0
 */
#define SETTINGS_VERSION 1		// Grbl value 
#define CONFIG_VERSION 100		// TinyG value

struct Settings {				// Current global settings (persisted in EEPROM from byte 1 onwards)
	double steps_per_mm[3];
	double default_feed_rate;
	double default_seek_rate;
	double mm_per_arc_segment;
};
struct Settings settings;		// make it global


struct Config {
	/* general configs */
	uint8_t config_version;		// config format version. starts at 100
	double mm_per_arc_segment;	// arc drawing resolution in millimeters per segment (ex:0.1)

	/* per-axis values */
	uint16_t seek_steps_sec[4];	// max seek whole steps per second (ex: 1600)
	uint16_t feed_steps_sec[4];	// max feed whole steps per second (ex: 1200)
	double degree_per_step[4];	// degrees per whole step (ex: 1.8)
	double mm_per_rev[4];		// millimeters of travel per revolution (ex: 2.54)
	double mm_travel[4];		// millimeters of travel total in N dimension (ex: 400)
  	uint8_t microstep[4];		// microsteps to apply for each axis (ex: 8)
 	uint8_t low_pwr_idle[4];	// 1=low power idle mode, 0=full power idle mode
	uint8_t limit_enable[4];	// 1=limit switches enabled, 0=not enabled	

	/* computed values */		// see _config_computed() for derivations
	double steps_per_mm[4];		// # of steps (actually microsteps) per mm of travel
	double default_feed_rate;	// mm of travel in mm per second (was mm / minute in Grbl)
	double default_seek_rate;	// mm of travel in mm per second (was mm / minute in Grbl)
};
struct Config cfg; 				// make it global


/*******************************
 *
 * Motor and Robot Settings
 *
 *******************************/
  
// The following are the default settings (used when resetting eeprom-settings)

#define X_AXIS 0						// defines array indexes
#define Y_AXIS 1
#define Z_AXIS 2
#define A_AXIS 3

#define X_MICROSTEPS 8					// microsteps 
#define Y_MICROSTEPS 8					// (stepper driver configuration parameter)
#define Z_MICROSTEPS 8
#define A_MICROSTEPS 8
/*
#define X_MICROSTEPS 1					// microsteps 
#define Y_MICROSTEPS 1					// (stepper driver configuration parameter)
#define Z_MICROSTEPS 1					// FOR NOW YOU MUST ALSO SET MICROSTEP_UNITS_bm
#define A_MICROSTEPS 1
*/

#define X_SEEK_WHOLE_STEPS_PER_SEC 1600	// max whole steps per second for G0 motion
#define Y_SEEK_WHOLE_STEPS_PER_SEC 1600 // (motor parameter)
#define Z_SEEK_WHOLE_STEPS_PER_SEC 1600
#define A_SEEK_WHOLE_STEPS_PER_SEC 1600

#define X_SEEK_STEPS_PER_SEC (X_SEEK_WHOLE_STEPS_PER_SEC * X_MICROSTEPS)
#define Y_SEEK_STEPS_PER_SEC (Y_SEEK_WHOLE_STEPS_PER_SEC * Y_MICROSTEPS)
#define Z_SEEK_STEPS_PER_SEC (Z_SEEK_WHOLE_STEPS_PER_SEC * Z_MICROSTEPS)
#define A_SEEK_STEPS_PER_SEC (A_SEEK_WHOLE_STEPS_PER_SEC * A_MICROSTEPS)

#define X_FEED_WHOLE_STEPS_PER_SEC 1000	// max whole steps per sec for feed motion
#define Y_FEED_WHOLE_STEPS_PER_SEC 1000 // (motor parameter)
#define Z_FEED_WHOLE_STEPS_PER_SEC 1000
#define A_FEED_WHOLE_STEPS_PER_SEC 1000

#define X_FEED_STEPS_PER_SEC (X_FEED_WHOLE_STEPS_PER_SEC * X_MICROSTEPS)
#define Y_FEED_STEPS_PER_SEC (Y_FEED_WHOLE_STEPS_PER_SEC * Y_MICROSTEPS)
#define Z_FEED_STEPS_PER_SEC (Z_FEED_WHOLE_STEPS_PER_SEC * Z_MICROSTEPS)
#define A_FEED_STEPS_PER_SEC (A_FEED_WHOLE_STEPS_PER_SEC * A_MICROSTEPS)

#define X_DEGREE_PER_WHOLE_STEP	1.8		// degrees per whole step
#define Y_DEGREE_PER_WHOLE_STEP	1.8 	// (motor parameter)
#define Z_DEGREE_PER_WHOLE_STEP	1.8
#define A_DEGREE_PER_WHOLE_STEP	1.8

#define X_DEGREE_PER_STEP (X_DEGREE_PER_WHOLE_STEP / X_MICROSTEPS)
#define Y_DEGREE_PER_STEP (Y_DEGREE_PER_WHOLE_STEP / Y_MICROSTEPS)
#define Z_DEGREE_PER_STEP (Z_DEGREE_PER_WHOLE_STEP / Z_MICROSTEPS)
#define A_DEGREE_PER_STEP (A_DEGREE_PER_WHOLE_STEP / A_MICROSTEPS)

#define X_MM_PER_REVOLUTION 2.54		// typically 0.100" per revolution
#define Y_MM_PER_REVOLUTION 2.54		// (machine parameter)
#define Z_MM_PER_REVOLUTION 2.54
#define A_MM_PER_REVOLUTION 2.54

#define X_MM_TRAVEL 400					// full excursion from min to max 
#define Y_MM_TRAVEL 400					// (machine parameter)
#define Z_MM_TRAVEL 300
#define A_MM_TRAVEL -1					// -1 is no limit (typ for rotary axis)

#define X_LIMIT_ENABLE TRUE				// 1=limit switches present and enabled
#define Y_LIMIT_ENABLE TRUE				// (machine parameter)
#define Z_LIMIT_ENABLE TRUE
#define A_LIMIT_ENABLE FALSE

#define X_LOW_POWER_IDLE_ENABLE TRUE	// 1=low power idle enabled 
#define Y_LOW_POWER_IDLE_ENABLE TRUE	// (machine parameter)
#define Z_LOW_POWER_IDLE_ENABLE TRUE
#define A_LOW_POWER_IDLE_ENABLE TRUE

#define X_STEPS_PER_MM ((360 / X_DEGREE_PER_STEP) / X_MM_PER_REVOLUTION)
#define Y_STEPS_PER_MM ((360 / Y_DEGREE_PER_STEP) / Y_MM_PER_REVOLUTION)
#define Z_STEPS_PER_MM ((360 / Z_DEGREE_PER_STEP) / Z_MM_PER_REVOLUTION)
#define A_STEPS_PER_MM ((360 / A_DEGREE_PER_STEP) / A_MM_PER_REVOLUTION)

// in millimeters per minute...
#define DEFAULT_FEEDRATE X_FEED_STEPS_PER_SEC / (360/X_DEGREE_PER_STEP)
#define DEFAULT_SEEKRATE X_SEEK_STEPS_PER_SEC / (360/X_DEGREE_PER_STEP)

/* Port configs - motor port bits are:
 *	b7	(in) max limit switch  	// alt: (out) spindle direction on A axis
 *	b6	(in) min limit switch	// alt: (out) spindle enable on A axis
 *	b5	(out) output bit for encoder port
 *	b4	(out) microstep 1
 *	b3	(out) microstep 0 
 *	b2	(out) motor enable
 *	b1	(out) direction
 *	b0	(out) step
 */
#define X_MOTOR_PORT PORTA		// labeled as motor #1
#define Y_MOTOR_PORT PORTF		//					#2
#define Z_MOTOR_PORT PORTE		//					#3
#define A_MOTOR_PORT PORTD		//					#4

#define X_MOTOR_PORT_DIR_gm		0x3F		// direction register settings
#define Y_MOTOR_PORT_DIR_gm		0x3F
#define Z_MOTOR_PORT_DIR_gm		0x3F
#define A_MOTOR_PORT_DIR_gm		0x3F		// spindle out bits are also on b7 & b6

#define STEP_PULSE_MICROSECONDS	2			// step pulse width microseconds (delay)

/* port constants */

#define MAX_LIMIT_BIT_bp		7			// motor control port bit positions
#define MIN_LIMIT_BIT_bp		6
#define ENCODER_OUT_BIT_bp		5			// 4 encoder out bits total, one from each axis
#define MICROSTEP_BIT_1_bp		4
#define MICROSTEP_BIT_0_bp		3
#define MOTOR_ENABLE_BIT_bp 	2
#define DIRECTION_BIT_bp		1
#define STEP_BIT_bp				0

#define MAX_LIMIT_BIT_bm		(1<<MAX_LIMIT_BIT_bp)		// motor control port bit masks
#define MIN_LIMIT_BIT_bm		(1<<MIN_LIMIT_BIT_bp)
#define ENCODER_OUT_BIT_bm		(1<<ENCODER_OUT_BIT_bp)		
#define MICROSTEP_BIT_1_bm		(1<<MICROSTEP_BIT_1_bp)
#define MICROSTEP_BIT_0_bm		(1<<MICROSTEP_BIT_0_bp)
#define MOTOR_ENABLE_BIT_bm 	(1<<MOTOR_ENABLE_BIT_bp)
#define DIRECTION_BIT_bm		(1<<DIRECTION_BIT_bp)
#define STEP_BIT_bm				(1<<STEP_BIT_bp)

#define MICROSTEP_FULL_bm (~MICROSTEP_BIT_1_bm | ~MICROSTEP_BIT_0_bm)
#define MICROSTEP_HALF_bm (~MICROSTEP_BIT_1_bm | MICROSTEP_BIT_0_bm)
#define MICROSTEP_QUARTER_bm (MICROSTEP_BIT_1_bm | ~MICROSTEP_BIT_0_bm)
#define MICROSTEP_EIGHTH_bm (MICROSTEP_BIT_1_bm | MICROSTEP_BIT_0_bm)
#define MICROSTEP_UNITS_bm MICROSTEP_EIGHTH_bm

/* bit positions and masks used by line buffer and some other routines */

#define X_BIT_bp				0			// general purpose bit position for x axis
#define Y_BIT_bp				1
#define Z_BIT_bp				2
#define A_BIT_bp				3
#define X_DIRECTION_BIT_bp		4			// secondary bit position for x axis
#define Y_DIRECTION_BIT_bp		5
#define Z_DIRECTION_BIT_bp		6
#define A_DIRECTION_BIT_bp		7

#define X_BIT_bm				(1<<X_BIT_bp)// general purpose bit mask for x axis
#define Y_BIT_bm				(1<<Y_BIT_bp)
#define Z_BIT_bm				(1<<Z_BIT_bp)
#define A_BIT_bm				(1<<A_BIT_bp)
#define X_DIRECTION_BIT_bm		(1<<X_DIRECTION_BIT_bp)	// secondary bit mask for x axis
#define Y_DIRECTION_BIT_bm		(1<<Y_DIRECTION_BIT_bp)
#define Z_DIRECTION_BIT_bm		(1<<Z_DIRECTION_BIT_bp)
#define A_DIRECTION_BIT_bm		(1<<A_DIRECTION_BIT_bp)

/* spindle config and constants - bits use the min/max bits from the A axis as outputs */

#define SPINDLE_ENABLE_PORT 	A_MOTOR_PORT
#define SPINDLE_ENABLE_BIT_bm 	(1<<6)		// also used to set port I/O direction

#define SPINDLE_DIRECTION_PORT 	A_MOTOR_PORT
#define SPINDLE_DIRECTION_BIT_bm (1<<7)		// also used to set port I/O direction

/* timer configs */

#define X_TIMER			TCC0				// x-axis timer
#define Y_TIMER			TCD0
#define Z_TIMER			TCE0
#define A_TIMER			TCF0

#define X_TIMER_vect 	TCC0_OVF_vect		// x-axis step rate timer vector
#define Y_TIMER_vect 	TCD0_OVF_vect
#define Z_TIMER_vect 	TCE0_OVF_vect
#define A_TIMER_vect 	TCF0_OVF_vect

/* timer and rate constants */

#define TC_WGMODE		0					// normal mode (count to TOP and rollover)
#define TC_OVFINTLVL	3					// high level interrupt

#define TC_CLK_OFF 		0					// turn timer off (clock = 0 Hz)
#define TC_CLK_DIV_1 	1					// timer freq =  32 Mhz
#define TC_CLK_DIV_2 	2					// timer freq =  16 Mhz (32Mhz / 2)
#define TC_CLK_DIV_4 	3					// timer freq =   8 Mhz (32Mhz / 4)
#define TC_CLK_DIV_8 	4					// timer freq =   4 Mhz (32Mhz / 8)
#define TC_CLK_DIV_64 	5					// timer freq = 500 Khz (32Mhz / 64)
#define TC_CLK_DIV_256 	6					// timer freq = 125 Khz (32Mhz / 256)

											// times below reflect 32 MHz CPU clock:
//#define DIV1_RANGE 	(2^16/(F_CPU/1000000))	// up to   2.048 mSec / pulse (~500+ pps)
//#define DIV2_RANGE 	(2^16/(F_CPU/2000000))	// up to   4.096 mSec / pulse (~250-500 pps)
//#define DIV4_RANGE 	(2^16/(F_CPU/4000000))	// up to   8.192 mSec / pulse (~125-250 pps)
//#define DIV8_RANGE 	(2^16/(F_CPU/8000000))	// up to  16.384 mSec / pulse (~ 60-125 pps)
//#define DIV64_RANGE (2^16/(F_CPU/6400000))	// up to 131.072 mSec / pulse (~  8-60
//#define DIV256_RANGE (2^16/(F_CPU/2560000))	// up to 524.288 mSec / pulse 
#define DIV1_RANGE 	(2048)
#define DIV2_RANGE 	(4096)
#define DIV4_RANGE 	(8192)
#define DIV8_RANGE 	(16384)
#define DIV64_RANGE (131072)
#define DIV256_RANGE (544288)

#endif
