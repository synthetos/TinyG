/*
 * config.h - configuration sub-system
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

#ifndef config_h
#define config_h

#define CFG_NVM_BASE 0x0000		// base address of usable NVM

/*
 * Global Scope Functions
 */
void cfg_init(void);
void cfg_init_gcode_model(void);
uint8_t cfg_config_parser(char *block, uint8_t display, uint8_t persist);
char cfg_get_configuration_group_char(int8_t group);
void cfg_dump_NVM(const uint16_t start_record, const uint16_t end_record, char *label);

#ifdef __UNIT_TESTS
void cfg_unit_tests(void);
#endif

/*
 * Global scope config structs
 */

struct cfgStructAxis {
	uint8_t axis_mode;		// see tgAxisMode in gcode.h
	uint8_t limit_mode;		// 1=limit switches enabled, 0=not enabled
	double seek_rate_max;	// max velocity in mm/min or deg/min
	double feed_rate_max;	// max velocity in mm/min or deg/min
	double travel_hard_limit;// distance between crashes or switches
	double travel_soft_limit;// work envelope w/warned or rejected blocks
	double radius;			// radius in mm for rotary axis modes

	// homing cycle settings
	uint8_t homing_enable;	// homing enabled for this axis
	double homing_rate;		// homing seek rate
	double homing_close;	// homing close rate
	double homing_offset;	// offset from zero at minimum
	double homing_backoff;	// axis backoff
};

struct cfgStructMotor {
	uint8_t	motor_map;		// map motor to axis
  	uint8_t microsteps;		// microsteps to apply for each axis (ex: 8)
	uint8_t polarity;		// 0=normal polarity, 1=reverse motor direction
 	uint8_t power_mode;		// 1=lo power idle mode, 0=full power idle mode
	double step_angle;		// degrees per whole step (ex: 1.8)
	double travel_rev;		// mm or deg of travel per motor revolution
	double steps_per_unit;	// steps (usteps)/mm or deg of travel
};

// Global Config structure - main structure
struct cfgStructGlobal {
	// general settings / globals
	double min_segment_len;		// arc and line drawing resolution in mm
	double min_segment_time;	// minimum segment time in microseconds
	double linear_jerk_max;		// linear jerk constant
//	double tangential_jerk_upper_boundary;	// above use 1-sin rule
//	double tangential_jerk_lower_boundary;	// below use cos rule
	uint8_t homing_mode;		// 0=off, 1=power-on (G28)
	uint8_t homing_state;		// HOMING state
	uint8_t cycle_active;		// TRUE while cycle active (e.g. homing)
	uint8_t accel_enabled;		// enable acceleration control

	// axis and motor structs
	struct cfgStructAxis a[AXES];// settings for axes X,Y,Z,A B,C
	struct cfgStructMotor m[MOTORS];// settings for motors 1-4
};

struct cfgStructGlobal cfg; 	// declared in the header to make it global
#define CFG(x) cfg.a[x]			// handy macros for referencing axis values,
								// e.g: CFG(X_AXIS).steps_per_mm
#endif


