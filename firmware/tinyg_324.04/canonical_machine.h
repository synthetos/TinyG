/*
 * canonical_machine.h - rs274/ngc canonical machining functions
 * Part of TinyG project
 *
 * This code is a loose implementation of Kramer, Proctor and Messina's
 * canonical machining functions as described in the NIST RS274/NGC v3
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

#ifndef canonical_machine_h
#define canonical_machine_h

/*--- global canonical machine structures and definitions ---*/

double vector[AXES];	// vector of axes for passing to subroutines

struct canonicalMachineCycle {		// struct to manage cycles
	uint8_t state;					// cycle state
}; struct canonicalMachineCycle cy;

enum cyCycleState {
	CY_STATE_OFF,					// cycle is OFF (must be zero)
	CY_STATE_NEW,					// initial call to cycle
	CY_STATE_HOMING_X_START,		// start X homing move
	CY_STATE_HOMING_X_WAIT,			// wait for limit switch or end-of-move
	CY_STATE_HOMING_Y_START,
	CY_STATE_HOMING_Y_WAIT,
	CY_STATE_HOMING_Z_START,
	CY_STATE_HOMING_Z_WAIT,
	CY_STATE_HOMING_A_START,
	CY_STATE_HOMING_A_WAIT,
	CY_STATE_HOMING_RTZ_START,		// return to zero move
	CY_STATE_HOMING_RTZ_WAIT,
	CY_STATE_MAX
};

enum homingState {
	HOMING_NOT_HOMED,
	HOMING_COMPLETE,
	HOMING_IN_PROCESS
};

/*--- helper functions for canonical machining functions ---*/
void cm_save_gcode_model(void);
void cm_restore_gcode_model(void);
double cm_get_position(uint8_t axis);
uint8_t cm_get_next_action(void);
uint8_t cm_get_motion_mode(void);
uint8_t cm_get_inches_mode(void);
uint8_t cm_get_absolute_mode(void);
uint8_t cm_get_path_control_mode(void);
uint8_t cm_isbusy(void);
double *cm_set_vector(double x, double y, double z, double a, double b, double c);
//void cm_set_target(double vector[]);
void cm_set_target(double target[], double flag[]);
void cm_set_offset(double i, double j, double k);
void cm_set_radius(double r);
void cm_set_absolute_override(uint8_t absolute_override);

/*--- canonical machining functions ---*/
void cm_init_canon(void);					// init canonical machine

uint8_t cm_select_plane(uint8_t plane);
uint8_t cm_set_origin_offsets(double offset[]);
uint8_t cm_use_length_units(uint8_t inches_mode);	// G20, G21
uint8_t cm_set_distance_mode(uint8_t absolute_mode);// G90, G91
uint8_t cm_set_traverse_rate(double seek_rate);		// (no code)
uint8_t cm_straight_traverse(double target[]);

uint8_t cm_set_feed_rate(double feed_rate);			// F parameter
uint8_t cm_set_inverse_feed_rate_mode(uint8_t mode);// True= inv mode
uint8_t cm_set_motion_control_mode(uint8_t mode);	// G61, G61.1, G64
uint8_t cm_dwell(double seconds);					// G4, P parameter
uint8_t cm_straight_feed(double target[]); 

uint8_t cm_set_spindle_speed(double speed);			// S parameter
uint8_t cm_start_spindle_clockwise(void);			// M3
uint8_t cm_start_spindle_counterclockwise(void);	// M4
uint8_t cm_stop_spindle_turning(void);				// M5

uint8_t cm_change_tool(uint8_t tool);				// M6, T
uint8_t cm_select_tool(uint8_t tool);				// T parameter
uint8_t cm_comment(char *comment);					// comment handler
uint8_t cm_message(char *message);					// msg to console

uint8_t cm_program_stop(void);						// M0
uint8_t cm_optional_program_stop(void);				// M1
uint8_t cm_program_resume(void);					// (no code)
uint8_t cm_program_end(void);						// M2
uint8_t cm_async_stop(void);						// (no code)
uint8_t cm_async_start(void);						// (re)start motors
uint8_t cm_async_end(void);							// (no code)
uint8_t cm_stop(void);								// stop cycle

uint8_t cm_arc_feed(double target[],				// G2, G3
					double i, double j, double k,
					double radius, uint8_t motion_mode);

void cm_print_machine_state(void);

/*--- canonical machining cycles ---*/

//uint8_t cm_return_to_home(void);					// G28, G30
//uint8_t cm_run_return_to_home(void);

uint8_t cm_homing_cycle(void);
uint8_t cm_run_homing_cycle(void);

#endif
