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

double vector[AXES];				// vector of axes for passing to subroutines

struct canonicalMachineSingleton {	// struct to manage cm globals and cycles
	uint8_t machine_state;			// see cmMachineState
	uint8_t hold_state;				// feedhold sub-state machine
	uint8_t canned_cycle_state;		// canned cycle sub-state machine
	uint8_t return_code;			// command return codes
	uint8_t status_report_counter;
}; struct canonicalMachineSingleton cm;

/* 
 * Definitions used by canonical machine and gcode interpreter
 */

/* 
 * Machine cycle state transition model
 *
 * The following variables track canonical machine state and state transitions.
 *
 *		- cm.machine_state
 *		- mr.feedhold_state
 *		- cm.cycle_start_asserted
 *
 *	Standard transitions:
 *
 *		machine_state[RESET] ---(cycle_start)---> machine_state[RUN]
 *		machine_state[RUN]	 ---(program_stop)--> machine_state[STOP]
 *		machine_state[RUN]	 ---(program_end)---> machine_state[RESET]
 *		machine_state[RUN]	 ---(abort (^x))----> machine_state[RESET]
 *		machine_state[RUN]	 ---(feedhold)------> machine_state[HOLD]
 *		machine_state[STOP]	 ---(cycle_start)---> machine_state[RUN]
 *		machine_state[HOLD]	 ---(cycle_start)---> machine_state[END_HOLD]
 *		machine_state[END_HOLD] ---(auto)-------> machine_state[RUN or STOP]
 *
 * Other transitions that can happen but are exceptions or ignored 
 *
 *		machine_state[RUN]	 ---(cycle_start)---> machine_state[RUN]
 *		machine_state[HOLD]	 ---(feedhold)------> machine_state[HOLD]
 *
 *	Sub-state machines manage transitions in cycles and feedholds, as well 
 *	as spindle state and program location (i.e. where will the the program
 *	resume after cycle_start is pushed) 
 *
 *	TODO: gm.program_flow needs to be integrated into this
 */

enum cmMachineState {
	MACHINE_RESET = 0,					// machine has been reset or aborted
	MACHINE_RUN,						// machine is running
	MACHINE_STOP,						// program stop or no more blocks
	MACHINE_HOLD,						// feedhold in progress
	MACHINE_END_HOLD					// transitional state to leave feedhold
};

enum cmFeedholdState {
	FEEDHOLD_OFF = 0,					// no feedhold in effect
	FEEDHOLD_SYNC, 						// sync to latest aline segment
	FEEDHOLD_PLAN, 						// replan blocks for feedhold
	FEEDHOLD_DECEL,						// decelerate to hold point
	FEEDHOLD_HOLD						// holding
};

/* The difference between NextAction and MotionMode is that NextAction is 
 * used by the current block, and may carry non-modal commands, whereas 
 * MotionMode persists across blocks (as G modal group 1)
 */

enum cmNextAction {						// motion mode and non-modals
	NEXT_ACTION_NONE = 0,				// no moves
	NEXT_ACTION_MOTION,					// action set by MotionMode
	NEXT_ACTION_DWELL,					// G4
	NEXT_ACTION_GO_HOME,				// G28
	NEXT_ACTION_OFFSET_COORDINATES		// G92
};

enum cmMotionMode {						// G Modal Group 1
	MOTION_MODE_STRAIGHT_TRAVERSE = 0,	// G0 - seek
	MOTION_MODE_STRAIGHT_FEED,			// G1 - feed
	MOTION_MODE_CW_ARC,					// G2 - arc feed
	MOTION_MODE_CCW_ARC,				// G3 - arc feed
	MOTION_MODE_STRAIGHT_PROBE,			// G38.2
	MOTION_MODE_CANCEL_MOTION_MODE,		// G80
	MOTION_MODE_CANNED_CYCLE_81,		// G81 - drilling
	MOTION_MODE_CANNED_CYCLE_82,		// G82 - drilling with dwell
	MOTION_MODE_CANNED_CYCLE_83,		// G83 - peck drilling
	MOTION_MODE_CANNED_CYCLE_84,		// G84 - right hand tapping
	MOTION_MODE_CANNED_CYCLE_85,		// G85 - boring, no dwell, feed out
	MOTION_MODE_CANNED_CYCLE_86,		// G86 - boring, spindle stop, rapid out
	MOTION_MODE_CANNED_CYCLE_87,		// G87 - back boring
	MOTION_MODE_CANNED_CYCLE_88,		// G88 - boring, spindle stop, manual out
	MOTION_MODE_CANNED_CYCLE_89			// G89 - boring, dwell, feed out
};

enum cmUnitsMode {
	MILLIMETER_MODE = 0,
	INCHES_MODE
};

enum cmDistanceMode {
	INCREMENTAL_MODE = 0,
	ABSOLUTE_MODE
};

enum cmPathControlMode {				// G Modal Group 13
	PATH_EXACT_STOP = 0,				// G61
	PATH_EXACT_PATH,					// G61.1
	PATH_CONTINUOUS,					// G64 and typically the default mode
	PATH_CONTINUOUS_FROM_ARC  			// special case for trajectory planner
};

enum cmProgramFlow {
	PROGRAM_FLOW_RUNNING = 0,			// must be zero
	PROGRAM_FLOW_PAUSED,
	PROGRAM_FLOW_COMPLETED
};

enum cmSpindleState {					// spindle settings
	SPINDLE_OFF = 0,
	SPINDLE_CW,
	SPINDLE_CCW
};

enum cmCanonicalPlane {					// canonical plane - translates to:
										// axis_0	axis_1	axis_2
	CANON_PLANE_XY = 0,					//   X		  Y		  Z
	CANON_PLANE_XZ,						//   X		  Z		  Y
	CANON_PLANE_YZ						//	 Y		  Z		  X							
};

enum cmDirection {						// used for spindle and arc dir
	DIRECTION_CW = 0,
	DIRECTION_CCW
};

enum cmAxisMode {			// axis modes (ordered: see _cm_get_feed_time())
	AXIS_DISABLED = 0,		// kill axis
	AXIS_STANDARD,			// axis in coordinated motion w/standard behaviors
	AXIS_INHIBITED,			// axis is computed but not activated
	AXIS_RADIUS,			// rotary axis calibrated to circumference
	AXIS_SLAVE_X,			// rotary axis slaved to X axis
	AXIS_SLAVE_Y,			// rotary axis slaved to Y axis
	AXIS_SLAVE_Z,			// rotary axis slaved to Z axis
	AXIS_SLAVE_XY,			// rotary axis slaved to XY plane
	AXIS_SLAVE_XZ,			// rotary axis slaved to XZ plane
	AXIS_SLAVE_YZ,			// rotary axis slaved to YZ plane
	AXIS_SLAVE_XYZ			// rotary axis slaved to XYZ movement
};	// ordering must be preserved. See _cm_get_feed_time() and seek time()

enum cmCycleState {
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

enum cmHomingState {
	HOMING_NOT_HOMED,
	HOMING_COMPLETE,
	HOMING_IN_PROCESS
};

/*--- helper functions for canonical machining functions ---*/
void cm_save_gcode_model(void);
void cm_restore_gcode_model(void);
double cm_get_position(uint8_t axis);
double *cm_get_gcode_model_position(double position[]);
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

uint8_t cm_cycle_start(void);						// (no Gcode)
uint8_t cm_program_stop(void);						// M0
uint8_t cm_optional_program_stop(void);				// M1
uint8_t cm_program_end(void);						// M2
uint8_t cm_feedhold(void);							// (no Gcode)
uint8_t cm_abort(void);								// (no Gcode)
uint8_t cm_exec_stop(void);
uint8_t cm_exec_end(void);

uint8_t cm_arc_feed(double target[],				// G2, G3
					double i, double j, double k,
					double radius, uint8_t motion_mode);

void cm_print_machine_state(void);
void cm_init_status_report(void);
void cm_decr_status_report(void);
void cm_force_status_report(void);
uint8_t cm_try_status_report(void);

/*--- canned cycles ---*/

//uint8_t cm_return_to_home(void);					// G28, G30
//uint8_t cm_run_return_to_home(void);

uint8_t cm_homing_cycle(void);
uint8_t cm_run_homing_cycle(void);

#endif
