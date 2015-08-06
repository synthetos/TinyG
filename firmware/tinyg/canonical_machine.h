/*
 * canonical_machine.h - rs274/ngc canonical machining functions
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart Jr.
 *
 * This code is a loose implementation of Kramer, Proctor and Messina's
 * canonical machining functions as described in the NIST RS274/NGC v3
 */
/* This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef CANONICAL_MACHINE_H_ONCE
#define CANONICAL_MACHINE_H_ONCE

#include "config.h"

/* Defines, Macros, and  Assorted Parameters */

#define MODEL 	(GCodeState_t *)&cm.gm		// absolute pointer from canonical machine gm model
#define PLANNER (GCodeState_t *)&bf->gm		// relative to buffer *bf is currently pointing to
#define RUNTIME (GCodeState_t *)&mr.gm		// absolute pointer from runtime mm struct
#define ACTIVE_MODEL cm.am					// active model pointer is maintained by state management

#define _to_millimeters(a) ((cm.gm.units_mode == INCHES) ? (a * MM_PER_INCH) : a)

#define JOGGING_START_VELOCITY ((float)10.0)
#define DISABLE_SOFT_LIMIT (-1000000)


/*****************************************************************************
 * MACHINE STATE MODEL
 *
 * The following main variables track canonical machine state and state transitions.
 *		- cm.machine_state	- overall state of machine and program execution
 *		- cm.cycle_state	- what cycle the machine is executing (or none)
 *		- cm.motion_state	- state of movement
 */
// *** Note: check config printout strings align with all the state variables

// ### LAYER 8 CRITICAL REGION ###
// ### DO NOT CHANGE THESE ENUMERATIONS WITHOUT COMMUNITY INPUT ###
typedef enum {				        // check alignment with messages in config.c / msg_stat strings
    COMBINED_INITIALIZING = 0,		// [0] machine is initializing          //iff macs == MACHINE_INITIALIZING
    COMBINED_READY,					// [1] machine is ready for use         //iff macs == MACHINE_READY
    COMBINED_ALARM,					// [2] machine in alarm state           //iff macs == MACHINE_ALARM
    COMBINED_PROGRAM_STOP,			// [3] program stop/no more blocks      //iff macs == MACHINE_PROGRAM_STOP
    COMBINED_PROGRAM_END,			// [4] program end                      //iff macs == MACHINE_PROGRAM_END
    COMBINED_RUN,					// [5] machine is running               //iff macs == MACHINE_CYCLE, cycs == CYCLE_OFF, mots != MOTION_HOLD
    COMBINED_HOLD,					// [6] machine is holding               //iff macs == MACHINE_CYCLE, cycs == CYCLE_OFF, mots == MOTION_HOLD
    COMBINED_PROBE,					// [7] probe cycle active               //iff macs == MACHINE_CYCLE, cycs == CYCLE_PROBE
    COMBINED_CYCLE,					// [8] reserved for canned cycles       < not used >
    COMBINED_HOMING,				// [9] homing cycle active              //iff macs == MACHINE_CYCLE, cycs = CYCLE_HOMING
    COMBINED_JOG,					// [10] jogging cycle active            //iff macs == MACHINE_CYCLE, cycs = CYCLE_JOG
    COMBINED_INTERLOCK,             // [11] machine in safety interlock hold//iff macs == MACHINE_INTERLOCK
    COMBINED_SHUTDOWN,				// [12] machine in shutdown state       //iff macs == MACHINE_SHUTDOWN
    COMBINED_PANIC				    // [13] machine in panic state          //iff macs == MACHINE_PANIC
} cmCombinedState;
//### END CRITICAL REGION ###

typedef enum {
    MACHINE_INITIALIZING = 0,		// machine is initializing
    MACHINE_READY,					// machine is ready for use
    MACHINE_ALARM,					// machine in alarm state
    MACHINE_PROGRAM_STOP,			// no blocks to run; like PROGRAM_END but without the M2 to reset gcode state
    MACHINE_PROGRAM_END,			// program end (same as MACHINE_READY, really...)
    MACHINE_CYCLE,					// machine is running; blocks still to run, or steppers are busy
    MACHINE_INTERLOCK,              // machine in interlock state
    MACHINE_SHUTDOWN,				// machine in shutdown state
    MACHINE_PANIC				    // machine in panic state
} cmMachineState;

typedef enum {
	CYCLE_OFF = 0,					// machine is idle
	CYCLE_MACHINING,				// in normal machining cycle
	CYCLE_PROBE,					// in probe cycle
	CYCLE_HOMING,					// homing is treated as a specialized cycle
	CYCLE_JOG						// jogging is treated as a specialized cycle
} cmCycleState;

typedef enum {
    MOTION_STOP = 0,				// motion has stopped: set when the steppers reach the end of the planner queue
    MOTION_PLANNING,				// machine has planned an ALINE segment, but not yet started to execute them
    MOTION_RUN,						// machine is in motion: set when the steppers execute an ALINE segment
    MOTION_HOLD						// feedhold in progress: set whenever we leave FEEDHOLD_OFF, unset whenever we enter FEEDHOLD_OFF
} cmMotionState;

typedef enum {				        // feedhold_state machine
	FEEDHOLD_OFF = 0,				// no feedhold in effect
	FEEDHOLD_SYNC, 					// start hold - sync to latest aline segment
	FEEDHOLD_PLAN, 					// replan blocks for feedhold
	FEEDHOLD_DECEL,					// decelerate to hold point
	FEEDHOLD_HOLD,					// holding
	FEEDHOLD_END_HOLD				// end hold (transient state to OFF)
} cmFeedholdState;

typedef enum {				        // feed override state machine
    MFO_OFF = 0,
    MFO_REQUESTED,
    MFO_SYNC
} cmOverrideState;

typedef enum {				        // master queue flush state machine
    FLUSH_OFF = 0,				    // no queue flush in effect
    FLUSH_REQUESTED,                // flush has been requested but not started yet
} cmQueueFlushState;

typedef enum {				        // applies to cm.homing_state
	HOMING_NOT_HOMED = 0,			// machine is not homed (0=false)
	HOMING_HOMED = 1,				// machine is homed (1=true)
	HOMING_WAITING					// machine waiting to be homed
} cmHomingState;

typedef enum {					    // applies to cm.probe_state
	PROBE_FAILED = 0,				// probe reached endpoint without triggering
	PROBE_SUCCEEDED = 1,			// probe was triggered, cm.probe_results has position
	PROBE_WAITING					// probe is waiting to be started
} cmProbeState;

typedef enum {
    SAFETY_INTERLOCK_ENGAGED = 0,   // meaning the interlock input is CLOSED (low)
    SAFETY_INTERLOCK_DISENGAGED
} cmSafetyState;

/* The difference between NextAction and MotionMode is that NextAction is
 * used by the current block, and may carry non-modal commands, whereas
 * MotionMode persists across blocks (as G modal group 1)
 */

typedef enum {						    // these are in order to optimized CASE statement
	NEXT_ACTION_DEFAULT = 0,			// Must be zero (invokes motion modes)
	NEXT_ACTION_SEARCH_HOME,			// G28.2 homing cycle
	NEXT_ACTION_SET_ABSOLUTE_ORIGIN,	// G28.3 origin set
	NEXT_ACTION_HOMING_NO_SET,			// G28.4 homing cycle with no coordinate setting
	NEXT_ACTION_SET_G28_POSITION,		// G28.1 set position in abs coordinates
	NEXT_ACTION_GOTO_G28_POSITION,		// G28 go to machine position
	NEXT_ACTION_SET_G30_POSITION,		// G30.1
	NEXT_ACTION_GOTO_G30_POSITION,		// G30
	NEXT_ACTION_SET_COORD_DATA,			// G10
	NEXT_ACTION_SET_ORIGIN_OFFSETS,		// G92
	NEXT_ACTION_RESET_ORIGIN_OFFSETS,	// G92.1
	NEXT_ACTION_SUSPEND_ORIGIN_OFFSETS,	// G92.2
	NEXT_ACTION_RESUME_ORIGIN_OFFSETS,	// G92.3
	NEXT_ACTION_DWELL,					// G4
	NEXT_ACTION_STRAIGHT_PROBE			// G38.2
} cmNextAction;

typedef enum {						    // G Modal Group 1
	MOTION_MODE_STRAIGHT_TRAVERSE=0,	// G0 - straight traverse
	MOTION_MODE_STRAIGHT_FEED,			// G1 - straight feed
	MOTION_MODE_CW_ARC,					// G2 - clockwise arc feed
	MOTION_MODE_CCW_ARC,				// G3 - counter-clockwise arc feed
	MOTION_MODE_CANCEL_MOTION_MODE,		// G80
	MOTION_MODE_STRAIGHT_PROBE,			// G38.2
	MOTION_MODE_CANNED_CYCLE_81,		// G81 - drilling
	MOTION_MODE_CANNED_CYCLE_82,		// G82 - drilling with dwell
	MOTION_MODE_CANNED_CYCLE_83,		// G83 - peck drilling
	MOTION_MODE_CANNED_CYCLE_84,		// G84 - right hand tapping
	MOTION_MODE_CANNED_CYCLE_85,		// G85 - boring, no dwell, feed out
	MOTION_MODE_CANNED_CYCLE_86,		// G86 - boring, spindle stop, rapid out
	MOTION_MODE_CANNED_CYCLE_87,		// G87 - back boring
	MOTION_MODE_CANNED_CYCLE_88,		// G88 - boring, spindle stop, manual out
	MOTION_MODE_CANNED_CYCLE_89			// G89 - boring, dwell, feed out
} cmMotionMode;

typedef enum {                          // Used for detecting gcode errors. See NIST section 3.4
	MODAL_GROUP_G0 = 0, 				// {G10,G28,G28.1,G92} 	non-modal axis commands (note 1)
	MODAL_GROUP_G1,						// {G0,G1,G2,G3,G80}	motion
	MODAL_GROUP_G2,						// {G17,G18,G19}		plane selection
	MODAL_GROUP_G3,						// {G90,G91}			distance mode
	MODAL_GROUP_G5,						// {G93,G94}			feed rate mode
	MODAL_GROUP_G6,						// {G20,G21}			units
	MODAL_GROUP_G7,						// {G40,G41,G42}		cutter radius compensation
	MODAL_GROUP_G8,						// {G43,G49}			tool length offset
	MODAL_GROUP_G9,						// {G98,G99}			return mode in canned cycles
	MODAL_GROUP_G12,					// {G54,G55,G56,G57,G58,G59} coordinate system selection
	MODAL_GROUP_G13,					// {G61,G61.1,G64}		path control mode
	MODAL_GROUP_M4,						// {M0,M1,M2,M30,M60}	stopping
	MODAL_GROUP_M6,						// {M6}					tool change
	MODAL_GROUP_M7,						// {M3,M4,M5}			spindle turning
	MODAL_GROUP_M8,						// {M7,M8,M9}			coolant (M7 & M8 may be active together)
	MODAL_GROUP_M9						// {M48,M49}			speed/feed override switches
} cmModalGroup;
#define MODAL_GROUP_COUNT (MODAL_GROUP_M9+1)
// Note 1: Our G0 omits G4,G30,G53,G92.1,G92.2,G92.3 as these have no axis components to error check

typedef enum {				        // canonical plane - translates to:
									// 		axis_0	axis_1	axis_2
	CANON_PLANE_XY = 0,				// G17    X		  Y		  Z
	CANON_PLANE_XZ,					// G18    X		  Z		  Y
	CANON_PLANE_YZ					// G19	  Y		  Z		  X
} cmCanonicalPlane;

typedef enum {
	INCHES = 0,						// G20
	MILLIMETERS,					// G21
	DEGREES							// ABC axes (this value used for displays only)
} cmUnitsMode;

typedef enum {
	ABSOLUTE_COORDS = 0,			// machine coordinate system
	G54,							// G54 coordinate system
	G55,							// G55 coordinate system
	G56,							// G56 coordinate system
	G57,							// G57 coordinate system
	G58,							// G58 coordinate system
	G59								// G59 coordinate system
} cmCoordSystem;
#define COORD_SYSTEM_MAX G59		// set this manually to the last one

typedef enum {
    ABSOLUTE_OVERRIDE_OFF = 0,      // G53 enabled
    ABSOLUTE_OVERRIDE_ON
} cmAbsoluteOverride;

typedef enum {			            // G Modal Group 13
	PATH_EXACT_PATH = 0,			// G61 - hits corners but does not stop if it does not need to.
	PATH_EXACT_STOP,				// G61.1 - stops at all corners
	PATH_CONTINUOUS					// G64 and typically the default mode
} cmPathControl;

typedef enum {
	ABSOLUTE_MODE = 0,				// G90
	INCREMENTAL_MODE				// G91
} cmDistanceMode;

typedef enum {
	INVERSE_TIME_MODE = 0,			// G93
	UNITS_PER_MINUTE_MODE,			// G94
	UNITS_PER_REVOLUTION_MODE		// G95 (unimplemented)
} cmFeedRateMode;

typedef enum {
	ORIGIN_OFFSET_SET=0,			// G92 - set origin offsets
	ORIGIN_OFFSET_CANCEL,			// G92.1 - zero out origin offsets
	ORIGIN_OFFSET_SUSPEND,			// G92.2 - do not apply offsets, but preserve the values
	ORIGIN_OFFSET_RESUME			// G92.3 - resume application of the suspended offsets
} cmOriginOffset;

typedef enum {
	PROGRAM_STOP = 0,
	PROGRAM_END
} cmProgramFlow;

typedef enum {				        // spindle state settings (See hardware.h for bit settings)
	SPINDLE_OFF = 0,
	SPINDLE_CW,
	SPINDLE_CCW
} cmSpindleState;

typedef enum {				        // mist and flood coolant states
	COOLANT_OFF = 0,				// all coolant off
	COOLANT_ON,						// request coolant on or indicates both coolants are on
	COOLANT_MIST,					// indicates mist coolant on
	COOLANT_FLOOD					// indicates flood coolant on
} cmCoolantState;

typedef enum {					    // used for spindle and arc dir
	DIRECTION_CW = 0,
	DIRECTION_CCW
} cmDirection;

typedef enum {					    // axis modes (ordered: see _cm_get_feed_time())
	AXIS_DISABLED = 0,				// kill axis
	AXIS_STANDARD,					// axis in coordinated motion w/standard behaviors
	AXIS_INHIBITED,					// axis is computed but not activated
	AXIS_RADIUS						// rotary axis calibrated to circumference
} cmAxisMode;	// ordering must be preserved. See cm_set_move_times()
#define AXIS_MODE_MAX_LINEAR AXIS_INHIBITED
#define AXIS_MODE_MAX_ROTARY AXIS_RADIUS


/*****************************************************************************
 * GCODE MODEL - The following GCodeModel/GCodeInput structs are used:
 *
 * - gm is the core Gcode model state. It keeps the internal gcode state model in
 *	 normalized, canonical form. All values are unit converted (to mm) and in the
 *	 machine coordinate system (absolute coordinate system). Gm is owned by the
 *	 canonical machine layer and should be accessed only through cm_ routines.
 *
 *	 The gm core struct is copied and passed as context to the runtime where it is
 *	 used for planning, replanning, and reporting.
 *
 * - gmx is the extended gcode model variables that are only used by the canonical
 *	 machine and do not need to be passed further down.
 *
 * - gn is used by the gcode interpreter and is re-initialized for each
 *   gcode block.It accepts data in the new gcode block in the formats
 *	 present in the block (pre-normalized forms). During initialization
 *	 some state elements are necessarily restored from gm.
 *
 * - gf is used by the gcode parser interpreter to hold flags for any data
 *	 that has changed in gn during the parse. cm.gf.target[] values are also used
 *	 by the canonical machine during set_target().
 *
 * - cfg (config struct in config.h) is also used heavily and contains some
 *	 values that might be considered to be Gcode model values. The distinction
 *	 is that all values in the config are persisted and restored, whereas the
 *	 gm structs are transient. So cfg has the G54 - G59 offsets, but gm has the
 *	 G92 offsets. cfg has the power-on / reset gcode default values, but gm has
 *	 the operating state for the values (which may have changed).
 */
typedef struct GCodeState {				// Gcode model state - used by model, planning and runtime
	uint32_t linenum;					// Gcode block line number
	uint8_t motion_mode;				// Group1: G0, G1, G2, G3, G38.2, G80, G81,
										// G82, G83 G84, G85, G86, G87, G88, G89

	float target[AXES]; 				// XYZABC where the move should go
	float work_offset[AXES];			// offset from the work coordinate system (for reporting only)

	float feed_rate; 					// F - normalized to millimeters/minute or in inverse time mode
	float parameter;					// P - parameter used for dwell time in seconds, G10 coord select...

	float move_time;					// optimal time for move given axis constraints
	float minimum_time;					// minimum time possible for move given axis constraints
	float spindle_speed;				// in RPM

	uint8_t mist_coolant;				// TRUE = mist on (M7), FALSE = off (M9)
	uint8_t flood_coolant;				// TRUE = flood on (M8), FALSE = off (M9)
	uint8_t spindle_mode;				// 0=OFF (M5), 1=CW (M3), 2=CCW (M4)

    cmFeedRateMode feed_rate_mode;      // See cmFeedRateMode for settings
    cmCanonicalPlane select_plane;      // G17,G18,G19 - values to set plane to
    cmUnitsMode units_mode;             // G20,G21 - 0=inches (G20), 1 = mm (G21)
    cmPathControl path_control;         // G61... EXACT_PATH, EXACT_STOP, CONTINUOUS
    cmDistanceMode distance_mode;       // G90=use absolute coords, G91=incremental movement
    cmDistanceMode arc_distance_mode;   // G90.1=use absolute IJK offsets, G91.1=incremental IJK offsets
    cmAbsoluteOverride absolute_override;// G53 TRUE = move using machine coordinates - this block only
    cmCoordSystem coord_system;         // G54-G59 - select coordinate system 1-9
    uint8_t tool;				// G	// M6 tool change - moves "tool_select" to "tool"
    uint8_t tool_select;		// G	// T value - T sets this value
} GCodeState_t;

typedef struct GCodeStateExtended {		// Gcode dynamic state extensions - used by model and arcs
	uint16_t magic_start;				// magic number to test memory integrity
	uint8_t next_action;				// handles G modal group 1 moves & non-modals
	uint8_t program_flow;				// used only by the gcode_parser

	float position[AXES];				// XYZABC model position (Note: not used in gn or gf)
	float origin_offset[AXES];			// XYZABC G92 offsets (Note: not used in gn or gf)
	float g28_position[AXES];			// XYZABC stored machine position for G28
	float g30_position[AXES];			// XYZABC stored machine position for G30

//    bool m48_enable;                    // master feedrate / spindle speed override enable
//    bool mfo_enable;                    // feedrate override enable
//    float mfo_factor;                   // 1.0000 x F feed rate. Go up or down from there
//    bool mto_enable;                    // traverse override enable
//    float mto_factor;                   // valid from 0.05 to 1.00

	float feed_rate_override_factor;	// 1.0000 x F feed rate. Go up or down from there
	float traverse_override_factor;		// 1.0000 x traverse rate. Go down from there
	uint8_t	feed_rate_override_enable;	// TRUE = overrides enabled (M48), F=(M49)
	uint8_t	traverse_override_enable;	// TRUE = traverse override enabled

	bool origin_offset_enable;          // G92 offsets enabled/disabled.  0=disabled, 1=enabled
	bool block_delete_switch;           // set true to enable block deletes (true is default)

	float spindle_override_factor;		// 1.0000 x S spindle speed. Go up or down from there
	uint8_t	spindle_override_enable;	// TRUE = override enabled

// unimplemented gcode parameters
//	float cutter_radius;				// D - cutter radius compensation (0 is off)
//	float cutter_length;				// H - cutter length compensation (0 is off)

	uint16_t magic_end;

} GCodeStateX_t;

typedef struct GCodeInput {				// Gcode model inputs - meaning depends on context

	uint8_t next_action;				// handles G modal group 1 moves & non-modals
	uint8_t motion_mode;				// Group1: G0, G1, G2, G3, G38.2, G80, G81,
										// G82, G83 G84, G85, G86, G87, G88, G89

	uint8_t modals[MODAL_GROUP_COUNT];
	uint8_t program_flow;				// used only by the gcode_parser
	uint32_t linenum;					// N word or autoincrement in the model
	float target[AXES]; 				// XYZABC where the move should go

	uint8_t L_word;						// L word - used by G10s
	float feed_rate; 					// F - normalized to millimeters/minute
	uint8_t feed_rate_mode;				// See cmFeedRateMode for settings

	float feed_rate_override_factor;	// 1.0000 x F feed rate. Go up or down from there
	float traverse_override_factor;		// 1.0000 x traverse rate. Go down from there
	uint8_t	feed_rate_override_enable;	// TRUE = overrides enabled (M48), F=(M49)
	uint8_t	traverse_override_enable;	// TRUE = traverse override enabled
	uint8_t override_enables;			// enables for feed and spoindle (GN/GF only)

//	bool m48_enable;			        // M48/M49 input (enables for feed and spindle)
//	bool mfo_enable;                    // M50 feedrate override enable
//	bool mto_enable;                    // Mxx traverse override enable
//	bool sso_enable;                    // M51 spindle speed override enable

	uint8_t select_plane;				// G17,G18,G19 - values to set plane to
	uint8_t units_mode;					// G20,G21 - 0=inches (G20), 1 = mm (G21)
	uint8_t coord_system;				// G54-G59 - select coordinate system 1-9
	uint8_t path_control;				// G61... EXACT_PATH, EXACT_STOP, CONTINUOUS
	uint8_t distance_mode;				// G91   0=use absolute coords(G90), 1=incremental movement
	uint8_t arc_distance_mode;			// G91.1   0=use absolute coords(G90), 1=incremental movement
	uint8_t origin_offset_mode;			// G92...TRUE=in origin offset mode
	uint8_t absolute_override;			// G53 TRUE = move using machine coordinates - this block only (G53)
	uint8_t tool;						// Tool after T and M6 (tool_select and tool_change)
	uint8_t tool_select;				// T value - T sets this value
	uint8_t tool_change;				// M6 tool change flag - moves "tool_select" to "tool"
	uint8_t mist_coolant;				// TRUE = mist on (M7), FALSE = off (M9)
	uint8_t flood_coolant;				// TRUE = flood on (M8), FALSE = off (M9)

//    uint8_t spindle_control;            // 0=OFF (M5), 1=CW (M3), 2=CCW (M4)
	uint8_t spindle_mode;				// 0=OFF (M5), 1=CW (M3), 2=CCW (M4)
	float spindle_speed;				// in RPM
	float spindle_override_factor;		// 1.0000 x S spindle speed. Go up or down from there
	uint8_t	spindle_override_enable;	// TRUE = override enabled

	float parameter;					// P - parameter used for dwell time in seconds, G10 coord select...
	float arc_radius;					// R - radius value in arc radius mode
	float arc_offset[3];  				// IJK - used by arc commands

// unimplemented gcode parameters
//	float cutter_radius;				// D - cutter radius compensation (0 is off)
//	float cutter_length;				// H - cutter length compensation (0 is off)

} GCodeInput_t;

typedef struct GCodeFlags {             // Gcode model input flags
    bool next_action;
    bool motion_mode;

    bool modals[MODAL_GROUP_COUNT];
    bool program_flow;
    bool linenum;
    bool target[AXES];

    bool L_word;
    bool feed_rate;
    bool feed_rate_mode;

	bool feed_rate_override_factor;	// 1.0000 x F feed rate. Go up or down from there
	bool traverse_override_factor;		// 1.0000 x traverse rate. Go down from there
	bool feed_rate_override_enable;	// TRUE = overrides enabled (M48), F=(M49)
	bool traverse_override_enable;	// TRUE = traverse override enabled
	bool override_enables;			// enables for feed and spoindle (GN/GF only)

    bool m48_enable;
    bool mfo_enable;
    bool mto_enable;
    bool sso_enable;

    bool select_plane;
    bool units_mode;
    bool coord_system;
    bool path_control;
    bool distance_mode;
    bool arc_distance_mode;
    bool origin_offset_mode;
    bool absolute_override;
    bool tool;
    bool tool_select;
    bool tool_change;
    bool mist_coolant;
    bool flood_coolant;

//    bool spindle_control;
	bool spindle_mode;
    bool spindle_speed;
    bool spindle_override_factor;
    bool spindle_override_enable;

    bool parameter;
    bool arc_radius;
    bool arc_offset[3];
} GCodeFlags_t;

/*****************************************************************************
 * CANONICAL MACHINE STRUCTURES
 */

typedef struct cmAxis {
	cmAxisMode axis_mode;				    // see tgAxisMode in gcode.h
	float velocity_max;                     // max velocity in mm/min or deg/min
	float feedrate_max;                     // max velocity in mm/min or deg/min
	float travel_max;                       // max work envelope for soft limits
	float travel_min;                       // min work envelope for soft limits
	float jerk_max;                         // max jerk (Jm) in mm/min^3 divided by 1 million
	float jerk_high;				    // high speed deceleration jerk (Jh) in mm/min^3 divided by 1 million
	float recip_jerk;                       // stored reciprocal of current jerk value - has the million in it
	float junction_dev;                     // aka cornering delta
	float radius;                           // radius in mm for rotary axis modes

    uint8_t homing_input;               // set 1-N for homing input. 0 will disable homing
    uint8_t homing_dir;                 // 0=search to negative, 1=search to positive
	float search_velocity;                  // homing search velocity
	float latch_velocity;                   // homing latch velocity
	float latch_backoff;                    // backoff from switches prior to homing latch movement
	float zero_backoff;                     // backoff from switches for machine zero
} cfgAxis_t;

typedef struct cmSingleton {                // struct to manage cm globals and cycles
	magic_t magic_start;                    // magic number to test memory integrity

	/**** Config variables (PUBLIC) ****/

	// system group settings
	float junction_acceleration;            // centripetal acceleration max for cornering
    float junction_aggression;		        // how aggressively will the machine corner? 1.0 or so is about the upper limit
	float chordal_tolerance;                // arc chordal accuracy setting in mm
    uint8_t soft_limit_enable;
    bool limit_enable;                      // true to enable limit switches (disabled is same as override)
    bool safety_interlock_enable;           // true to enable safety interlock system

	// hidden system settings
	float arc_segment_len;                  // arc drawing resolution in mm

	// gcode power-on default settings - defaults are not the same as the gm state
	cmCoordSystem default_coord_system;     // G10 active coordinate system default
	cmCanonicalPlane default_select_plane;  // G17,G18,G19 reset default
	cmUnitsMode default_units_mode;         // G20,G21 reset default
	cmPathControl default_path_control;     // G61,G61.1,G64 reset default
	cmDistanceMode default_distance_mode;   // G90,G91 reset default

	// coordinate systems and offsets
	float offset[COORDS+1][AXES];           // persistent coordinate offsets: absolute (G53) + G54,G55,G56,G57,G58,G59

	// settings for axes X,Y,Z,A B,C
	cfgAxis_t a[AXES];

	/**** Runtime variables (PRIVATE) ****/

    // global state variables and requestors

    cmMachineState machine_state;           // macs: machine/cycle/motion is the actual machine state
    cmCycleState cycle_state;               // cycs
    cmMotionState motion_state;             // momo
    cmFeedholdState hold_state;             // hold: feedhold state machine
    cmOverrideState mfo_state;              // feed override state machine
    cmQueueFlushState queue_flush_state;    // master queue flush state machine

    uint8_t safety_interlock_disengaged;    // set non-zero to start interlock processing (value is input number)
    uint8_t safety_interlock_reengaged;     // set non-zero to end interlock processing (value is input number)
    cmSafetyState safety_interlock_state;   // safety interlock state
    uint32_t esc_boot_timer;                // timer for Electronic Speed Control (Spindle electronics) to boot

    cmHomingState homing_state;             // home: homing cycle sub-state machine
    uint8_t homed[AXES];                    // individual axis homing flags

	uint8_t probe_state;                    // 1==success, 0==failed
	float probe_results[AXES];              // probing results

	uint8_t	g28_flag;                       // true = complete a G28 move
	uint8_t	g30_flag;                       // true = complete a G30 move
	uint8_t deferred_write_flag;            // G10 data has changed (e.g. offsets) - flag to persist them

	uint8_t feedhold_requested;             // feedhold character has been received
	uint8_t queue_flush_requested;          // queue flush character has been received
	uint8_t cycle_start_requested;          // cycle start character has been received (flag to end feedhold)
	float jogging_dest;                     // jogging direction as a relative move from current position

	bool end_hold_requested;                //
	uint8_t limit_requested;                // set non-zero to request limit switch processing (value is input number)
	uint8_t shutdown_requested;             // set non-zero to request shutdown in support of external estop (value is input number)

	/**** Model states ****/
	GCodeState_t *am;                       // active Gcode model is maintained by state management
	GCodeState_t  gm;                       // core gcode model state
	GCodeStateX_t gmx;                      // extended gcode model state
	GCodeInput_t  gn;                       // gcode input values - transient
	GCodeFlags_t  gf;                       // gcode input flags - transient

	magic_t magic_end;
} cmSingleton_t;

/**** Externs - See canonical_machine.c for allocation ****/

extern cmSingleton_t cm;				// canonical machine controller singleton


/*****************************************************************************
 * FUNCTION PROTOTYPES
 * Serves as a table of contents for the rather large canonical machine source file.
 */

/*--- Internal functions and helpers ---*/

// Model state getters and setters
cmCombinedState cm_get_combined_state(void);
cmMachineState cm_get_machine_state(void);
cmCycleState cm_get_cycle_state(void);
cmMotionState cm_get_motion_state(void);
cmFeedholdState cm_get_hold_state(void);
cmHomingState cm_get_homing_state(void);
uint8_t cm_get_jogging_state(void);
void cm_set_motion_state(uint8_t motion_state);

float cm_get_axis_jerk(uint8_t axis);
void cm_set_axis_jerk(uint8_t axis, float jerk);

uint32_t cm_get_linenum(const GCodeState_t *gcode_state);
uint8_t cm_get_motion_mode(const GCodeState_t *gcode_state);
uint8_t cm_get_coord_system(const GCodeState_t *gcode_state);
uint8_t cm_get_units_mode(const GCodeState_t *gcode_state);
uint8_t cm_get_select_plane(const GCodeState_t *gcode_state);
uint8_t cm_get_path_control(const GCodeState_t *gcode_state);
uint8_t cm_get_distance_mode(const GCodeState_t *gcode_state);
uint8_t cm_get_arc_distance_mode(const GCodeState_t *gcode_state);
uint8_t cm_get_feed_rate_mode(const GCodeState_t *gcode_state);
uint8_t cm_get_tool(const GCodeState_t *gcode_state);
uint8_t cm_get_spindle_mode(const GCodeState_t *gcode_state);
uint8_t	cm_get_block_delete_switch(void);
uint8_t cm_get_runtime_busy(void);
float cm_get_feed_rate(const GCodeState_t *gcode_state);

void cm_set_motion_mode(GCodeState_t *gcode_state, uint8_t motion_mode);
void cm_set_spindle_mode(GCodeState_t *gcode_state, uint8_t spindle_mode);
void cm_set_spindle_speed_parameter(GCodeState_t *gcode_state, float speed);
void cm_set_tool_number(GCodeState_t *gcode_state, uint8_t tool);
void cm_set_absolute_override(GCodeState_t *gcode_state, uint8_t absolute_override);
void cm_set_model_linenum(uint32_t linenum);

// Coordinate systems and offsets
float cm_get_active_coord_offset(const uint8_t axis);
float cm_get_work_offset(const GCodeState_t *gcode_state, const uint8_t axis);
void cm_set_work_offsets(GCodeState_t *gcode_state);
float cm_get_absolute_position(const GCodeState_t *gcode_state, const uint8_t axis);
float cm_get_work_position(const GCodeState_t *gcode_state, const uint8_t axis);

// Critical helpers
void cm_update_model_position_from_runtime(void);
void cm_finalize_move(void);
stat_t cm_deferred_write_callback(void);
void cm_set_model_target(const float target[], const bool flags[]);
stat_t cm_test_soft_limits(const float target[]);

/*--- Canonical machining functions (loosely) defined by NIST [organized by NIST Gcode doc] ---*/

// Initialization and termination (4.3.2)
void canonical_machine_init(void);
void canonical_machine_reset(void);
void canonical_machine_init_assertions(void);
stat_t canonical_machine_test_assertions(void);

// Alarms and state management
stat_t cm_alrm(nvObj_t *nv);                                    // trigger alarm from command input
stat_t cm_shutd(nvObj_t *nv);                                   // trigger shutdown from command input
stat_t cm_pnic(nvObj_t *nv);                                    // trigger panic from command input
stat_t cm_clr(nvObj_t *nv);                                     // clear alarm and shutdown from command input
void cm_clear(void);                                            // raw clear command
void cm_parse_clear(const char *s);                             // parse gcode for M30 or M2 clear condition
stat_t cm_is_alarmed(void);                                     // return non-zero status if alarm, shutdown or panic
void cm_halt_all(void);                                         // halt motion, spindle and coolant
void cm_halt_motion(void);                                      // halt motion (immediate stop) but not spindle & other IO
stat_t cm_alarm(const stat_t status, const char *msg);          // enter alarm state - preserve Gcode state
stat_t cm_shutdown(const stat_t status, const char *msg);       // enter shutdown state - dump all state
stat_t cm_panic(const stat_t status, const char *msg);          // enter panic state - needs RESET

// Representation (4.3.3)
stat_t cm_select_plane(const uint8_t plane);							        // G17, G18, G19
stat_t cm_set_units_mode(const uint8_t mode);							        // G20, G21
stat_t cm_set_distance_mode(const uint8_t mode);						        // G90, G91
stat_t cm_set_arc_distance_mode(const uint8_t mode);						    // G90, G91
stat_t cm_set_coord_offsets(const uint8_t coord_system,                         // G10
                            const uint8_t L_word,
                            const float offset[], const bool flag[]);

void cm_set_position(const uint8_t axis, const float position);				    // set absolute position - single axis
stat_t cm_set_absolute_origin(const float origin[], bool flags[]);	            // G28.3
void cm_set_axis_origin(uint8_t axis, const float position);	                // G28.3 planner callback

stat_t cm_set_coord_system(const uint8_t coord_system);				            // G54 - G59
stat_t cm_set_origin_offsets(const float offset[], bool flags[]);		        // G92
stat_t cm_reset_origin_offsets(void); 							                // G92.1
stat_t cm_suspend_origin_offsets(void); 						                // G92.2
stat_t cm_resume_origin_offsets(void);				 			                // G92.3

// Free Space Motion (4.3.4)
stat_t cm_straight_traverse(float target[], bool flags[]);		                // G0
stat_t cm_set_g28_position(void);								                // G28.1
stat_t cm_goto_g28_position(float target[], bool flags[]); 	                    // G28
stat_t cm_set_g30_position(void);								                // G30.1
stat_t cm_goto_g30_position(float target[], bool flags[]);		                // G30

// Machining Attributes (4.3.5)
stat_t cm_set_feed_rate(float feed_rate);						                // F parameter
stat_t cm_set_feed_rate_mode(uint8_t mode);						                // G93, G94, (G95 unimplemented)
stat_t cm_set_path_control(GCodeState_t *gcode_state, const uint8_t mode);      // G61, G61.1, G64

// Machining Functions (4.3.6)
stat_t cm_straight_feed(float target[], bool flags[]);		                    // G1
stat_t cm_dwell(float seconds);									                // G4, P parameter

stat_t cm_arc_feed(float target[], bool target_f[],                             // G2/G3 - target endpoint
                   float offset[], bool offset_f[],                             // IJK offsets
                   float radius,   bool radius_f,                               // radius if radius mode
                   float P_word,   bool P_word_f,                               // parameter
                   const bool modal_g1_f,                                       // modal group flag for motion group
                   const uint8_t motion_mode);                                  // defined motion mode

// Spindle Functions (4.3.7)
// see spindle.h for spindle definitions - which would go right here

// Tool Functions (4.3.8)
stat_t cm_select_tool(uint8_t tool);							// T parameter
stat_t cm_change_tool(uint8_t tool);							// M6

// Miscellaneous Functions (4.3.9)
stat_t cm_mist_coolant_control(uint8_t mist_coolant); 			// M7
stat_t cm_flood_coolant_control(uint8_t flood_coolant);			// M8, M9

stat_t cm_override_enables(uint8_t flag); 						// M48, M49
stat_t cm_feed_rate_override_enable(uint8_t flag); 				// M50
stat_t cm_feed_rate_override_factor(uint8_t flag);				// M50.1
stat_t cm_traverse_override_enable(uint8_t flag); 				// M50.2
stat_t cm_traverse_override_factor(uint8_t flag);				// M50.3
stat_t cm_spindle_override_enable(uint8_t flag); 				// M51
stat_t cm_spindle_override_factor(uint8_t flag);				// M51.1

void cm_message(char *message);								// msg to console (e.g. Gcode comments)

// Program Functions (4.3.10)
void cm_request_feedhold(void);
void cm_request_queue_flush(void);
void cm_request_cycle_start(void);

stat_t cm_feedhold_sequencing_callback(void);					// process feedhold, cycle start and queue flush requests
stat_t cm_queue_flush(void);									// flush serial and planner queues with coordinate resets

void cm_cycle_start(void);										// (no Gcode)
void cm_cycle_end(void); 										// (no Gcode)
void cm_canned_cycle_end(void);                                 // end of canned cycle
void cm_feedhold(void);											// (no Gcode)
void cm_program_stop(void);										// M0
void cm_optional_program_stop(void);							// M1
void cm_program_end(void);										// M2

/*--- Cycles ---*/

// Homing cycles
stat_t cm_homing_cycle_start(void);								// G28.2
stat_t cm_homing_cycle_start_no_set(void);						// G28.4
//stat_t cm_homing_callback(void);								// G28.2/.4 main loop callback
stat_t cm_homing_cycle_callback(void);                          // G28.2/.4 main loop callback

// Probe cycles
stat_t cm_straight_probe(float target[], bool flags[]);		    // G38.2
stat_t cm_probe_callback(void);									// G38.2 main loop callback

// Jogging cycle
stat_t cm_jogging_callback(void);								// jogging cycle main loop
stat_t cm_jogging_cycle_start(uint8_t axis);					// {"jogx":-100.3}
float cm_get_jogging_dest(void);

/*--- cfgArray interface functions ---*/

char cm_get_axis_char(const int8_t axis);

stat_t cm_get_mline(nvObj_t *nv);		// get model line number
stat_t cm_get_line(nvObj_t *nv);		// get active (model or runtime) line number
stat_t cm_get_stat(nvObj_t *nv);		// get combined machine state as value and string
stat_t cm_get_macs(nvObj_t *nv);		// get raw machine state as value and string
stat_t cm_get_cycs(nvObj_t *nv);		// get raw cycle state (etc etc)...
stat_t cm_get_mots(nvObj_t *nv);		// get raw motion state...
stat_t cm_get_hold(nvObj_t *nv);		// get raw hold state...
stat_t cm_get_home(nvObj_t *nv);		// get raw homing state...
stat_t cm_get_unit(nvObj_t *nv);		// get unit mode...
stat_t cm_get_coor(nvObj_t *nv);		// get coordinate system in effect...
stat_t cm_get_momo(nvObj_t *nv);		// get motion mode...
stat_t cm_get_plan(nvObj_t *nv);		// get active plane...
stat_t cm_get_path(nvObj_t *nv);		// get patch control mode...
stat_t cm_get_dist(nvObj_t *nv);		// get distance mode...
stat_t cm_get_admo(nvObj_t *nv);		// get arc offset mode...
stat_t cm_get_frmo(nvObj_t *nv);		// get feedrate mode...
stat_t cm_get_toolv(nvObj_t *nv);		// get tool (value)
stat_t cm_get_pwr(nvObj_t *nv);			// get motor power enable state

stat_t cm_get_vel(nvObj_t *nv);			// get runtime velocity...
stat_t cm_get_feed(nvObj_t *nv);
stat_t cm_get_pos(nvObj_t *nv);			// get runtime work position...
stat_t cm_get_mpo(nvObj_t *nv);			// get runtime machine position...
stat_t cm_get_ofs(nvObj_t *nv);			// get runtime work offset...

stat_t cm_run_qf(nvObj_t *nv);			// run queue flush
stat_t cm_run_home(nvObj_t *nv);		// start homing cycle

stat_t cm_dam(nvObj_t *nv);				// dump active model (debugging command)

stat_t cm_run_jogx(nvObj_t *nv);		// start jogging cycle for x
stat_t cm_run_jogy(nvObj_t *nv);		// start jogging cycle for y
stat_t cm_run_jogz(nvObj_t *nv);		// start jogging cycle for z
stat_t cm_run_joga(nvObj_t *nv);		// start jogging cycle for a

stat_t cm_get_am(nvObj_t *nv);			// get axis mode
stat_t cm_set_am(nvObj_t *nv);			// set axis mode
stat_t cm_set_hi(nvObj_t *nv);          // set homing input

//stat_t cm_set_ja(nvObj_t *nv);			// set junction aggression with 1,000,000 correction
//stat_t cm_set_vm(nvObj_t *nv);			// set velocity max and reciprocal
//stat_t cm_set_fr(nvObj_t *nv);			// set feedrate max and reciprocal
stat_t cm_set_jm(nvObj_t *nv);			// set jerk max with 1,000,000 correction
stat_t cm_set_jh(nvObj_t *nv);			// set jerk high with 1,000,000 correction

/*--- text_mode support functions ---*/

#ifdef __TEXT_MODE

	void cm_print_vel(nvObj_t *nv);		// model state reporting
	void cm_print_feed(nvObj_t *nv);
	void cm_print_line(nvObj_t *nv);
	void cm_print_stat(nvObj_t *nv);
	void cm_print_macs(nvObj_t *nv);
	void cm_print_cycs(nvObj_t *nv);
	void cm_print_mots(nvObj_t *nv);
	void cm_print_hold(nvObj_t *nv);
	void cm_print_home(nvObj_t *nv);
	void cm_print_hom(nvObj_t *nv);
	void cm_print_unit(nvObj_t *nv);
	void cm_print_coor(nvObj_t *nv);
	void cm_print_momo(nvObj_t *nv);
	void cm_print_plan(nvObj_t *nv);
	void cm_print_path(nvObj_t *nv);
	void cm_print_dist(nvObj_t *nv);
	void cm_print_admo(nvObj_t *nv);
	void cm_print_frmo(nvObj_t *nv);
	void cm_print_tool(nvObj_t *nv);
	void cm_print_g92e(nvObj_t *nv);

	void cm_print_gpl(nvObj_t *nv);		// Gcode defaults
	void cm_print_gun(nvObj_t *nv);
	void cm_print_gco(nvObj_t *nv);
	void cm_print_gpa(nvObj_t *nv);
	void cm_print_gdi(nvObj_t *nv);

	void cm_print_lin(nvObj_t *nv);		// generic print for linear values
	void cm_print_pos(nvObj_t *nv);		// print runtime work position in prevailing units
	void cm_print_mpo(nvObj_t *nv);		// print runtime work position always in MM units
	void cm_print_ofs(nvObj_t *nv);		// print runtime work offset always in MM units

	void cm_print_ja(nvObj_t *nv);		// global CM settings
	void cm_print_ct(nvObj_t *nv);
	void cm_print_sl(nvObj_t *nv);
//	void cm_print_ml(nvObj_t *nv);
//	void cm_print_ma(nvObj_t *nv);
//	void cm_print_ms(nvObj_t *nv);
//	void cm_print_st(nvObj_t *nv);
	void cm_print_lim(nvObj_t *nv);
	void cm_print_saf(nvObj_t *nv);

	void cm_print_am(nvObj_t *nv);		// axis print functions
	void cm_print_fr(nvObj_t *nv);
	void cm_print_vm(nvObj_t *nv);
	void cm_print_tm(nvObj_t *nv);
	void cm_print_tn(nvObj_t *nv);
	void cm_print_jm(nvObj_t *nv);
	void cm_print_jh(nvObj_t *nv);
	void cm_print_jd(nvObj_t *nv);
	void cm_print_ra(nvObj_t *nv);
    
	void cm_print_hi(nvObj_t *nv);
	void cm_print_hd(nvObj_t *nv);
	void cm_print_sn(nvObj_t *nv);
	void cm_print_sx(nvObj_t *nv);
	void cm_print_sv(nvObj_t *nv);
	void cm_print_lv(nvObj_t *nv);
	void cm_print_lb(nvObj_t *nv);
	void cm_print_zb(nvObj_t *nv);
	void cm_print_cofs(nvObj_t *nv);
	void cm_print_cpos(nvObj_t *nv);

#else // __TEXT_MODE

	#define cm_print_vel tx_print_stub		// model state reporting
	#define cm_print_feed tx_print_stub
	#define cm_print_line tx_print_stub
	#define cm_print_stat tx_print_stub
	#define cm_print_macs tx_print_stub
	#define cm_print_cycs tx_print_stub
	#define cm_print_mots tx_print_stub
	#define cm_print_hold tx_print_stub
	#define cm_print_home tx_print_stub
	#define cm_print_hom tx_print_stub
	#define cm_print_unit tx_print_stub
	#define cm_print_coor tx_print_stub
	#define cm_print_momo tx_print_stub
	#define cm_print_plan tx_print_stub
	#define cm_print_path tx_print_stub
	#define cm_print_dist tx_print_stub
	#define cm_print_admo tx_print_stub
	#define cm_print_frmo tx_print_stub
	#define cm_print_tool tx_print_stub
	#define cm_print_g92e tx_print_stub

	#define cm_print_gpl tx_print_stub		// Gcode defaults
	#define cm_print_gun tx_print_stub
	#define cm_print_gco tx_print_stub
	#define cm_print_gpa tx_print_stub
	#define cm_print_gdi tx_print_stub

	#define cm_print_lin tx_print_stub		// generic print for linear values
	#define cm_print_pos tx_print_stub		// print runtime work position in prevailing units
	#define cm_print_mpo tx_print_stub		// print runtime work position always in MM uints
	#define cm_print_ofs tx_print_stub		// print runtime work offset always in MM uints

	#define cm_print_ja tx_print_stub		// global CM settings
	#define cm_print_ct tx_print_stub
	#define cm_print_sl tx_print_stub
//	#define cm_print_ml tx_print_stub
//	#define cm_print_ma tx_print_stub
//	#define cm_print_ms tx_print_stub
//	#define cm_print_st tx_print_stub
	#define cm_print_lim tx_print_stub
	#define cm_print_saf tx_print_stub

	#define cm_print_am tx_print_stub		// axis print functions
	#define cm_print_fr tx_print_stub
	#define cm_print_vm tx_print_stub
	#define cm_print_tm tx_print_stub
	#define cm_print_tn tx_print_stub
	#define cm_print_jm tx_print_stub
	#define cm_print_jh tx_print_stub
	#define cm_print_jd tx_print_stub
	#define cm_print_ra tx_print_stub
    
	#define cm_print_hi tx_print_stub
	#define cm_print_hd tx_print_stub
	#define cm_print_sn tx_print_stub
	#define cm_print_sx tx_print_stub
	#define cm_print_sv tx_print_stub
	#define cm_print_lv tx_print_stub
	#define cm_print_lb tx_print_stub
	#define cm_print_zb tx_print_stub
	#define cm_print_cofs tx_print_stub
	#define cm_print_cpos tx_print_stub

#endif // __TEXT_MODE

#endif // End of include guard: CANONICAL_MACHINE_H_ONCE
