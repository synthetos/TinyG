/*
 * motion_control.h - cartesian robot controller.
 * Part of Grbl
 *
 * Copyright (c) 2009 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef motion_control_h
#define motion_control_h 

enum mcType {			// types of moves that can be put in the move queue
	MC_TYPE_LINE,
	MC_TYPE_DWELL,
	MC_TYPE_START,
	MC_TYPE_STOP,
	MC_TYPE_END
};

#include <string.h>						// needed for memset in clear_vector()
#define clear_vector(a) memset(a,0,sizeof(a)) // used in motion_control.c & gcode.c

/*
 * Global Scope Functions
 */

void mc_init(void);
int mc_motion_start(void);
int mc_motion_stop(void);
int mc_motion_end(void);
int mc_dwell(double seconds);
int mc_set_position(double x, double y, double z);

int mc_line(double x, double y, double z, 
			double feed_rate, int invert_feed_rate);
int mc_line_continue();

int mc_arc(double theta, double angular_travel, 
		   double radius, double linear_travel, 
		   int axis_1, int axis_2, int axis_linear, 
		   double feed_rate, int invert_feed_rate);
int mc_arc_continue();

int mc_home(void);						// send tool home


/****** Canonical machining functions from RS274NGC_3 *****
 * full set and extensions
 * items with no corresponding G, M or other code are blank

mc_init_canon()						// init canonical machining functions
mc_end_canon()

mc_select_plane()					// G17/G18/G19 (steal from GC routine)
mc_set_origin_offsets(x,y,z)		// G92 (G10?)
mc_use_length_units(UNITS)			// G20/G21

mc_set_traverse_rate(rate)			// (no code, get from config)
mc_straight_traverse(x,y,z)			// G0

mc_set_feed_rate(rate)				// F parameter
mc_set_feed_reference()				// 
mc_set_motion_control_mode()		// G61/G61.1/G64
mc_start_speed_feed_synch()			// 
mc_stop_speed_feed_synch()			// 

mc_arc_feed()						// G2/G3
mc_dwell(seconds)					// G4, P parameter
mc_ellipse_feed()					// 
mc_stop()							// M0,M1
mc_straight_feed()					// G1
mc_straight_probe()					// G38.2

mc_orient_spindle(orientation, dir)	// 
mc_set_spindle_speed()				// S parameter
mc_spindle_retract()				// 
mc_spindle_retract_traverse()		// 
mc_start_spindle_clockwise()		// M3
mc_start_spindle_counterclockwise()	// M4
mc_stop_spindle_turning				// M5
mc_use_no_spindle_force()			// 
mc_use_no_spindle_torque()			// 
mc_use_spindle_force()				// 
mc_use_spindle_torque()				// 

mc_change_tool()					// M6, T parameter
mc_select_tool()					// T parameter
mc_use_tool_length_offset()			// 

mc_clamp_axis()						// 
mc_unclamp_axis()					// 

mc_comment(char *)
mc_message(char *)
mc_disable_feed_override()
mc_disable_speed_override()
mc_enable_feed_override()
mc_enable_speed_override()
mc_flood_off()						// M9 (flood and mist both off)
mc_flood_on()						// M8
mc_mist_off()						// M9 (flood and mist both off)
mc_mist_on()						// M7
mc_pallet_shuttle()
mc_through_tool_off()				// 
mc_through_tool_on()				// 
mc_turn_probe_off()
mc_turn_probe_on()

mc_optional_program_stop()			// M1
mc_program_stop()					// M0
mc_program_end()					// M2

mc_set_cutter_radius_compensation()	  // G41/G42
mc_start_cutter_radius_compensation() // G41/G42
mc_stop_cutter_radius_compensation()  // G40

// functions not in canonical set (extensions)
mc_start()							// (re)enables stepper timers
mc_return_to_home()					// G28 
mc_set_distance_mode()				// G90/G91 (absolute/incremental motion)
 */

/****** Canonical machining functions from RS274NGC_3 *****
 * supported functions and extensions

mc_init_canon()						// init canonical machining functions

mc_select_plane()					// G17/G18/G19 (steal from GC routine)
mc_set_origin_offsets(x,y,z)		// supported as limited G92 for zeroing
mc_use_length_units(UNITS)			// G20/G21

mc_set_traverse_rate(rate)			// (no code, get from config)
mc_straight_traverse(x,y,z)			// G0
mc_set_feed_rate(rate)				// F parameter

mc_arc_feed()						// G2/G3
mc_dwell(seconds)					// G4, P parameter
mc_straight_feed()					// G1

mc_set_spindle_speed()				// S parameter
mc_start_spindle_clockwise()		// M3
mc_start_spindle_counterclockwise()	// M4
mc_stop_spindle_turning				// M5

mc_change_tool()					// M6, T parameter
mc_select_tool()					// T parameter

mc_comment(char *)					// handled in gcode parser / normalization
mc_message(char *)					// handled in gcode parser / normalization

mc_optional_program_stop()			// M1
mc_program_stop()					// M0
mc_program_end()					// M2
mc_stop()							// used by M0,M1
mc_start()							// (re)enables stepper timers

mc_return_to_home()					// G28 
mc_set_distance_mode()				// G90/G91 (absolute/incremental motion)
 */

#endif
