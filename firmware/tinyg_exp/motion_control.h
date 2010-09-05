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
int mc_set_position(double x, double y, double z);
int mc_motion_start(void);
int mc_motion_stop(void);
int mc_motion_end(void);

int mc_start(void);
int mc_stop(void);
int mc_start_stop_continue();

int mc_line(double x, double y, double z, double feed_rate, uint8_t invert_feed_rate);
int mc_line_continue(void);

int mc_dwell(double seconds);
int mc_dwell_continue(void);

int mc_arc(double theta, double angular_travel, 
		   double radius, double linear_travel, 
		   uint8_t axis_1, uint8_t axis_2, uint8_t axis_linear, 
		   double feed_rate, uint8_t invert_feed_rate);
int mc_arc_continue();

int mc_go_home_cycle(void);


/****** Canonical machining functions from RS274NGC_3 *****
 * full set and extensions
 * items with no corresponding G, M or other code are blank

cm_init_canon()						// init canonical machining functions
cm_end_canon()

cm_select_plane()					// G17/G18/G19 (steal from GC routine)
cm_set_origin_offsets(x,y,z)		// G92 (G10?)
cm_use_length_units(UNITS)			// G20/G21

cm_set_traverse_rate(rate)			// (no code, get from config)
cm_straight_traverse(x,y,z)			// G0

cm_set_feed_rate(rate)				// F parameter
cm_set_feed_rate_mode(mode)			// extension for inverse feed rate
cm_set_feed_reference()				// 
cm_set_motion_control_mode()		// G61/G61.1/G64
cm_start_speed_feed_synch()			// 
cm_stop_speed_feed_synch()			// 

cm_arc_feed()						// G2/G3
cm_dwell(seconds)					// G4, P parameter
cm_ellipse_feed()					// 
cm_stop()							// M0,M1
cm_straight_feed()					// G1
cm_straight_probe()					// G38.2

cm_orient_spindle(orientation, dir)	// 
cm_set_spindle_speed()				// S parameter
cm_spindle_retract()				// 
cm_spindle_retract_traverse()		// 
cm_start_spindle_clockwise()		// M3
cm_start_spindle_counterclockwise()	// M4
cm_stop_spindle_turning				// M5
cm_use_no_spindle_force()			// 
cm_use_no_spindle_torque()			// 
cm_use_spindle_force()				// 
cm_use_spindle_torque()				// 

cm_change_tool()					// M6, T parameter
cm_select_tool()					// T parameter
cm_use_tool_length_offset()			// 

cm_clamp_axis()						// 
cm_unclamp_axis()					// 

cm_comment(char *)
cm_message(char *)
cm_disable_feed_override()
cm_disable_speed_override()
cm_enable_feed_override()
cm_enable_speed_override()
cm_flood_off()						// M9 (flood and mist both off)
cm_flood_on()						// M8
cm_mist_off()						// M9 (flood and mist both off)
cm_mist_on()						// M7
cm_pallet_shuttle()
cm_through_tool_off()				// 
cm_through_tool_on()				// 
cm_turn_probe_off()
cm_turn_probe_on()

cm_optional_program_stop()			// M1
cm_program_stop()					// M0
cm_program_end()					// M2

cm_set_cutter_radius_compensation()	  // G41/G42
cm_start_cutter_radius_compensation() // G41/G42
cm_stop_cutter_radius_compensation()  // G40

// functions not in canonical set (extensions)
cm_start()							// (re)enables stepper timers
cm_return_to_home()					// G28 
cm_set_distance_mode()				// G90/G91 (absolute/incremental motion)
 */

/****** Canonical machining functions from RS274NGC_3 *****
 * supported functions and extensions

cm_init_canon()						// init canonical machining functions

cm_select_plane()					// G17/G18/G19 (steal from GC routine)
cm_set_origin_offsets(x,y,z)		// supported as limited G92 for zeroing
cm_use_length_units(UNITS)			// G20/G21

cm_set_traverse_rate(rate)			// (no code, get from config)
cm_straight_traverse(x,y,z)			// G0
cm_set_feed_rate(rate)				// F parameter
cm_set_feed_rate_mode(mode)			// extension for inverse feed rate

cm_arc_feed()						// G2/G3
cm_dwell(seconds)					// G4, P parameter
cm_straight_feed()					// G1

cm_set_spindle_speed()				// S parameter
cm_start_spindle_clockwise()		// M3
cm_start_spindle_counterclockwise()	// M4
cm_stop_spindle_turning				// M5

cm_change_tool()					// M6, T parameter
cm_select_tool()					// T parameter

cm_comment(char *)					// handled in gcode parser / normalization
cm_message(char *)					// handled in gcode parser / normalization

cm_optional_program_stop()			// M1
cm_program_stop()					// M0
cm_program_end()					// M2
cm_stop()							// used by M0,M1
cm_start()							// (re)enables stepper timers

cm_return_to_home()					// G28 
cm_set_distance_mode()				// G90/G91 (absolute/incremental motion)
 */

#endif
