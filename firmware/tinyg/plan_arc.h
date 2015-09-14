/*
 * plan_arc.h - arc planning and motion execution
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef PLAN_ARC_H_ONCE
#define PLAN_ARC_H_ONCE

// Arc radius tests. See http://linuxcnc.org/docs/html/gcode/gcode.html#sec:G2-G3-Arc
//#define ARC_RADIUS_ERROR_MAX    ((float)0.5)        // max allowable mm between start and end radius
#define ARC_RADIUS_ERROR_MAX    ((float)1.0)        // max allowable mm between start and end radius
#define ARC_RADIUS_ERROR_MIN    ((float)0.005)      // min mm where 1% rule applies
#define ARC_RADIUS_TOLERANCE    ((float)0.001)      // 0.1% radius variance test

// See planner.h for MM_PER_ARC_SEGMENT and other arc setting #defines

typedef struct arArcSingleton {	    // persistent planner and runtime variables
	magic_t magic_start;
	uint8_t run_state;			    // runtime state machine sequence

	float position[AXES];		    // accumulating runtime position
	float offset[3]; 	 		    // IJK offsets

	float length;				    // length of line or helix in mm
	float theta;				    // total angle specified by arc
	float theta_end;
	float radius;				    // Raw R value, or computed via offsets
	float angular_travel;		    // travel along the arc
	float linear_travel;		    // travel along linear axis of arc
    float planar_travel;
	uint8_t full_circle;		    // set true if full circle arcs specified
	uint32_t rotations;			    // Number of full rotations for full circles (P value)

	uint8_t plane_axis_0;           // arc plane axis 0 - e.g. X for G17
	uint8_t plane_axis_1;           // arc plane axis 1 - e.g. Y for G17
	uint8_t linear_axis;            // linear axis (normal to plane)

	float arc_time;					// total running time for arc (derived)
	float arc_segments;				// number of segments in arc or blend
	int32_t arc_segment_count;		// count of running segments
	float arc_segment_theta;		// angular motion per segment
	float arc_segment_linear_travel;// linear motion per segment
	float center_0;				    // center of circle at plane axis 0 (e.g. X for G17)
	float center_1;				    // center of circle at plane axis 1 (e.g. Y for G17)

	GCodeState_t gm;			    // Gcode state struct is passed for each arc segment. Usage:
//	uint32_t linenum;			    // line number of the arc feed move - same for each segment
//	float target[AXES];			    // arc segment target
//	float work_offset[AXES];	    // offset from machine coord system for reporting (same for each segment)
//	float move_time;			    // segment_time: constant time per aline segment

	magic_t magic_end;
} arc_t;
extern arc_t arc;


/* arc function prototypes */	// NOTE: See canonical_machine.h for cm_arc_feed() prototype

void cm_arc_init(void);
stat_t cm_arc_callback(void);
void cm_abort_arc(void);

#endif	// End of include guard: PLAN_ARC_H_ONCE
