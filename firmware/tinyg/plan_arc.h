/*
 * plan_arc.h - arc planning and motion execution
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart, Jr.
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

// Arc radius tests. See http://linuxcnc.org/docs/html/gcode/g-code.html#gcode:g2-g3
// which reads: "When the arc is projected on the selected plane, the distance from
//               the current point to the center differs from the distance from the
//               end point to the center by more than (.05 inch/.5 mm)
//               OR ((.0005 inch/.005mm) AND .1% of radius)."
//
// LinuxCNC values
//#define ARC_RADIUS_ERROR_MAX    ((float)0.5)      // max allowable mm between start and end radius
//#define ARC_RADIUS_ERROR_MIN    ((float)0.005)    // min mm where 0.1% rule applies
//#define ARC_RADIUS_TOLERANCE    ((float)0.001)    // 0.1% radius variance test
//
// Relaxed values - some CAM packages do not meet the LinuxCNC tests (Vectric 2D)
#define ARC_RADIUS_ERROR_MAX    ((float)1.0)        // max allowable mm between start and end radius
#define ARC_RADIUS_ERROR_MIN    ((float)0.005)      // min mm where 0.1% rule applies
#define ARC_RADIUS_TOLERANCE    ((float)0.05)       // 0.1% radius variance test

// ArcSingleton
//
// Note that the arc singleton is used both for arc setup and for arc execution.
// This means that you cannot set up an arc while a previous arc is still running.
// To avoid this the controller will not initiate a new arc while a previous arc is 
// still active in the arc-callback. Once all lines have been let to the planner
// then the new arc can begin. This does not starve the planner as it is still 
// executing the last remaining lines from the previous arc.

typedef struct arArcSingleton {	    // persistent planner and runtime variables
	magic_t magic_start;
	uint8_t run_state;              // runtime state machine sequence

	float position[AXES];		    // accumulating runtime position
	float offset[3]; 	 		    // IJK offsets

	float length;				    // length of line or helix in mm
	float radius;				    // Raw R value, or computed via offsets
	float theta;				    // total angle specified by arc
	float angular_travel;		    // travel along the arc
    float planar_travel;
	float linear_travel;		    // travel along linear axis of arc
	bool full_circle;		        // set true if full circle arcs specified
	float rotations;			    // Number of full rotations for full circles (P value)

    cmAxes plane_axis_0;            // arc plane axis 0 - e.g. X for G17
    cmAxes plane_axis_1;            // arc plane axis 1 - e.g. Y for G17
    cmAxes linear_axis;             // linear axis (normal to plane)

    float segments;                 // number of segments in arc or blend
    int32_t segment_count;          // count of running segments
    float segment_theta;            // angular motion per segment
    float segment_linear_travel;    // linear motion per segment
    float center_0;                 // center of circle at plane axis 0 (e.g. X for G17)
    float center_1;                 // center of circle at plane axis 1 (e.g. Y for G17)

	GCodeState_t gm;			    // Gcode state struct is passed for each arc segment. Usage:

	magic_t magic_end;
} arc_t;
extern arc_t arc;


/* arc function prototypes */	// NOTE: See canonical_machine.h for cm_arc_feed() prototype

void cm_arc_init(void);
stat_t cm_arc_callback(void);
void cm_abort_arc(void);

stat_t cm_arc_feed( const float target[], const bool target_f[],             // G2/G3 - target endpoint
                    const float offset[], const bool offset_f[],             // IJK offsets
                    const float radius, const bool radius_f,                 // radius if radius mode
                    const float P_word, const bool P_word_f,                 // parameter
                    const bool modal_g1_f,                                   // modal group flag for motion group
                    const uint8_t motion_mode);                              // defined motion mode

#endif	// End of include guard: PLAN_ARC_H_ONCE
