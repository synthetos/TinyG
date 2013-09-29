/*
 * plan_arc.h - arc planning and motion execution
 * Part of TinyG project
 *
 * Copyright (c) 2011 - 2013 Alden S. Hart Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
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

#ifdef __cplusplus
extern "C"{
#endif

// See planner.h for MM_PER_ARC_SEGMENT setting

typedef struct arArcSingleton {	// persistent planner and runtime variables
	magic_t magic_start;
	uint8_t run_state;			// runtime state machine sequence

	float endpoint[AXES];		// arc endpoint position
	float position[AXES];		// accumulating runtime position

	float length;				// length of line or helix in mm
	float arc_time;				// total running time for arc (derived)
	float theta;				// total angle specified by arc
	float radius;				// computed via offsets
	float angular_travel;		// travel along the arc
	float linear_travel;		// travel along linear axis of arc
	uint8_t axis_1;				// arc plane axis
	uint8_t axis_2;				// arc plane axis
	uint8_t axis_linear;		// transverse axis (helical)

	float segments;				// number of segments in arc or blend
	int32_t segment_count;		// count of running segments
	float segment_theta;		// angular motion per segment
	float segment_linear_travel;// linear motion per segment
	float center_1;				// center of circle at axis 1 (typ X)
	float center_2;				// center of circle at axis 2 (typ Y)

	GCodeState_t gm;			// Gcode state struct is passed for each arc segment. Usage:
//	uint32_t linenum;			// line number of the arc feed move - same for each segment
//	float target[AXES];			// arc segment target
//	float work_offset[AXES];	// offset from machine coord system for reporting (same for each segment)
//	float move_time;			// segment_time: constant time per aline segment

	magic_t magic_end;
} arc_t;
extern arc_t ar;

// function prototypes

stat_t cm_arc(const GCodeState_t *gm,
			  const float i, 
			  const float j, 
			  const float k, 
			  const float theta,
			  const float radius,
			  const float angular_travel,
			  const float linear_travel, 
			  const uint8_t axis_1,
			  const uint8_t axis_2,
			  const uint8_t axis_linear);

stat_t cm_arc_callback(void);
void cm_abort_arc(void);

#ifdef __cplusplus
}
#endif

#endif	// End of include guard: PLAN_ARC_H_ONCE
