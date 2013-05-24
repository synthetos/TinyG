/*
 * plan_arc.h - arc planning and motion execution
 * Part of TinyG project
 *
 * Copyright (c) 2011 - 2013 Alden S. Hart Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
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
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef plan_arc_h
#define plan_arc_h 

// See planner.h for MM_PER_ARC_SEGMENT setting

typedef struct arArcSingleton {			// persistent planner and runtime variables
	float magic_start;
	uint8_t run_state;			// runtime state machine sequence
	uint32_t linenum;			// line number of the arc feed move (Nxxxxx)
	uint32_t lineindex;			// line index of the arc feed move (autoincrement)
	
	float endpoint[AXES];		// endpoint position
	float position[AXES];		// accumulating runtime position
	float target[AXES];		// runtime target position
	float work_offset[AXES];	// offset from machine coord system for reporting

	float length;				// length of line or helix in mm
	float time;				// total running time (derived)
	float min_time;			// not sure this is needed
	float theta;				// total angle specified by arc
	float radius;				// computed via offsets
	float angular_travel;		// travel along the arc
	float linear_travel;		// travel along linear axis of arc
	uint8_t axis_1;				// arc plane axis
	uint8_t axis_2;				// arc plane axis
	uint8_t axis_linear;		// transverse axis (helical)

	float segments;			// number of segments in arc or blend
	int32_t segment_count;		// count of running segments
	float segment_time;		// constant time per aline segment
	float segment_theta;		// angular motion per segment
	float segment_linear_travel;// linear motion per segment
	float center_1;			// center of circle at axis 1 (typ X)
	float center_2;			// center of circle at axis 2 (typ Y)
	float magic_end;
} arc_t;
arc_t ar;

// function prototypes
stat_t ar_arc(	const float target[],
				const float i, const float j, const float k, 
				const float theta, 
				const float radius, 
		   		const float angular_travel, 
				const float linear_travel, 
		   		const uint8_t axis_1, 
				const uint8_t axis_2, 
				const uint8_t axis_linear,
				const float minutes,
				const float work_offset[],
				const float min_time);

stat_t ar_arc_callback(void);
void ar_abort_arc(void);

#endif
