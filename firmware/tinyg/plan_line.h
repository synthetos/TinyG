/*
 * plan_line.h - acceleration managed line planning and motion execution
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
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

#ifndef plan_line_h
#define plan_line_h 

// NOTE: function prototypes are in planner.h for ease of access by external files

/*
struct mpMoveRuntimeSingleton {	// persistent runtime variables
//	uint8_t (*run_move)(struct mpMoveRuntimeSingleton *m); // currently running move - left in for reference
	uint32_t linenum;			// runtime line/block number of BF being executed
	uint8_t move_state;			// state of the overall move
	uint8_t section_state;		// state within a move section

	double endpoint[AXES];		// final target for bf (used to correct rounding errors)
	double position[AXES];		// current move position
	double target[AXES];		// target move position
	double unit[AXES];			// unit vector for axis scaling & planning
	double work_offset[AXES];	// offset from the work coordinate system (for reporting only)

	double head_length;			// copies of bf variables of same name
	double body_length;
	double tail_length;
	double entry_velocity;
	double cruise_velocity;
	double exit_velocity;

	double length;				// length of line in mm
	double move_time;			// total running time (derived)
	double midpoint_velocity;	// velocity at accel/decel midpoint
	double jerk;				// max linear jerk

	double segments;			// number of segments in arc or blend
	uint32_t segment_count;		// count of running segments
	double segment_move_time;	// actual time increment per aline segment
	double microseconds;		// line or segment time in microseconds
	double segment_length;		// computed length for aline segment
	double segment_velocity;	// computed velocity for aline segment
	double forward_diff_1;      // forward difference level 1 (Acceleration)
	double forward_diff_2;      // forward difference level 2 (Jerk - constant)
//	double accel_time;			// total pseudo-time for acceleration calculation
//	double elapsed_accel_time;	// current running time for accel calculation
//	double midpoint_acceleration;//acceleration at the midpoint
//	double jerk_div2;			// max linear jerk divided by 2
//	double segment_accel_time;	// time increment for accel computation purposes
};

static struct mpMoveRuntimeSingleton mr;// static context for runtime
*/

#endif
