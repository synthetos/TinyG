/*
 * motion_control.c - cartesian robot controller.
 * Part of Grbl
 *
 * Copyright (c) 2009 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify it under the 
 * termsof the GNU General Public License as published by the Free Software 
 * Foundation, either version 3 of the License, or (at your option) any later 
 * version.
 *
 * Grbl is distributed in the hope that it will be useful, but WITHOUT ANY 
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
 * FOR A PARTICULAR PURPOSE. See GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along 
 * with Grbl. If not, see <http://www.gnu.org/licenses/>.
 */
/* --- TinyG Notes ----
 *
 *  Modified to support Xmega family processors (Alden Hart, 2010)
 *  Introduced non-blocking line and arc generation to support multitasking
 *  Organized variabales into static structs
 *	Blocking versions of mc_line and mc_arc have been removed as of build 209
 *	There's a long discussion of canonical machining funcs at end of the .h
 */

#include "xmega_init.h"				// put before <util/delay.h>
#include <util/delay.h>				// needed for dwell
#include <stdlib.h>
#include <math.h>
#include <avr/io.h>

#include "tinyg.h"
#include "config.h"
#include "motion_control.h"
#include "move_buffer.h"
#include "stepper.h"

/*
 * Local Scope Data and Functions
 */

enum mcGeneratorState {
	MC_STATE_OFF,					// generator is OFF
	MC_STATE_NEW,					// initial call to generator
	MC_STATE_RUNNING,				// in process, needs re-entry (continuation)
	MC_STATE_MAX
};

struct MotionControlState {			// robot pos & vars used by lines and arcs
	uint8_t line_state;				// line generator state. mc_line_continue()
	uint8_t move_type;				// type of move. See mvType
	int32_t position[3];    		// current position of the tool in abs steps
	int32_t target[3];				// target position of the tool in abs steps
	int32_t steps[3];				// target line in relative steps
	uint32_t microseconds;			// target move microseconds
	double mm_of_travel;			// different than ma.mm_of_travel
};
static struct MotionControlState mc;

struct MotionControlArc {			// vars used by arc generation & continue
	int8_t arc_state;				// arc generator state. mc_arc_continue()
	int segments;					// number of segments in arc
	int	segment_counter;			// number of segments queued so far
	int invert_feed_rate;
	int axis_1;
	int axis_2;
	int axis_linear;

	double dtarget[3];				// target position in floating point
	double mm_of_travel;			// different than mc.mm_of_travel
	double center_x;				// center of this circle
	double center_y;				// center of this circle

	double theta;
	double radius;
	double feed_rate;
	double theta_per_segment;		// angular motion per segment
	double linear_per_segment;		// linear motion per segment
	double angular_travel;
	double linear_travel;
};
struct MotionControlArc ma;

/* 
 * mc_init() 
 */

void mc_init()
{
	clear_vector(mc.position);		// zero robot position
	mc.line_state = MC_STATE_OFF;	// turn off the generators
	ma.arc_state = MC_STATE_OFF;
}

/* 
 * mc_motion_start() - (re)start motion
 */

int mc_motion_start()
{
	return (TG_OK);
}

/* 
 * mc_motion_stop() - stop current motion immediately
 */

int mc_motion_stop()
{
	st_stop();						// stop the steppers
	return (TG_OK);
}

/* 
 * mc_motion_end() - end current motion and program
 *
 *	Should do all the following things (from NIST RS274NG_3 are marked RS274)
 * 	Those we don't care about are in [brackets]
 *
 *	- Stop all motion once current block is complete 
 *		(as opposed to kill, which stops immediately)
 *	- Axes is set to zero (like G92)
 * 	- Selected plane is set to CANON_PLANE_XY (like G17).
 *	- Distance mode is set to MODE_ABSOLUTE (like G90).
 *	- Feed rate mode is set to UNITS_PER_MINUTE (like G94).
 * 	- [Feed and speed overrides are set to ON (like M48).
 *	- [Cutter compensation is turned off (like G40).
 *	- The spindle is stopped (like M5).
 *	- The current motion mode is set to G1
 *	- Coolant is turned off (like M9).
 */

int mc_motion_end()
{
	mc_motion_stop();				// first actually stop the motion
	mc.line_state = MC_STATE_OFF;	// turn off the generators
	ma.arc_state = MC_STATE_OFF;
	mv_flush();						// empty and reset the move queue
	return (TG_OK);
}

/* 
 * mc_set_position() - set current position (support for G92)
 */

int mc_set_position(double x, double y, double z)
{
	mc.position[X] = lround(x*CFG(X).steps_per_mm);
	mc.position[Y] = lround(y*CFG(Y).steps_per_mm);
	mc.position[Z] = lround(z*CFG(Z).steps_per_mm); 
	return (TG_OK);
}

/* 
 * mc_line() - queue a line move; non-blocking version
 *
 * Compute and post a line segment to the move buffer.
 * Execute linear motion in absolute millimeter coordinates. 
 * Feed rate given in millimeters/second unless invert_feed_rate is true. 
 * Then the feed_rate means that the motion should be completed in 
 *	 1/feed_rate minutes
 *
 * Zero length lines are skipped at this level. 
 * Zero length lines that are actually dwells come in thru mc_dwell().
 * The mv_queue doesn't check line length and queues anything.
 */

int mc_line(double x, double y, double z, double feed_rate, int invert_feed_rate)
{
	mc.target[X] = lround(x*CFG(X).steps_per_mm);
	mc.target[Y] = lround(y*CFG(Y).steps_per_mm);
	mc.target[Z] = lround(z*CFG(Z).steps_per_mm); 

	mc.steps[X] = mc.target[X]-mc.position[X];
	mc.steps[Y] = mc.target[Y]-mc.position[Y];
	mc.steps[Z] = mc.target[Z]-mc.position[Z];

	// skip zero length lines
	if ((mc.steps[X] + mc.steps[Y] + mc.steps[Z]) == 0) {
		return (TG_ZERO_LENGTH_LINE);
	}

	if (invert_feed_rate) {
		mc.microseconds = lround(ONE_MINUTE_OF_MICROSECONDS/feed_rate);
	} else {  // Ask Phythagoras to estimate how many mm next move will take
 		mc.mm_of_travel = sqrt(square(mc.steps[X]/CFG(X).steps_per_mm) + 
							   square(mc.steps[Y]/CFG(Y).steps_per_mm) + 
							   square(mc.steps[Z]/CFG(Z).steps_per_mm));
		mc.microseconds = lround((mc.mm_of_travel/feed_rate)*1000000);
	}
	mc.move_type = MC_TYPE_LINE;
	mc.line_state = MC_STATE_NEW;
	memcpy(mc.position, mc.target, sizeof(mc.target)); 	// record new robot pos
	return (mc_line_continue());
}

/* 
 * mc_line_continue() - continuation to generate and load a linear move
 *
 *	This is a line generator that can be called multiple times until it can 
 *	successfully load the line into the move buffer.
 *	The mc.move_type must be set before calling this routine
 */
int mc_line_continue() 
{
	if (mc.line_state == MC_STATE_OFF) {
		return (TG_NOOP);			  // return NULL for non-started line
	}
//	mc.line_state = MC_STATE_RUNNING; // technically correct but not needed
	if (mv_test_move_buffer_full()) { // this is where you would block
		return (TG_EAGAIN);
	} else {
		mv_queue_move_buffer(mc.steps[X], mc.steps[Y], mc.steps[Z], mc.microseconds, mc.move_type); 
		mc.line_state = MC_STATE_OFF;	  // line is done. turn generator off
		return (TG_OK);
	}
}

/* 
 * mc_dwell() - queue a dwell (non-blocking behavior)
 *
 * Dwells are performed by passing a dwell move to the stepper drivers.
 * A dewll is described as a zero lenegth line with a non-zero execution time.
 * Dwells look like any other line except they are flagged as dwell for queing.
 * The stepper driver sees this and times the move but does not send any pulses.
 * Uses the X axis as only the X axis knows how to deal with a dwell.
 * Dwells are queued as lines so the line continuation is used for non-blocking.
 *
 * NOTE: 
 *	It's not necessary to set the target as this is set correctly in the Gcode. 
 */

int mc_dwell(double seconds) 
{
	mc.steps[X] = 0;
	mc.steps[Y] = 0;
	mc.steps[Z] = 0;
	mc.mm_of_travel = 0;	// not actually used, but debug makes more sense
	mc.microseconds = trunc(seconds*1000000);
	mc.move_type = MC_TYPE_DWELL;
	mc.line_state = MC_STATE_NEW;
	return (mc_line_continue());
}

/* 
 * mc_arc() - execute an arc; non-blocking version
 *
 *	The arc is approximated by generating a huge number of tiny, linear segments. 
 *	The length of each segment is configured in config.h by setting 
 *	MM_PER_ARC_SEGMENT.  
 */

int mc_arc(double theta, 			// starting angle
		   double angular_travel, 	// radians to go along arc (+ CW, - CCW)
		   double radius, 			// radius of the circle in millimeters.
		   double linear_travel, 
		   int axis_1, 				// select circle plane in tool space
		   int axis_2,  			// select circle plane in tool space
		   int axis_linear, 		// linear travel if tracing a helical motion
		   double feed_rate, 		// feed rate
		   int invert_feed_rate)	// feed rate mode
{
	// load the move and arc structs
	mc.move_type = MC_TYPE_LINE;
	ma.theta = theta;
	ma.radius = radius;
	ma.angular_travel = angular_travel;
	ma.linear_travel = linear_travel;
	ma.feed_rate = feed_rate;
	ma.invert_feed_rate = invert_feed_rate;
	ma.axis_1 = axis_1;
	ma.axis_2 = axis_2;
	ma.axis_linear = axis_linear;
	ma.mm_of_travel = hypot(ma.angular_travel*ma.radius, labs(ma.linear_travel));
	
	if (ma.mm_of_travel < MM_PER_ARC_SEGMENT) { 	// too short to draw
		return (TG_ARC_SPECIFICATION_ERROR);
	}
	ma.segments = ceil(ma.mm_of_travel/cfg.mm_per_arc_segment);
  
  	/*  Multiply inverse feed_rate to compensate for the fact that this movement 
	 *	is approximated by a number of discrete segments. 
	 *	The inverse feed_rate should be correct for the sum of all segments.*/

	if (ma.invert_feed_rate) { 
		ma.feed_rate *= ma.segments; 
	}
	ma.theta_per_segment = ma.angular_travel/ma.segments;
	ma.linear_per_segment = ma.linear_travel/ma.segments;
	ma.center_x = (mc.position[ma.axis_1]/CFG(ma.axis_1).steps_per_mm)-sin(ma.theta)*ma.radius;
	ma.center_y = (mc.position[ma.axis_2]/CFG(ma.axis_2).steps_per_mm)-cos(ma.theta)*ma.radius;

  	// 	A vector to track the end point of each segment. Initialize linear axis
	ma.dtarget[ma.axis_linear] = mc.position[ma.axis_linear]/CFG(Z).steps_per_mm;
	ma.arc_state = MC_STATE_NEW;	// new arc, NJ. (I'm here all week Try veal)
	return (mc_arc_continue());
}

/* 
 * mc_arc_continue() - continuation inner loop to generate and load an arc move
 *
 * Generates the line segments in an arc and queues them to the move buffer.
 *
 * Operation: 
 *	This function is called initially by mc_arc_nonblock.
 * 	Function will either run to arc completion or the move buffer queue is full.
 * 	It can then be re-entered to generate and queue the next segment(s) of arc.
 * 	Calling this function when there is no arc in process has no effect (NOOP).
 *
 * Note on mv_test_move_buffer_full()
 *	The move buffer is tested and sometime later it's queued (via mc_line())
 *	This only works because no ISRs queue this buffer, and this continuation 
 *	routine cannot be pre-empted. If these conditions change you need to 
 *	implement a critical region or mutex of some sort.
 */
int mc_arc_continue() 
{
	if (ma.arc_state == MC_STATE_OFF) {
		return (TG_NOOP);					// return NULL for non-started arc
	} else if (ma.arc_state == MC_STATE_NEW) {
		ma.segment_counter=0;
		ma.arc_state = MC_STATE_RUNNING;
	}
	mc.move_type = MC_TYPE_LINE;
	while (ma.segment_counter <= ma.segments) {
		if (mv_test_move_buffer_full()) {	// this is where you would block
			return (TG_EAGAIN);
		}
		ma.segment_counter++;
		ma.theta += ma.theta_per_segment;
		ma.dtarget[ma.axis_1] = ma.center_x+sin(ma.theta)*ma.radius;
		ma.dtarget[ma.axis_2] = ma.center_y+cos(ma.theta)*ma.radius;
		ma.dtarget[ma.axis_linear] += ma.linear_per_segment;
		mc_line(ma.dtarget[X], ma.dtarget[Y], ma.dtarget[Z], ma.feed_rate, ma.invert_feed_rate);
  	}
	ma.arc_state = MC_STATE_OFF;		// arc is done. turn the generator off
	return (TG_OK);
}


/* 
 * mc_home()  (st_go_home is NOT IMPLEMENTED)
 */

int mc_home()
{
//	st_go_home();
	clear_vector(mc.position); // By definition this is location [0, 0, 0]
	return (TG_OK);
}
