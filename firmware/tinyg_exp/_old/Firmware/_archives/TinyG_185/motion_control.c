/*
 * motion_control.c - cartesian robot controller.
 * Part of Grbl
 *
 * Copyright (c) 2009 Simen Svale Skogsrud
 * Modified for TinyG project by Alden Hart 2010
 *
 * Grbl is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with Grbl.  
 * If not, see <http://www.gnu.org/licenses/>.
 */
/* --- TinyG Notes ----
 *
 *  Modified to support Xmega family processors
 *  Introduced non-blocking line and arc generation behaviors to support multitasking
 *  Organized variabales into static structs
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

struct MotionControlState {			// robot position & vars used by line and arc
	int8_t line_state;				// line generator state. See mc_line_generator()
	int32_t position[3];    		// current position of the tool in absolute steps
	int32_t target[3];				// target position of the tool in absolute steps
	int32_t steps[3];				// target line in relative steps
	uint32_t microseconds;			// target move microseconds
	double mm_of_travel;			// different than ma.mm_of_travel
};
static struct MotionControlState mc;

struct MotionControlArcState {		// vars used by arc generation & re-entrancy
	int8_t arc_state;				// arc generator state. See mc_arc_generator()
	int	i;							// arc index counter
	int segments;					// number of segments in arc
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
struct MotionControlArcState ma;

/* 
 * mc_init() 
 */

void mc_init()
{
	clear_vector(mc.position);
	mc.line_state = TG_OFF;			// turn off the generators
	ma.arc_state = TG_OFF;
}

/* mc_line()
 *
 *	Compute and post a line segment to the move buffer.
 *	This routine does not check 
 *	Execute linear motion in absolute millimeter coordinates. 
 *	Feed rate given in millimeters/second unless invert_feed_rate is true. 
 *	Then the feed_rate means that the motion should be completed in 1/feed_rate minutes.
 */

int mc_line(double x, double y, double z, double feed_rate, int invert_feed_rate)
{
	mc.target[X_AXIS] = lround(x*CFG(X_AXIS).steps_per_mm);
	mc.target[Y_AXIS] = lround(y*CFG(Y_AXIS).steps_per_mm);
	mc.target[Z_AXIS] = lround(z*CFG(Z_AXIS).steps_per_mm); 

	mc.steps[X_AXIS] = mc.target[X_AXIS]-mc.position[X_AXIS];
	mc.steps[Y_AXIS] = mc.target[Y_AXIS]-mc.position[Y_AXIS];
	mc.steps[Z_AXIS] = mc.target[Z_AXIS]-mc.position[Z_AXIS];

	if (invert_feed_rate) {
		mc.microseconds = lround(ONE_MINUTE_OF_MICROSECONDS/feed_rate);
	} else {  // Ask Phythagoras to estimate how many mm next move is going to take
 		mc.mm_of_travel = sqrt(square(mc.steps[X_AXIS]/CFG(X_AXIS).steps_per_mm) + 
							   square(mc.steps[Y_AXIS]/CFG(Y_AXIS).steps_per_mm) + 
							   square(mc.steps[Z_AXIS]/CFG(Z_AXIS).steps_per_mm));
		mc.microseconds = lround((mc.mm_of_travel/feed_rate)*1000000);
	}
    mv_queue_move_buffer(mc.steps[X_AXIS], mc.steps[Y_AXIS], mc.steps[Z_AXIS], mc.microseconds); 
    mv_queue_move_buffer2(mc.steps[X_AXIS], mc.steps[Y_AXIS], mc.steps[Z_AXIS], mc.microseconds); 

	memcpy(mc.position, mc.target, sizeof(mc.target)); 	// record new robot position
	return (TG_DONE);
}

/* 
 * mc_line_nonblock() - mc_line with non-blocking behaviors
 */

int mc_line_nonblock(double x, double y, double z, double feed_rate, int invert_feed_rate)
{
	mc.target[X_AXIS] = lround(x*CFG(X_AXIS).steps_per_mm);
	mc.target[Y_AXIS] = lround(y*CFG(Y_AXIS).steps_per_mm);
	mc.target[Z_AXIS] = lround(z*CFG(Z_AXIS).steps_per_mm); 

	mc.steps[X_AXIS] = mc.target[X_AXIS]-mc.position[X_AXIS];
	mc.steps[Y_AXIS] = mc.target[Y_AXIS]-mc.position[Y_AXIS];
	mc.steps[Z_AXIS] = mc.target[Z_AXIS]-mc.position[Z_AXIS];

	if (invert_feed_rate) {
		mc.microseconds = lround(ONE_MINUTE_OF_MICROSECONDS/feed_rate);
	} else {  // Ask Phythagoras to estimate how many mm next move is going to take
 		mc.mm_of_travel = sqrt(square(mc.steps[X_AXIS]/CFG(X_AXIS).steps_per_mm) + 
							   square(mc.steps[Y_AXIS]/CFG(Y_AXIS).steps_per_mm) + 
							   square(mc.steps[Z_AXIS]/CFG(Z_AXIS).steps_per_mm));
		mc.microseconds = lround((mc.mm_of_travel/feed_rate)*1000000);
	}
	mc.line_state = TG_NEW;
	memcpy(mc.position, mc.target, sizeof(mc.target)); 	// record new robot position
	return (mc_line_continuation());
}

/* 
 * mc_line_continuation() - continuation to generate and load a linear move
 *
 *	This is a line generator that can be called multiple times until it can 
 *	successfully load the line into the buffer.
 *
 *	Returns:
 *		0	a new line can be started. 
 *		1 	the line is blocked and a new Gcode command should not be started
 */
int mc_line_continuation() 
{
	if (mc.line_state == TG_OFF) {
		return (TG_OFF);			// return for non-started line
	}
//	mc.line_state == TG_CONTINUE;
	if (mv_test_move_buffer_full()) { // this is where you would block
		return (TG_CONTINUE);
	}
//	mv_queue_move_buffer(mc.steps[X_AXIS], mc.steps[Y_AXIS], mc.steps[Z_AXIS], mc.microseconds); 
	mv_queue_move_buffer2(mc.steps[X_AXIS], mc.steps[Y_AXIS], mc.steps[Z_AXIS], mc.microseconds); 

	mc.line_state = TG_OFF;			// line is done. turn the generator off.
	return (TG_DONE);
}

/* mc_arc() - execute an arc 
 *
 *	Theta = start angle 
 *	Angular_travel = number of radians to go along the arc
 *		Positive angular_travel means clockwise, negative means counterclockwise. 
 *	Radius = the radius of the circle in millimeters. 
 *	Axis_1 and axis_2 selects the circle plane in tool space. 
 *	Stick the remaining axis in axis_l which will be the axis for linear travel 
 *		if you are tracing a helical motion.
 *
 *	The arc is approximated by generating a huge number of tiny, linear segments. 
 *	The length of each segment is configured in config.h by setting MM_PER_ARC_SEGMENT.  
 */

int mc_arc(double theta, double angular_travel, double radius, double linear_travel, 
	int axis_1, int axis_2, int axis_linear, double feed_rate, int invert_feed_rate)
{
	// load the arc struct
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
	
	if (ma.mm_of_travel == 0.0) { 
		return (TG_DONE); 
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

  	// 	A vector to track the end point of each segment. Initialize the linear axis
	ma.dtarget[ma.axis_linear] = mc.position[ma.axis_linear]/CFG(Z_AXIS).steps_per_mm;
	
	//	Generate and queue the line segments along the arc
	for (ma.i=0; ma.i<=ma.segments; ma.i++) {
		ma.theta += ma.theta_per_segment;
		ma.dtarget[ma.axis_1] = ma.center_x+sin(ma.theta)*ma.radius;
		ma.dtarget[ma.axis_2] = ma.center_y+cos(ma.theta)*ma.radius;
		ma.dtarget[ma.axis_linear] += ma.linear_per_segment;
		mc_line(ma.dtarget[X_AXIS], 
				ma.dtarget[Y_AXIS], 
				ma.dtarget[Z_AXIS], 
				ma.feed_rate, 
				ma.invert_feed_rate);
  	}
	return (TG_DONE);
}

/* 
 * mc_arc_nonblock() - execute an arc with non-blocking behaviors
 */

int mc_arc_nonblock(double theta, double angular_travel, 
					double radius, double linear_travel, 
					int axis_1, int axis_2, int axis_linear, 
					double feed_rate, int invert_feed_rate)
{
	// load the arc struct
	ma.arc_state = TG_NEW;		// new arc, NJ. (I'm here all week. Tyy the veal)
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
	
	if (ma.mm_of_travel == 0.0) { 
		return (TG_DONE); 
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

  	// 	A vector to track the end point of each segment. Initialize the linear axis
	ma.dtarget[ma.axis_linear] = mc.position[ma.axis_linear]/CFG(Z_AXIS).steps_per_mm;
	return (mc_arc_continuation());
}

/* 
 * mc_arc_continuation() - continuation inner loop to generate and load an arc move
 *
 * Operation: 
 *	Generates the line segments in an arc and queues them to the line buffer.
 *	This function is called initially by mc_arc_nonblock.
 * 	Function will either run to arc completion or the line buffer queue is full.
 * 	It can then be re-entered to generate and queue the next segment(s) of the arc.
 * 	Calling this function when there is no arc in process has no effect.
 *
 */
int mc_arc_continuation() 
{
	if (ma.arc_state == TG_OFF) {
		return (TG_OFF);
	} else if (ma.arc_state == TG_NEW) {
		ma.i=0;
		ma.arc_state = TG_CONTINUE;
	}
	while (ma.i <= ma.segments) {
		if (mv_test_move_buffer_full()) {		// this is where you would block
			return (TG_CONTINUE);
		}
		ma.i++;
		ma.theta += ma.theta_per_segment;
		ma.dtarget[ma.axis_1] = ma.center_x+sin(ma.theta)*ma.radius;
		ma.dtarget[ma.axis_2] = ma.center_y+cos(ma.theta)*ma.radius;
		ma.dtarget[ma.axis_linear] += ma.linear_per_segment;
		mc_line(ma.dtarget[X_AXIS], 
				ma.dtarget[Y_AXIS], 
				ma.dtarget[Z_AXIS], 
				ma.feed_rate, 
				ma.invert_feed_rate);
  	}
	ma.arc_state = TG_OFF;			// arc is done. turn the generator off.
	return (TG_DONE);
}

/* 
 * mc_dwell() 
 */

void mc_dwell(uint32_t milliseconds) 
{
	mv_synchronize();
	_delay_ms(milliseconds);
}

/* 
 * mc_go_home()  (st_go_home is NOT IMPLEMENTED)
 */

void mc_go_home()
{
	st_go_home();
	clear_vector(mc.position); // By definition this is location [0, 0, 0]
}
