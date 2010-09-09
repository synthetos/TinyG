/*
  motion_control.c - cartesian robot controller.
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

--- TinyG Notes:
   Modified for TinyG project by Alden Hart - May 2010
   Modified to support Xmega family processors
   Moved stuff into global structs
*/

#include "xmega_support.h"	// should be first

#include <stdlib.h>
#include <math.h>
#include <util/delay.h>
#include <avr/io.h>

#include "tinyg.h"
#include "config.h"
#include "motion_control.h"
#include "stepper.h"

struct MotionControlState {
	int32_t position[3];    		// current position of the tool in absolute steps
	int32_t target[3];				// target position of the tool in absolute steps
	int32_t steps[3];				// target line in relative steps
	double millimeters_of_travel;	// 

	double theta;
	double radius;
	double angular_travel;
	double linear_travel;
	double feed_rate;
	int invert_feed_rate;
	double dtarget[3];				// target position in floating point
	double center_x;				// center of this circle
	double center_y;				// center of this circle
	uint16_t segments;				// number of segments in arc
	double theta_per_segment;		// angular motion per segment
	double linear_per_segment;		// linear motion per segment
};

static struct MotionControlState mc;


/* 
 * mc_init() 
 */

void mc_init()
{
	clear_vector(mc.position);
}

/* 
 * mc_dwell() 
 */

void mc_dwell(uint32_t milliseconds) 
{
	st_synchronize();
	_delay_ms(milliseconds);
}

/* mc_line()
 *
 *	Execute linear motion in absolute millimeter coordinates. 
 *	Feed rate given in millimeters/second unless invert_feed_rate is true. 
 *	Then the feed_rate means that the motion should be completed in 1/feed_rate minutes.
 */

void mc_line(double x, double y, double z, float feed_rate, int invert_feed_rate)
{
	mc.target[X_AXIS] = lround(x*cfg.steps_per_mm[X_AXIS]);
	mc.target[Y_AXIS] = lround(y*cfg.steps_per_mm[Y_AXIS]);
	mc.target[Z_AXIS] = lround(z*cfg.steps_per_mm[Z_AXIS]); 

	for(int i = X_AXIS; i <= Z_AXIS; i++) {
		mc.steps[i] = mc.target[i]-mc.position[i];
	}
  
	if (invert_feed_rate) {
    	st_buffer_line(mc.steps[X_AXIS], mc.steps[Y_AXIS], mc.steps[Z_AXIS], 
			lround(ONE_MINUTE_OF_MICROSECONDS/feed_rate));
	} else {  // Ask old Phythagoras to estimate how many mm our next move is going to take us
 		mc.millimeters_of_travel = sqrt(square(mc.steps[X_AXIS]/cfg.steps_per_mm[X_AXIS]) + 
									    square(mc.steps[Y_AXIS]/cfg.steps_per_mm[Y_AXIS]) + 
									    square(mc.steps[Z_AXIS]/cfg.steps_per_mm[Z_AXIS]));
		st_buffer_line(mc.steps[X_AXIS], mc.steps[Y_AXIS], mc.steps[Z_AXIS], 
			lround((mc.millimeters_of_travel/feed_rate)*1000000));
	}
	memcpy(mc.position, mc.target, sizeof(mc.target)); 	// needed for arc motion 
}

/* mc_arc() - execute an arc 
 *
 *	Theta = start angle 
 *	Angular_travel = number of radians to go along the arc
 *	Positive angular_travel means clockwise, negative means counterclockwise. 
 *	Radius = the radius of the circle in millimeters. 
 *	Axis_1 and axis_2 selects the circle plane in tool space. 
 *	Stick the remaining axis in axis_l which will be the axis for linear travel if you are tracing a helical motion.
 *
 *	The arc is approximated by generating a huge number of tiny, linear segments. 
 *	The length of each segment is configured in config.h by setting MM_PER_ARC_SEGMENT.  
 */

void mc_arc(double theta, double angular_travel, double radius, double linear_travel, int axis_1, int axis_2, 
  int axis_linear, double feed_rate, int invert_feed_rate)
{
	// load up the struct so the variable shows up in Simulator2 watch screen
	mc.theta = theta;
	mc.radius = radius;
	mc.angular_travel = angular_travel;
	mc.linear_travel = linear_travel;
	mc.feed_rate = feed_rate;
	mc.invert_feed_rate = invert_feed_rate;

	mc.millimeters_of_travel = hypot(mc.angular_travel*mc.radius, labs(mc.linear_travel));
	
	if (mc.millimeters_of_travel == 0.0) { 
		return; 
	}
	mc.segments = ceil(mc.millimeters_of_travel/cfg.mm_per_arc_segment);
  
  	/*  Multiply inverse feed_rate to compensate for the fact that this movement 
		is approximated by a number of discrete segments. 
		The inverse feed_rate should be correct for the sum of all segments.*/

	if (mc.invert_feed_rate) { 
		mc.feed_rate *= mc.segments; 
	}
  
	mc.theta_per_segment = mc.angular_travel/mc.segments;
	mc.linear_per_segment = mc.linear_travel/mc.segments;
	
	mc.center_x = (mc.position[axis_1]/cfg.steps_per_mm[axis_1])-sin(mc.theta)*mc.radius;
	mc.center_y = (mc.position[axis_2]/cfg.steps_per_mm[axis_2])-cos(mc.theta)*mc.radius;

  	/* 	A vector to track the end point of each segment
		Initialize the linear axis */
	
	mc.dtarget[axis_linear] = mc.position[axis_linear]/Z_STEPS_PER_MM;
  	for (int i=0; i<=mc.segments; i++) {
		mc.dtarget[axis_linear] += mc.linear_per_segment;
		mc.theta += mc.theta_per_segment;
		mc.dtarget[axis_1] = mc.center_x+sin(mc.theta)*mc.radius;
		mc.dtarget[axis_2] = mc.center_y+cos(mc.theta)*mc.radius;
		mc_line(mc.dtarget[X_AXIS], mc.dtarget[Y_AXIS], mc.dtarget[Z_AXIS], mc.feed_rate, mc.invert_feed_rate);
  	}
}

/* 
 * mc_go_home()  (st_go_home is NOT IMPLEMENTED)
 */

void mc_go_home()
{
	st_go_home();
	clear_vector(mc.position); // By definition this is location [0, 0, 0]
}
