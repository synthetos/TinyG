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

#include <string.h>						// needed for memset in clear_vector()
#define clear_vector(a) memset(a,0,sizeof(a)) // used in motion_control.c & gcode.c

/*
 * Global Scope Functions
 */

void mc_init(void);
void mc_motion_stop(void);
int mc_dwell(double seconds);
void mc_go_home(void);						// Send the tool home

int mc_line(double x, double y, double z, 
			double feed_rate, int invert_feed_rate);

int mc_line_nonblock(double x, double y, double z, 
					 double feed_rate, int invert_feed_rate);

int mc_line_continuation();


int mc_arc(double theta, double angular_travel, 
		   double radius, double linear_travel,
		   int axis_1, int axis_2, int axis_linear, 
			double feed_rate, int invert_feed_rate);

int mc_arc_nonblock(double theta, double angular_travel, 
					double radius, double linear_travel, 
					int axis_1, int axis_2, int axis_linear, 
					double feed_rate, int invert_feed_rate);

int mc_arc_continuation();

#endif
