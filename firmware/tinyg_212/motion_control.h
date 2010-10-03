/*
 * motion_control.h - cartesian robot controller.
 * Part of TinyG
 *
 * Copyright (c) 2009 Simen Svale Skogsrud
 * Portions copyright (c) 2010 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, or 
 * (at your (option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef motion_control_h
#define motion_control_h 

#include <string.h>			// needed for memset in clear_vector()

// used in motion_control.c & gcode.c
#define clear_vector(a) memset(a,0,sizeof(a)) 

/*
 * Global Scope Functions
 */

void mc_init(void);

int mc_set_position(double x, double y, double z, double a);
int mc_async_stop(void);
int mc_async_start(void);
int mc_async_end(void);

int mc_queued_stop(void);
int mc_queued_start(void);
int mc_queued_end(void);
int mc_queued_start_stop_continue();

//int mc_line(double x, double y, double z, double a, double feed_rate, uint8_t invert_feed_rate);
int mc_line(double x, double y, double z, double a, double feed_rate);
int mc_line_continue(void);

int mc_dwell(double seconds);
int mc_dwell_continue(void);

int mc_arc(double theta, double angular_travel, 
		   double radius, double linear_travel, 
		   uint8_t axis_1, uint8_t axis_2, uint8_t axis_linear, 
		   double feed_rate, uint8_t invert_feed_rate);
int mc_arc_continue();

int mc_go_home_cycle(void);

#endif
