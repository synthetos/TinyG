/*
 * planner.h - cartesian trajectory planning and motion execution
 * Part of TinyG project
 *
 * Copyright (c) 2011 Alden S. Hart Jr.
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
 */

#ifndef planner_h
#define planner_h 

/*
 *	Most of these factors are the result of a lot of tweaking. 
 *	Change at your peril.
 */

#define MP_BUFFER_SIZE 64		// sub-move buffer pool (48 min, 255 max)
#define MP_BUFFERS_NEEDED 3		// write buffers needed for aline
#define MP_MAX_LOOKBACK_DEPTH (MP_BUFFER_SIZE/3)

//#define MIN_LINE_LENGTH 0.01	// mm - smallest complete line it can handle
#define MIN_LINE_LENGTH 0.03	// mm - smallest complete line it can handle
#define MIN_SEGMENT_LENGTH 0.03	// accel/decel segments - must be <= MIN_LINE_LENGTH
#define MM_PER_ARC_SEGMENT 0.03	// set to produce ~10 ms segments
#define MIN_SEGMENT_TIME 10000	// microseconds - 10 ms. works well
#define ONE_MINUTE_OF_MICROSECONDS 60000000

#define TANGENTIAL_JERK_UPPER_BOUNDARY 300	// mm/min
#define TANGENTIAL_JERK_LOWER_BOUNDARY 100	// mm/min

/*
 *	Useful macros
 */

#define clear_vector(a) memset(a,0,sizeof(a)) // used in motion_control.c & gcode.c
#define _mp_bump(a) ((a<MP_BUFFER_SIZE-1)?(a+1):0)	// buffer incr & wrap
#define uSec(a) (a * ONE_MINUTE_OF_MICROSECONDS)
#define MP_LINE(t,m) ((cfg.accel_enabled == TRUE) ? mp_aline(t,m) : mp_line(t,m))

/*
 * Global Scope Functions
 */

void mp_init(void);
uint8_t mp_move_dispatcher(void);
uint8_t mp_check_for_write_buffers(uint8_t count);

uint8_t mp_isbusy(void);
uint8_t mp_set_axis_position(const double position[]);
void mp_copy_vector(double dest[], const double src[], uint8_t length);
double mp_get_axis_vector_length(const double target[], const double position[]);

void mp_async_stop(void);
void mp_async_start(void);
void mp_async_end(void);
void mp_queued_stop(void);
void mp_queued_start(void);
void mp_queued_end(void);

uint8_t mp_dwell(const double seconds);
uint8_t mp_line(const double target[], const double minutes);
uint8_t mp_aline(const double target[], const double minutes);
uint8_t mp_arc(const double target[],
			   const double i, const double j, const double k, 
			   const double theta, 
			   const double radius, 
		   	   const double angular_travel, 
			   const double linear_travel, 
		   	   const uint8_t axis_1, 
			   const uint8_t axis_2, 
			   const uint8_t axis_linear,
			   const double minutes);

uint8_t mp_go_home_cycle(void);

void mp_unit_tests(void);

#endif
