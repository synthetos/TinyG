/*
 * planner.h - cartesian trajectory planning and motion execution
 * Part of TinyG project
 *
 * Copyright (c) 2011 Alden S. Hart Jr.
 * Portions copyright (c) 2009 Simen Svale Skogsrud
 *
 * TinyG is free software: you can redistribute it and/or modify
 * it under the terms of the Creative Commons CC-BY-NC license 
 * (Creative Commons Attribution Non-Commercial Share-Alike license)
 * as published by Creative Commons. You should have received a copy 
 * of the Creative Commons CC-BY-NC license along with TinyG.
 * If not see http://creativecommons.org/licenses/
 *
 * TinyG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 */

#ifndef planner_h
#define planner_h 

#include <string.h>				// needed for memset in clear_vector()

//#define MC_LINE mp_line
#define MC_LINE mp_aline

/*
 *	Most of these factors are the result of a lot of tweaking. 
 *	Change at your peril.
 */
//#define MP_BUFFER_SIZE 12		// sub-move buffer pool (48 min, 255 max)
#define MP_BUFFER_SIZE 48		// sub-move buffer pool (48 min, 255 max)
#define MP_BUFFERS_NEEDED 3		// write buffers needed for aline

//#define MIN_LINE_LENGTH 0.01	// mm - smallest complete line it can handle
#define MIN_LINE_LENGTH 0.03	// mm - smallest complete line it can handle
#define MIN_SEGMENT_LENGTH 0.03	// accel/decel segments - must be <= MIN_LINE_LENGTH
#define MM_PER_ARC_SEGMENT 0.03	// set to produce ~10 ms segments
#define MIN_SEGMENT_TIME 10000	// microseconds - 10 ms. works well

#define _mp_bump(a) ((a<MP_BUFFER_SIZE-1)?(a+1):0)	// buffer incr & wrap
#define _steps(x,a) round(a * CFG(x).steps_per_unit)

#define clear_vector(a) memset(a,0,sizeof(a)) // used in motion_control.c & gcode.c
#define uSec(a)	(round(a * ONE_MINUTE_OF_MICROSECONDS))

/*
 * Global Scope Functions
 */

void mp_init(void);
//uint8_t mp_move_dispatcher(void);
uint8_t mp_move_dispatcher(uint8_t kill);

uint8_t mp_test_write_buffer(uint8_t count);
uint8_t mp_queue_write_buffer(uint8_t move_type);
uint8_t mp_end_run_buffer(void);
void mp_unget_write_buffer(void);
struct mpBuffer * mp_get_write_buffer(void); 
struct mpBuffer * mp_get_run_buffer(void);
struct mpBuffer * mp_get_prev_buffer_implicit(void);
struct mpBuffer * mp_get_prev_buffer(struct mpBuffer *b);
struct mpBuffer * mp_get_next_buffer(struct mpBuffer *b);
struct mpBuffer * mp_clear_buffer(struct mpBuffer *b); 

uint8_t mp_isbusy(void);
uint8_t mp_set_position(double x, double y, double z, double a);
uint8_t mp_async_stop(void);
uint8_t mp_async_start(void);
uint8_t mp_async_end(void);

uint8_t mp_queued_stop(void);
uint8_t mp_queued_start(void);
uint8_t mp_queued_end(void);

uint8_t mp_dwell(double seconds);
uint8_t mp_line(double x, double y, double z, double a, double minutes);
uint8_t mp_aline(double x, double y, double z, double a, double minutes);
uint8_t mp_arc(double x, double y, double z, double a,
			   double i, double j, double k, 
			   double theta, double radius, 
		   	   double angular_travel, double linear_travel, 
		   	   uint8_t axis_1, uint8_t axis_2, uint8_t axis_linear,
			   double minutes);

uint8_t mp_go_home_cycle(void);

void mp_unit_tests(void);

#endif
