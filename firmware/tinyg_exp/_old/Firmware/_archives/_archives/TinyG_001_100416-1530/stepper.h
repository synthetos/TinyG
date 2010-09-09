/*
  stepper.h - stepper motor interface
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
*/
/* 
  TinyG Notes
  Modified to support Xmega family processors
  Modifications Copyright (c) 2010 Alden S. Hart, Jr.

*/
#ifndef stepper_h
#define stepper_h 

#include <avr/io.h>
#include <avr/sleep.h>

struct Line {
 	uint32_t steps_x; 					// total steps in x direction
	uint32_t steps_y;  					// total steps in y direction
	uint32_t steps_z; 					// total steps in z direction
 	uint32_t microseconds; 				// total microseconds the move will take
// 	int32_t steps_max; 					// total steps it will take (max of the axes)
//	uint8_t direction_bits;				// bitmask for directions
};

struct axis { 						// one instance per axis
	/* operating  variables */
	int32_t counter;				// counts steps down to 0 (end of line)

	/* configuration variables */
	uint8_t microsteps;				// microsteps for this axis
	double max_seek_rate;			// maximum G0 speed (no load) in mm per minute
	double max_seek_steps;			// maximum G0 speed (no load) in steps / second
	double max_feed_rate;			// maximum G1/G2/G3 speed in mm per minute
	double max_feed_steps;			// maximum G1/G2/G3 speed in steps per second
	double steps_per_mm;			// steps per mm traveled for this axis

	/* register bindings */
	struct PORT_struct *port;		// motor control port
	struct TC0_struct *timer;		// timer/counter (type 0)
};

/* axes main structure
	Holds the individual axis structs
	Active_axes has bit set if axis is active. If thery are all clear the robot is idle
	Pattern is: X_BIT || Y_BIT || Z_BIT || A_BIT  (see config.h)
*/

struct axes {						// collect them all up and make easy access
	struct axis x;
	struct axis y;
	struct axis z;
	struct axis a;
	uint8_t active_axes;			// bit set if axis is active. 0 = robot is idle
};

/* function prototypes */

void st_motor_test(void);			// Test stepper motor subsystem
void st_init(void);					// Initialize and start stepper motor subsystem

void st_execute_line(void);		// Load and start next line from the line buffer
void _st_load_timer(struct axis *a, uint32_t step_rate, uint32_t microseconds); // helper function

struct Line *st_get_next_line(void);// Return pointer to next line struct
void st_synchronize(void);			// Block until all buffered steps are executed
void st_flush(void);				// Cancel all pending steps
void st_go_home(void);				// Execute the homing cycle
void st_buffer_line(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t rate);

void st_print_line(struct Line line); 
void st_print_active(void);
void st_print_four_ints(long x, long y, long z, long f);

#endif

