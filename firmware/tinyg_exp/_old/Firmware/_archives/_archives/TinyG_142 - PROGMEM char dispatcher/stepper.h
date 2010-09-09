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

struct Line {						// Lines are queued stepper movements
 	int32_t steps_x; 				// total steps in x direction (signed)
	int32_t steps_y;  				// total steps in y direction (signed)
	int32_t steps_z; 				// total steps in z direction (signed)
 	uint32_t microseconds; 			// total microseconds for the move (unsigned)
};

struct Axis { 						// axis control struct - one per axis
	/* operating  variables */
	int32_t counter;				// counts steps down to 0 (end of line)

	/* register bindings */
	struct PORT_struct *port;		// motor control port
	struct TC0_struct *timer;		// timer/counter (type 0)
};

/* axes main structure
	Holds the individual axis structs
	Active_axes has bit set if axis is active. If thery are all clear the robot is idle
	Pattern is: X_BIT || Y_BIT || Z_BIT || A_BIT  (see config.h)
*/

struct Axes {						// All axes grouped in 1 struct + some extra stuff
	struct Axis x;
	struct Axis y;
	struct Axis z;
	struct Axis a;
	uint8_t active_axes;			// bit set if axis is active. 0 = robot is idle
};

/* function prototypes */

void st_motor_test(void);			// Test stepper motor subsystem
void st_init(void);					// Initialize and start stepper motor subsystem

void st_execute_line(void);		// Load and start next line from the line buffer
void _st_load_timer(struct Axis *a, uint32_t step_rate, uint32_t microseconds); // helper function

struct Line *st_get_next_line(void);// Return pointer to next line struct
void st_synchronize(void);			// Block until all buffered steps are executed
void st_flush(void);				// Cancel all pending steps
void st_go_home(void);				// Execute the homing cycle
void st_buffer_line(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t rate);

#endif

