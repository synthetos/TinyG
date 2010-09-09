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

#ifndef stepper_h
#define stepper_h 

#include <avr/io.h>
#include <avr/sleep.h>

/* function prototypes */

void st_buffer_line(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t rate);
void st_init(void);					// Initialize and start the stepper motor subsystem
void st_synchronize(void);			// Block until all buffered steps are executed
void st_flush(void);				// Cancel all pending steps
void st_go_home(void);				// Execute the homing cycle

#endif
