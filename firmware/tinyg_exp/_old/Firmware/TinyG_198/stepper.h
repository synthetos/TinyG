/*
 * stepper.h - stepper motor interface
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 * Portions if this module copyright (c) 2009 Simen Svale Skogsrud
 *
 * TinyG is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with TinyG  
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef stepper_h
#define stepper_h

/*
 * Global Scope Functions
 */

void st_init(void);					// Initialize and start stepper motor subsystem
void st_motor_test(void);			// Test stepper motor subsystem
void st_execute_move(void);			// Dequeue and start next linear move in the move buffer
void st_set_polarity(uint8_t axis, uint8_t polarity);
void st_kill(void);					// Kill current move
void st_terminate(void);			// Terminate moves after the current move

#endif
