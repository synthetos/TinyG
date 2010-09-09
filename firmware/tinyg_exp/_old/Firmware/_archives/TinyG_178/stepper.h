/*
 * stepper.h - stepper motor interface
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 */

#ifndef stepper_h
#define stepper_h 

#include <avr/io.h>
#include <avr/sleep.h>

/*
 * Global Scope Functions
 */

void st_init(void);					// Initialize and start stepper motor subsystem
void st_motor_test(void);			// Test stepper motor subsystem
void st_execute_move(void);			// Dequeue and start next linear move in the move buffer
void st_kill(void);					// Kill current move
void st_terminate(void);			// Terminate moves after the current move
void st_buffer_move(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t rate);
uint8_t st_buffer_full(void);		// test if the line buffer is full
void st_flush(void);				// Cancel all pending steps
void st_go_home(void);				// Execute the homing cycle
void st_synchronize(void);			// Block until all buffered steps are executed

#endif

