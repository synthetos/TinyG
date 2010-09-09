/*
 * stepper.c - stepper motor interface
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * Some function names have been preserved from Grbl, but the code has been 
 * rewritten to take advantage of the xmega facilities and works differently 
 * than the Grbl (Reprap) Bresenham implementations.
 *
 * Coordinated motion (line drawing) is performed by dedicating a timer to each 
 * axis and stepping each motor at a computed rate (timer period value) for a 
 * specified number of pulses (counter value). Each timeout fires a high-priority 
 * interrupt which generates a step and decrements the counter by one. Timer 
 * counters are post-scaled in software to extend the range to 32 bits.
 *
 * The main_loop routines (motion_control.c non-ISR) call st_buffer_move() to put 
 * lines into the move buffer. The timer ISRs read moves from the buffer.
 *
 * Any axis that is part of the move has its ACTIVE bit set in ax.active.
 * When the axis move is complete this bit is cleared. When all active bits are 
 * cleared st_execute_move() is called to load the next move into the timers.
 *
 * But you need some way to start the timers if they are not already running,
 * so st_execute_move() must also be called from st_buffer_move() to start line
 * execution of the timers are not already running.  st_execute_move() therefore 
 * has a busy flag to prevent ISR and non-ISR calls from stepping on each other.
 *
 * st_buffer_move() will sleep if the buffer is full, waiting for a line completion,
 * allowing the motion control routines to wake up and generate the next line segment.
 * fill up the line buffer then sleep (idle) as the lines from the buffer are executed 
 *
 * Non-blocking motion control moves never call st_buffer_move() without first 
 * checking if space is available (st_buffer_full()), so they should never sleep.
 */

#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "xmega_init.h"
#include <util/delay.h>
#include "stepper.h"
#include "config.h"
#include "tinyg.h"

#ifdef __DEBUG
#include <stdio.h>
#include <avr/pgmspace.h>
#include "xio.h"
#endif


/* 
 * Local Scope Data and Functions
 */

/*  
 *	Stepper axis structures
 *	Holds the individual axis structs
 *	Active_axes has bit set if axis is active. If thery are all clear the robot is idle
 *	Pattern is: X_BIT || Y_BIT || Z_BIT || A_BIT  (see config.h)
 */

struct Axis { 						// axis control struct - one per axis
	/* operating  variables */
	uint32_t step_counter;			// counts steps down to 0 (end of line)
	uint16_t postscale_value;		// timer post-scale value
	uint16_t postscale_count;		// timer post-scale count
	uint16_t timer_period;			// value loaded into timers
	uint8_t polarity;				// 0=normal polarity, 1=reverse motor polarity

	/* register bindings */
	struct PORT_struct *port;		// motor control port
	struct TC0_struct *timer;		// timer/counter (type 0)
};

struct Axes {						// All axes grouped in 1 struct + some extra stuff
	uint8_t active_axes;			// bits are set if axis is active. 0 = robot is idle
	struct Axis x;
	struct Axis y;
	struct Axis z;
	struct Axis a;
};
static struct Axes ax;				// master axes structure

/* 
 * Move buffer structures
 */
#define MOVE_BUFFER_SIZE 4			// number of moves (lines) buffered

struct stMove {						// Linear moves are queued stepper movements
 	int32_t steps_x; 				// total steps in x direction (signed)
	int32_t steps_y;  				// total steps in y direction (signed)
	int32_t steps_z; 				// total steps in z direction (signed)
 	uint32_t microseconds; 			// total microseconds for the move (unsigned)
};

struct stMoves {
	volatile uint8_t move_busy; 				// MUTEX for st_execute_move()
	volatile uint8_t move_buffer_head;			// move queue index (for writes)
	volatile uint8_t move_buffer_tail;			// move dequeue index (for reads)
	struct stMove *buf;							// buffer pointer
	struct stMove move_buffer[MOVE_BUFFER_SIZE];// buffer storage
};
static struct stMoves mv;

void _st_load_timer(struct Axis *a, uint32_t steps, uint32_t ticks_per_step); 
struct stMove *_st_get_next_move(void);			// Return pointer to next move struct
void _st_print_exec_line(struct stMove mv, uint8_t active);
//void _st_print_exec_line(int32_t x, int32_t y, int32_t z, uint32_t us, uint8_t active);

/* 
 * st_motor_test() - test motor subsystem 
 */

void st_motor_test() {
	ax.x.step_counter = 0x00001000;
	ax.x.timer->PER = 0x1000;					// step rate (period)
	ax.x.timer->CTRLA = TC_CLK_ON;				// start clock

	ax.y.step_counter = 0x00000800;
	ax.y.timer->PER = 0x2000;
	ax.y.timer->CTRLA = TC_CLK_ON;

	ax.z.step_counter = 0x00000600;
	ax.z.timer->PER = 0x3000;
	ax.z.timer->CTRLA = TC_CLK_ON;

	ax.a.step_counter = 0x00000400;
	ax.a.timer->PER = 0x4000;
	ax.a.timer->CTRLA = TC_CLK_ON;

	ax.active_axes |= (X_ACTIVE_BIT_bm | Y_ACTIVE_BIT_bm | Z_ACTIVE_BIT_bm | A_ACTIVE_BIT_bm);
}

/* st_init() - initialize and start the stepper motor subsystem 
 *
 *	State at completion of initialization is:
 *	- each axis has a structure with an initialized port and a timer bound to it
 *	- ports:
 *		- input and output directions set
 *	- each axis is enabled 
 *	
 *  Note: high level interrupts must be enabled in main()
 *
 */

void st_init()
{
	ax.active_axes = 0;							// clear all active bits
//	move_busy = FALSE;								// clear the busy flag

	mv.move_buffer_head = 0;
	mv.move_buffer_tail = 0;
	mv.move_busy = FALSE;

// initialize X axis
	ax.x.polarity = cfg.a[X_AXIS].polarity;
	// motor control port
	ax.x.port = &X_MOTOR_PORT;					// bind PORT to structure
	ax.x.port->DIR = X_MOTOR_PORT_DIR_gm;		// set inputs and outputs
	ax.x.port->OUT = 0x00;						// set port bits to zero initially
	ax.x.port->OUT |= MICROSTEP_UNITS_bm;		// set microstep bits
	ax.x.port->OUTSET = MOTOR_ENABLE_BIT_bm; 	// disable the motor

	// motor control timer
	ax.x.timer = &X_TIMER;						// bind TIMER to structure
	ax.x.timer->CTRLA = TC_CLK_OFF;				// turn motor off
	ax.x.timer->CTRLB = TC_WGMODE;				// waveform generation mode
	ax.x.timer->INTCTRLA = TC_OVFINTLVL;		// interrupt mode

// initialize Y axis
	ax.y.polarity = cfg.a[Y_AXIS].polarity;

	ax.y.port = &Y_MOTOR_PORT;
	ax.y.port->DIR = Y_MOTOR_PORT_DIR_gm;
	ax.y.port->OUT = 0x00;
	ax.y.port->OUT |= MICROSTEP_UNITS_bm;
	ax.y.port->OUTSET = MOTOR_ENABLE_BIT_bm;

	ax.y.timer = &Y_TIMER;
	ax.y.timer->CTRLA = TC_CLK_OFF;
	ax.y.timer->CTRLB = TC_WGMODE;
	ax.y.timer->INTCTRLA = TC_OVFINTLVL;

// initialize Z axis
	ax.z.polarity = cfg.a[Z_AXIS].polarity;

	ax.z.port = &Z_MOTOR_PORT;
	ax.z.port->DIR = Z_MOTOR_PORT_DIR_gm;
	ax.z.port->OUT = 0x00;
	ax.z.port->OUT |= MICROSTEP_UNITS_bm;
	ax.z.port->OUTSET = MOTOR_ENABLE_BIT_bm;

	ax.z.timer = &Z_TIMER;
	ax.z.timer->CTRLA = TC_CLK_OFF;
	ax.z.timer->CTRLB = TC_WGMODE;
	ax.z.timer->INTCTRLA = TC_OVFINTLVL;

// initialize A axis
	ax.a.polarity = cfg.a[A_AXIS].polarity;

	ax.a.port = &A_MOTOR_PORT;
	ax.a.port->DIR = A_MOTOR_PORT_DIR_gm;
	ax.a.port->OUT = 0x00;
	ax.a.port->OUT |= MICROSTEP_UNITS_bm;
	ax.a.port->OUTSET = MOTOR_ENABLE_BIT_bm;

	ax.a.timer = &A_TIMER;
	ax.a.timer->CTRLA = TC_CLK_OFF;				
	ax.a.timer->CTRLB = TC_WGMODE;
	ax.a.timer->INTCTRLA = TC_OVFINTLVL;

//	st_motor_test();							// run the startup motor test
}


/*
 * ISRs - Motor timer interrupt service routines - service a tick from the axis timer
 *
 *	Use direct struct addresses and literal values for hardware devices becuase it's 
 *  faster than using the timer and port pointers in the axis structs
 */

ISR(X_TIMER_vect)
{
	if (--ax.x.postscale_count != 0) {
		return;
	}
	X_MOTOR_PORT.OUTSET = STEP_BIT_bm;			// turn X step bit on
	if (--ax.x.step_counter == 0) {
		X_TIMER.CTRLA = TC_CLK_OFF;				// stop the clock
		X_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm; // disable the motor
		ax.active_axes &= ~X_ACTIVE_BIT_bm;		// clear the X active bit
		if (ax.active_axes == 0) {				// if all axes are done
			st_execute_move();					// ...run next line
		}
	}
	ax.x.postscale_count = ax.x.postscale_value;// reset post-scaler counter
#ifdef __STEPPER_DELAY
	_delay_us(STEP_PULSE_MICROSECONDS);			// delay for correct pulse width
#endif
	X_MOTOR_PORT.OUTCLR = STEP_BIT_bm;			// turn X step bit off
}

ISR(Y_TIMER_vect)
{
	if (--ax.y.postscale_count != 0) {
		return;
	}
	Y_MOTOR_PORT.OUTSET = STEP_BIT_bm;
	if (--ax.y.step_counter == 0) {
		Y_TIMER.CTRLA = TC_CLK_OFF;
		Y_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
		ax.active_axes &= ~Y_ACTIVE_BIT_bm;
		if (ax.active_axes == 0) {
			st_execute_move();
		}
	}
	ax.y.postscale_count = ax.y.postscale_value;
#ifdef __STEPPER_DELAY
	_delay_us(STEP_PULSE_MICROSECONDS);
#endif
	Y_MOTOR_PORT.OUTCLR = STEP_BIT_bm;
}

ISR(Z_TIMER_vect)
{
	if (--ax.z.postscale_count != 0) {
		return;
	}
	Z_MOTOR_PORT.OUTSET = STEP_BIT_bm;
	if (--ax.z.step_counter == 0) {
		Z_TIMER.CTRLA = TC_CLK_OFF;	
		Z_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
		ax.active_axes &= ~Z_ACTIVE_BIT_bm;
		if (ax.active_axes == 0) {
			st_execute_move();
		}
	}
	ax.z.postscale_count = ax.z.postscale_value;
#ifdef __STEPPER_DELAY
	_delay_us(STEP_PULSE_MICROSECONDS);
#endif
	Z_MOTOR_PORT.OUTCLR = STEP_BIT_bm;
}

ISR(A_TIMER_vect)
{
	if (--ax.a.postscale_count != 0) {
		return;
	}
	A_MOTOR_PORT.OUTSET = STEP_BIT_bm;
	if (--ax.a.step_counter == 0) {
		A_TIMER.CTRLA = TC_CLK_OFF;
		A_MOTOR_PORT.OUTSET = MOTOR_ENABLE_BIT_bm;
		ax.active_axes &= ~A_ACTIVE_BIT_bm;
		if (ax.active_axes == 0) {
			st_execute_move();
		}
	}
	ax.a.postscale_count = ax.a.postscale_value;
#ifdef __STEPPER_DELAY
	_delay_us(STEP_PULSE_MICROSECONDS);
#endif
	A_MOTOR_PORT.OUTCLR = STEP_BIT_bm;
}

/*
 * st_execute_move() - dequeue move and load into stepper motors (if possible)
 *
 *	Load next linear move into timers and set direction bits
 *	If the motors are currently active it will load the line
 *	This routine can be called from ISR or non-ISR levels - mediated by "busy"
 *
 *	Busy race condition - there is a brief race condition in the busy test that 
 * 	should not actually cause any problems. If the routine were invoked by 
 *	st_buffer_line() (i.e. non-ISR invocation) an ISR call could occur during 
 *	the busy test; which the ISR *could* find the routine is not f - even 
 *	though it was previously invoked by the non-ISR caller. The interrupt would 
 *	run, loading the next line (or not), then return control to the non-ISR 
 *	invocation. The non-ISR invocation would then find that the axes were active 
 *	(ax.active_axes test), and exit. Alternately, it might find that the axes 
 *	were not active, but exit on the buffer empty test - because this is the 
 *	reason they are not active - the ISR found nothing to load. So please don't 
 *	mess with the ordering of this code region.
 */

void st_execute_move()
{
	uint64_t ticks;	// Timer ticks in the move. 2 minute move would overflow 32 bits
					// Using 64 bits is expensive! The division goes from ~640 
					// cycles at 32 buts to ~3800 cycles using 64 bits

	// don't re-order this code region - from here...
	if (mv.move_busy) { 	// prevents ISR from clobbering non-ISR invocation
		return;
	}
	mv.move_busy = TRUE;
	if (ax.active_axes != 0) {	// exit if any axis is still busy (any bit set)
		mv.move_busy = FALSE;	
		return;
	}
	if ((mv.buf = _st_get_next_move()) == NULL) {	// NULL is empty buffer condition
		mv.move_busy = FALSE;
		return;
  	}  //...to here. See race condition note.

	ax.active_axes = 0; 							// clear active axes

	// set direction bits (I almost never use the ternary operator, but here goes)
	((mv.buf->steps_x < 0) ^ ax.x.polarity) ? 
		(ax.x.port->OUTSET = DIRECTION_BIT_bm):		// CCW
		(ax.x.port->OUTCLR = DIRECTION_BIT_bm);		// CW

	((mv.buf->steps_y < 0) ^ ax.y.polarity) ? 
		(ax.y.port->OUTSET = DIRECTION_BIT_bm): 
		(ax.y.port->OUTCLR = DIRECTION_BIT_bm);

	((mv.buf->steps_z < 0) ^ ax.z.polarity) ? 
		(ax.z.port->OUTSET = DIRECTION_BIT_bm): 
		(ax.z.port->OUTCLR = DIRECTION_BIT_bm);

	// load timers
	ticks = mv.buf->microseconds * TICKS_PER_MICROSECOND;

	if (labs(mv.buf->steps_x) != 0) { 					// X axis
		_st_load_timer(&ax.x, labs(mv.buf->steps_x), 
		   (uint32_t)(ticks / labs(mv.buf->steps_x)));
		ax.active_axes |= (X_ACTIVE_BIT_bm);			// set X active
	}
	if (labs(mv.buf->steps_y) != 0) {  					// Y axis
		_st_load_timer(&ax.y, labs(mv.buf->steps_y), 
		   (uint32_t)(ticks / labs(mv.buf->steps_y)));
		ax.active_axes |= (Y_ACTIVE_BIT_bm);
	}
	if (labs(mv.buf->steps_z) != 0) { 					// Z axis
		_st_load_timer(&ax.z, labs(mv.buf->steps_z), 
		   (uint32_t)(ticks / labs(mv.buf->steps_z)));
		ax.active_axes |= (Z_ACTIVE_BIT_bm);
	}

	// enable the all at the same time (roughly). Better for motor sync.
	if (ax.active_axes || X_ACTIVE_BIT_bm) { ax.x.timer->CTRLA = TC_CLK_ON; }
	if (ax.active_axes || Y_ACTIVE_BIT_bm) { ax.y.timer->CTRLA = TC_CLK_ON; }
	if (ax.active_axes || Z_ACTIVE_BIT_bm) { ax.z.timer->CTRLA = TC_CLK_ON; }

#ifdef __DEBUG
//	_st_print_exec_line(*(mv.buf), ax.active_axes);
//	_st_print_exec_line(mv.buf->steps_x, mv.buf->steps_y, mv.buf->steps_z, 
//						mv.buf->microseconds, ax.active_axes);
#endif

	mv.move_busy = FALSE;
}

/* _st_load_timer() - helper routine for st_execute_move() 
 *
 *	The ISR should also be modified to end each move on a whole-step boundary 
 *	for power management reasons, and possibly revert the microsteps to whole
 *	if necessary to do this.
 */

void _st_load_timer(struct Axis *A, uint32_t steps, uint32_t ticks_per_step) 
{
	A->timer->CTRLA = TC_CLK_OFF;				// turn clock off, just to be sure
	A->port->OUTCLR = MOTOR_ENABLE_BIT_bm;		// enable motor
	A->step_counter = steps;

	// Normalize ticks_per_step by right shifting until the MSword is zero.
	// Accumulate LSBs shifted out of ticks_per_step into postscale_value.
	A->postscale_value = 1;
	while (ticks_per_step & 0xFFFF0000) {
		ticks_per_step >>= 1;					//(ticks_per_step >> 1);
		A->postscale_value <<= 1;
	}
	A->postscale_count = A->postscale_value;
	A->timer_period = (uint16_t)(ticks_per_step & 0x0000FFFF);
	A->timer->PER = (uint16_t)(ticks_per_step & 0x0000FFFF); // timer period
}

/*
 * st_queue_move_buffer() - Add a new linear movement to the move buffer
 *	
 * Inputs:
 *	steps_x, steps_y and steps_z are the signed, relative motion in steps 
 *	Microseconds specify how many microseconds the move should take to perform.
 *
 * Move buffer circular buffer operation
 *  move_buffer_tail is the array index from which the previous line was read.
 *	move_buffer_tail is always incremented before reading the line.
 *
 *	move_buffer_head is the array index to which the line will be written.
 *	move_buffer_head is always incremented after writing the line.
 *
 *	Buffer full:	move_buffer_head == move_buffer_tail
 *	Buffer empty:	move_buffer_head == move_buffer_tail+1 
 */

void st_queue_move_buffer(int32_t steps_x, 
						  int32_t steps_y, 
						  int32_t steps_z, 
						  uint32_t microseconds) 
{
	int next_buffer_head;

	// Bail on a zero length line (perhaps test for abs val < min line length)
	if ((steps_x == 0) && (steps_y == 0) && (steps_z) == 0) {
		return;
	};

	// Determine the buffer head index needed to store this line
	if ((next_buffer_head = mv.move_buffer_head + 1) >= MOVE_BUFFER_SIZE) {
		next_buffer_head = 0;					 // wrap condition
	}

	// If the buffer is full sleep until there is room in the buffer.
	while(mv.move_buffer_tail == next_buffer_head) {// tail advances, breaking loop
		sleep_mode(); // non-blocking queuing routines don't enter this routine 
					  // unless there is room in the queue. So this never gets hit.
	}

	mv.buf = &mv.move_buffer[mv.move_buffer_head];
	mv.buf->steps_x = steps_x; 			// write the line record to the buffer head
	mv.buf->steps_y = steps_y;
	mv.buf->steps_z = steps_z;  
	mv.buf->microseconds = microseconds;

	mv.move_buffer_head = next_buffer_head;
	st_execute_move();						// run this line
}

/*
 * st_test_move_buffer_full() - Test if the move buffer is full
 *
 *	Returns:	1 (TRUE) 	the buffer is full
 *  			0 (FALSE) 	the buffer is not full
 *
 *  Note: The method of testing for buffer full then writing the buffer as a
 *		  separate, non-atomic operation works as long as there is no pre-emption
 *		  that could invalidate the full/not-full result. As this project is
 *		  currently coded there is no pre-emption possible in this critical region -
 *		  i.e. there's no way somebody else can get in there and write to the line 
 *		  buffer between the not-full result and the subsequent write. 
 *		  Be careful about changing this condition.
 */

uint8_t st_test_move_buffer_full() 
{
	int next_buffer_head;

	if ((next_buffer_head = mv.move_buffer_head + 1) >= MOVE_BUFFER_SIZE) {
		next_buffer_head = 0;					  // wrap condition
	}
	if (mv.move_buffer_tail == next_buffer_head) { 	// test if full
		return (TRUE);
	};
	return (FALSE);
}

/* 
 * _st_get_next_move() - return the next line from the move buffer & advance buffer tail
 */ 

struct stMove *_st_get_next_move()
{
	if (mv.move_buffer_head == mv.move_buffer_tail) {	// buffer empty
		return (NULL);
	}
	mv.buf = &mv.move_buffer[mv.move_buffer_tail];	// get and save current index
	if (++mv.move_buffer_tail >= MOVE_BUFFER_SIZE) { // increment and wrap (no OBOE)
		mv.move_buffer_tail = 0;
	}
	return (mv.buf); 
}

/* 
 * st_synchronize() - block until all buffered steps are executed 
 */

void st_synchronize()
{
	while(mv.move_buffer_tail != mv.move_buffer_head) {
		sleep_mode();
	}    
}

/* 
 * st_flush() - cancel all buffered steps 
 */

void st_flush()
{
	cli();
	mv.move_buffer_tail = mv.move_buffer_head;
	mv.buf = NULL;
	sei();
}

/* 
 * st_kill() - STOP. NOW. UNCONDITIONALLY
 */

void st_kill()
{
	cli();										// stop intewrrupts
	ax.x.timer->CTRLA = TC_CLK_OFF;				// stop the clocks
	ax.y.timer->CTRLA = TC_CLK_OFF;
	ax.z.timer->CTRLA = TC_CLK_OFF;
	ax.a.timer->CTRLA = TC_CLK_OFF;
	
	mv.move_buffer_tail = mv.move_buffer_head;	// clear the move buffer
	ax.active_axes = 0;							// clear all the active bits
	sei();
}

/* 
 * st_terminate() - stop moves after the current move
 */

void st_terminate()
{
	cli();
	mv.move_buffer_tail = mv.move_buffer_head;	// clear the line buffer
	sei();
}

/* 
 * st_go_home() - perform the homing cycle 
 */

void st_go_home()
{
  // Todo: Perform the homing cycle
}


#ifdef __DEBUG
void _st_print_exec_line(struct stMove mv, uint8_t active)
{
	printf_P(PSTR("Exec X=%d Y=%d Z=%d\n"), mv.steps_x, 
											mv.steps_y, 
											mv.steps_z);
/*
	printf_P(PSTR("Exec X=%d Y=%d Z=%d uS=%d Active=%d\n"), mv.steps_x, 
															mv.steps_y, 
															mv.steps_z, 
															mv.microseconds,
															active);}
*/
#endif

/*
void _st_print_exec_line(int32_t x, int32_t y, int32_t z, uint32_t us, uint8_t active)
{
	printf_P(PSTR("Exec X=%d Y=%d Z=%d uS=%d Active=%d\n"), x, y, z, us, active);
}
*/
