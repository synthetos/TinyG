/* stepper.h - stepper motor interface
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for details. You should have received a copy of the GNU General Public 
 * License along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* 
 *	Coordinated motion (line drawing) is performed using a classic 
 *	Bresenham DDA as per reprap and grbl. A number of additional steps 
 *	are taken to optimize interpolation and pulse train accuracy.
 *
 *	- The DDA accepts and processes fractional motor steps. Steps are 
 *	  passed to the move queue as floats, and do not need to be integer
 *	  values. The DDA implements fractional steps and interpolation by 
 *	  extending the counter range downward using the DDA_SUBSTEPS setting. 
 *
 *	- The DDA is not used as a 'ramp' for acceleration management. Accel
 *	  is computed as 3rd order (controlled jerk) equations that generate 
 *	  accel/decel segments to the DDA in much the same way arc drawing
 *	  is approximated. The DDA runs at a constant rate for each segment,
 *	  up to a maximum of 50 Khz step rate.
 *
 *	- The DDA rate for a segment is set to an integer multiple of the 
 *	  step freqency of the fastest motor (major axis). This amount of 
 *	  overclocking is controlled by the DDA_OVERCLOCK value, typically 16x.
 *	  A minimum DDA rate is enforced that prevents overflowing the 16 bit 
 *	  DDA timer PERIOD value. The DDA timer always runs at 32 Mhz: the 
 *	  prescaler is not used. Various methods are used to keep the numbers 
 *	  in range for long lines. See _st_set_f_dda() for details.
 *
 *	- Pulse phasing is preserved between segments if possible. This makes
 *	  for smoother motion, particularly at very low speeds and short 
 *	  segment lengths (avoids pulse jitter). Phase continuity is achieved 
 *	  by simply not resetting the DDA counters across segments. In some 
 *	  cases the differences between timer values across segments are too 
 *	  large for this to work, and you risk motor stalls due to pulse 
 *	  starvation. These cases are detected and the counters are reset 
 *	  to prevent stalling.
 *
 *  - Pulse phasing is also helped by minimizing the time spent loading 
 *	  the next move segment. To this end as much as possible about that 
 *	  move is pre-computed during move execution. Also, all moves are 
 *	  loaded from the interrupt level, avoiding the need for mutual 
 *	  exclusion locking or volatiles (which slow things down).
 */
/**** Line planning and execution ****
 *
 *	Move planning, execution and pulse generation takes place at 3 levels:
 *
 *	Move planning occurs in the main-loop. The canonical machine calls the
 *	planner to generate lines, arcs, dwells and synchronous stop/starts.
 *	The planner module generates blocks (bf's) that hold parameters for 
 *	lines and the other move types. The blocks are backplanned to join 
 *	lines, and to take dwells and stops into account. ("plan" stage).
 *
 *	Arc movement is planned above the above the line planner. The arc 
 *	planner generates short lines that are passed to the line planner.
 *
 *	Move execution and load prep takes place at the LOW interrupt level. 
 *	Move execution generates the next acceleration, cruise, or deceleration
 *	segment for planned lines, or just transfers parameters needed for 
 *	dwells and stops. This layer also prepares moves for loading by 
 *	pre-calculating the values needed by the DDA, and converting the 
 *	executed move into parameters that can be directly loaded into the 
 *	steppers ("exec" and "prep" stages).
 *
 *	Pulse train generation takes place at the HI interrupt level. 
 *	The stepper DDA fires timer interrupts that generate the stepper pulses. 
 *	This level also transfers new stepper parameters once each pulse train
 *	("segment") is complete ("load" and "run" stages). 
 */
/* 	What happens when the pulse generator is done with the current pulse train 
 *	(segment) is a multi-stage "pull" queue that looks like this:
 *
 *	As long as the steppers are running the sequence of events is:
 *	  - The stepper interrupt (HI) runs the DDA to generate a pulse train
 *	  	  for the current move. This runs for the length of the pulse train
 *		  currently executing - the "segment", usually 5ms worth of pulses
 *
 *	  - When the current segment is finished the stepper interrupt LOADs the next 
 *		  segment from the prep buffer, reloads the timers, and starts the 
 *		  next segment. At the end of the load the stepper interrupt routine
 *		  requests an "exec" of the next move in order to prepare for the 
 *		  next load operation. It does this by calling the exec using a 
 *		  software interrupt (actually a timer, since that's all we've got).
 *
 *	  - As a result of the above, the EXEC handler fires at the LO interrupt 
 *		  level. It computes the next accel/decel segment for the current move 
 *		  (i.e. the move in the planner's runtime buffer) by calling back to 
 *		  the exec routine in planner.c. Or it gets and runs the next buffer 
 *		  in the planning queue - depending on the move_type and state. 
 *
 *	  - Once the segment has been computed the exec handler finshes up by running 
 *		  the PREP routine in stepper.c. This computes the DDA values and gets 
 *		  the segment into the prep buffer - and ready for the next LOAD operation.
 *
 *	  - The main loop runs in background to receive gcode blocks, parse them,
 *		  and send them to the planner in order to keep the planner queue 
 *		  full so that when the planner's runtime buffer completes the next move
 *		  (a gcode block or perhaps an arc segment) is ready to run.
 *
 *	If the steppers are not running the above is similar, except that the exec
 * 	is invoked from the main loop by the software interrupt, and the stepper 
 *	load is invoked from the exec by another software interrupt.
 */
/*	Control flow can be a bit confusing. This is a typical sequence for planning 
 *	executing, and running an acceleration planned line:
 *
 *	 1  planner.mp_aline() is called, which populates a planning buffer (bf) 
 *		and back-plans any pre-existing buffers.
 *
 *	 2  When a new buffer is added _mp_queue_write_buffer() tries to invoke
 *	    execution of the move by calling stepper.st_request_exec_move(). 
 *
 *	 3a If the steppers are running this request is ignored.
 *	 3b If the steppers are not running this will set a timer to cause an 
 *		EXEC "software interrupt" that will ultimately call st_exec_move().
 *
 *   4  At this point a call to _exec_move() is made, either by the 
 *		software interrupt from 3b, or once the steppers finish running 
 *		the current segment and have loaded the next segment. In either 
 *		case the call is initated via the EXEC software interrupt which 
 *		causes _exec_move() to run at the MEDium interupt level.
 *		 
 *	 5	_exec_move() calls back to planner.mp_exec_move() which generates 
 *		the next segment using the mr singleton.
 *
 *	 6	When this operation is complete mp_exec_move() calls the appropriate
 *		PREP routine in stepper.c to derive the stepper parameters that will 
 *		be needed to run the move - in this example st_prep_line().
 *
 *	 7	st_prep_line() generates the timer and DDA values and stages these into 
 *		the prep structure (sp) - ready for loading into the stepper runtime struct
 *
 *	 8	stepper.st_prep_line() returns back to planner.mp_exec_move(), which 
 *		frees the planning buffer (bf) back to the planner buffer pool if the 
 *		move is complete. This is done by calling _mp_request_finalize_run_buffer()
 *
 *	 9	At this point the MED interrupt is complete, but the planning buffer has 
 *		not actually been returned to the pool yet. The buffer will be returned
 *		by the main-loop prior to testing for an available write buffer in order
 *		to receive the next Gcode block. This handoff prevents possible data 
 *		conflicts between the interrupt and main loop.
 *
 *	10	The final step in the sequence is _load_move() requesting the next 
 *		segment to be executed and prepared by calling st_request_exec() 
 *		- control goes back to step 4.
 *
 *	Note: For this to work you have to be really careful about what structures
 *	are modified at what level, and use volatiles where necessary.
 */
/* Partial steps and phase angle compensation
 *
 *	The DDA accepts partial steps as input. Fractional steps are managed by the 
 *	sub-step value as explained elsewhere. The fraction initially loaded into 
 *	the DDA and the remainder left at the end of a move (the "residual") can
 *	be thought of as a phase angle value for the DDA accumulation. Each 360
 *	degrees of phase angle results in a step being generated. 
 */

#ifndef stepper_h
#define stepper_h

void stepper_init(void);

void st_set_motor_idle_timeout(float seconds);
void st_do_motor_idle_timeout(void);

void st_energize_motor(const uint8_t motor);
void st_deenergize_motor(const uint8_t motor);
void st_set_motor_power(const uint8_t motor);

void st_energize_motors(void);
void st_deenergize_motors(void);
void st_idle_motors(void);

stat_t st_motor_power_callback(void);

uint8_t st_isbusy(void);		// return TRUE is any axis is running (F=idle)
void st_set_polarity(const uint8_t motor, const uint8_t polarity);
void st_set_microsteps(const uint8_t motor, const uint8_t microstep_mode);
void st_set_power_mode(const uint8_t motor, const uint8_t power_mode);

uint8_t st_test_prep_state(void);
void st_request_exec_move(void);
void st_prep_null(void);
void st_prep_dwell(float microseconds);
stat_t st_prep_line(float steps[], float microseconds);

uint16_t st_get_stepper_run_magic(void);
uint16_t st_get_stepper_prep_magic(void);

#ifdef __DEBUG
void st_dump_stepper_state(void);
#endif

// handy macro
#define _f_to_period(f) (uint16_t)((float)F_CPU / (float)f)

/*
 * Stepper configs and constants
 */

// Currently there is no distinction between IDLE and OFF (DEENERGIZED)
// In the future IDLE will be powered at a low, torque-maintaining current

enum motorState {					// used w/start and stop flags to sequence motor power
	MOTOR_OFF = 0,					// motor is stopped and deenergized
	MOTOR_IDLE,						// motor is stopped and may be partially energized for torque maintenance
	MOTOR_STOPPED,					// motor is stopped and fully energized
	MOTOR_RUNNING					// motor is running (and fully energized)
};

enum cmStepperPowerMode {
	MOTOR_ENERGIZED_DURING_CYCLE=0,	// motor is fully powered during cycles
	MOTOR_IDLE_WHEN_STOPPED,		// idle motor shortly after it's stopped - even in cycle
	MOTOR_POWER_REDUCED_WHEN_IDLE,	// enable Vref current reduction (not implemented yet)
	DYNAMIC_MOTOR_POWER				// adjust motor current with velocity (not implemented yet)
};

enum prepBufferState {
	PREP_BUFFER_OWNED_BY_LOADER = 0,// staging buffer is ready for load
	PREP_BUFFER_OWNED_BY_EXEC		// staging buffer is being loaded
};

// motor stop bitfields
#define M1_STOP (0x01)				// used to set st_run.motor_stop_flags
#define M2_STOP	(0x02)
#define M3_STOP	(0x04)
#define M4_STOP	(0x08)
#define M5_STOP	(0x10)
#define M6_STOP	(0x20)
#define ALL_MOTORS_STOPPED	(M1_STOP | M2_STOP | M3_STOP | M4_STOP | M5_STOP | M6_STOP)

/* Timer settings for stepper module. See hardware.h for overall timer assignments */

// Stepper power management settings
//	Min/Max timeouts allowed for motor disable. Allow for inertial stop; must be non-zero
#define IDLE_TIMEOUT_SECONDS_MIN 	0.1				 // seconds !!! SHOULD NEVER BE ZERO !!!
#define IDLE_TIMEOUT_SECONDS_MAX   (4294967295/1000) // for conversion to uint32_t
#define IDLE_TIMEOUT_SECONDS 		0.1				 // seconds in DISABLE_AXIS_WHEN_IDLE mode

/* DDA substepping
 * 	DDA_SUBSTEPS sets the amount of fractional precision for substepping.
 *	Substepping is kind of like microsteps done in software to make
 *	interpolation more accurate.
 *
 *	Set to 1 to disable, but don't do this or you will lose a lot of accuracy.
 */
#define DDA_SUBSTEPS 100000		// 100,000 accumulates substeps to 6 decimal places

/* DDA overclocking
 * 	Overclocking multiplies the step rate of the fastest axis (major axis) 
 *	by an integer value up to the DDA_OVERCLOCK value. This makes the 
 *	interpolation of the non-major axes more accurate than simply setting
 *	the DDA to the speed of the major axis; and allows the DDA to run at 
 *	less than the max frequency when possible.
 *
 *	Set to 0 to disable.
 *
 *	NOTE: TinyG doesn't use tunable overclocking any more. It just overclocks
 *	at the fastest sustainable step rate which is about 50 Khz for the xmega.
 *	This minimizes the aliasing on minor axes at minimal impact to the major 
 *	axis. The DDA overclock setting and associated code are left in for historical
 *	purposes and in case we ever want to go back to pure overclocking.
 *
 *	Setting this value to 0 has the effect of telling the optimizer to take out
 *	entire code regions that are not called if this value is zero. So they are 
 *	left in for historical purposes and not commented out. These regions are noted.
 */
//#define DDA_OVERCLOCK 16		// doesn't have to be a binary multiple
#define DDA_OVERCLOCK 0			// Permanently disabled. See above NOTE

/* Accumulator resets
 * 	You want to reset the DDA accumulators if the new ticks value is way less 
 *	than previous value, but otherwise you should leave the accumulators alone.
 *	Preserving the accumulator value from the previous segment aligns pulse 
 *	phasing between segments. However, if the new accumulator is going to be 
 *	much less than the old one you must reset it or risk motor stalls.
 */
#define ACCUMULATOR_RESET_FACTOR 2	// amount counter range can safely change

/* DDA minimum operating frequency
 *	This is the minumum value the DDA time can run with a fixed 32 Mhz 
 *	clock. Anything lower will overflow the 16 bit PERIOD register.
 */
//#define F_DDA_MIN (float)489	// hz
#define F_DDA_MIN (float)500	// hz - is 489 Hz with some margin

/* Timer settings for stepper module. See system.h for timer assignments
 */
#define F_DDA 		(float)50000	// DDA frequency in hz.
#define F_DWELL		(float)10000	// Dwell count frequency in hz.
#define SWI_PERIOD 	100				// cycles you have to shut off SW interrupt
#define TIMER_PERIOD_MIN (20)		// used to trap bad timer loads

// Timer setups
#define STEP_TIMER_TYPE		TC0_struct // stepper subsubstem uses all the TC0's
#define STEP_TIMER_DISABLE 	0		// turn timer off (clock = 0 Hz)
#define STEP_TIMER_ENABLE	1		// turn timer clock on (F_CPU = 32 Mhz)
#define STEP_TIMER_WGMODE	0		// normal mode (count to TOP and rollover)

#define TIMER_DDA_ISR_vect	TCC0_OVF_vect	// must agree with assignment in system.h
#define TIMER_DWELL_ISR_vect TCD0_OVF_vect	// must agree with assignment in system.h
#define TIMER_LOAD_ISR_vect	TCE0_OVF_vect	// must agree with assignment in system.h
#define TIMER_EXEC_ISR_vect	TCF0_OVF_vect	// must agree with assignment in system.h

#define TIMER_OVFINTLVL_HI	3		// timer interrupt level (3=hi)
#define	TIMER_OVFINTLVL_MED 2;		// timer interrupt level (2=med)
#define	TIMER_OVFINTLVL_LO  1;		// timer interrupt level (1=lo)

#define TIMER_DDA_INTLVL 	TIMER_OVFINTLVL_HI
#define TIMER_DWELL_INTLVL	TIMER_OVFINTLVL_HI
#define TIMER_LOAD_INTLVL	TIMER_OVFINTLVL_HI
#define TIMER_EXEC_INTLVL	TIMER_OVFINTLVL_LO

#endif
