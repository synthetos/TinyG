/*
 * encoder.h - encoder interface
 * This file is part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* 
 *	Coordinated motion (line drawing) is performed using a classic Bresenham DDA. 
 *	A number of additional steps are taken to optimize interpolation and pulse train
 *	timing accuracy to minimize pulse jitter and make for very smooth motion and surface
 *	finish.
 *
 *    - The DDA is not used as a 'ramp' for acceleration management. Accel is computed 
 *		upstream in the motion planner as 3rd order (controlled jerk) equations. These
 *		generate accel/decel segments that rae passed to the DDA for step output.
 *
 *	  - The DDA accepts and processes fractional motor steps as floating point (doubles) 
 *		from the planner. Steps do not need to be whole numbers, and are not expected to be. 
 *		The step values are converted to integer by multiplying by a fixed-point precision 
 *		(DDA_SUBSTEPS, 100000). Rounding is performed to avoid a truncation bias.
 *
 *		If you enable the step diagnostics you will see that the planner and exec functions
 *		accurately generate the right number of fractional steps for the move during the 
 *		accel/cruise/decel phases. The theoretical value and the calculated value collected
 *		in steps_total agree to within 0.0001% or better.
 *
 *    - Constant Rate DDA clock: The DDA runs at a constant, maximum rate for every 
 *		segment regardless of actual step rate required. This means that the DDA clock 
 *		is not tuned to the step rate (or a multiple) of the major axis, as is typical
 *		for most DDAs. Running the DDA flat out might appear to be "wasteful", but it ensures 
 *		that the best aliasing results are achieved.
 *
 *		The observation is that TinyG is a hard real-time system in which every clock cycle 
 *		is knowable and can be accounted for. So if the system is capable of sustaining
 *		max pulse rate for the fastest move, it's capable of sustaining this rate for any
 *		move. So we just run it flat out and get the best pulse resolution for all moves. 
 *		If we were running from batteries or otherwise cared about the energy budget we 
 *		might not be so cavalier about this.
 *
 *		At 50 KHz constant clock rate we have 20 uSec between pulse timer (DDA) interrupts. 
 *		On the Xmega we consume <10 uSec in the interrupt - a whopping 50% of available cycles 
 *		going into pulse generation. On the ARM this is less of an issue, and we run a 
 *		100 Khz (or higher) pulse rate.
 *
 *    - Pulse timing is also helped by minimizing the time spent loading the next move 
 *		segment. The time budget for the load is less than the time remaining before the 
 *		next DDA clock tick. This means that the load must take < 10 uSec or the time  
 *		between pulses will stretch out when changing segments. This does not affect 
 *		positional accuracy but it would affect jitter and smoothness. To this end as much 
 *		as possible about that move is pre-computed during move execution (prep cycles). 
 *		Also, all moves are loaded from the DDA interrupt level (HI), avoiding the need 
 *		for mutual exclusion locking or volatiles (which slow things down).
 */
/*
 * ENCODERS
 *
 *	This is kind of a lie, at least for now. There are no oncoders. Instead the steppers count
 *	steps to provide a "truth" reference for position. This is used by the planner to correct 
 *	for drift. In the future when we have real encoders we'll stop counting steps and actually 
 *	measure the position. Which will be a lot easier than what this file does.
 */
#ifndef ENCODER_H_ONCE
#define ENCODER_H_ONCE


/**** Configs and Constants ****/


/**** Structures ****/

typedef struct enEncoder { 			// one real or virtual encoder per controlled motor
	uint8_t motor;					// motor encoder is mapped to 
	int8_t step_sign;				// set to +1 or -1
	int16_t steps_run;				// steps counted during stepper interrupt
	int32_t steps_total;			// steps accumulated from steps_run
	int32_t steps_total_display;	// total steps saved for display purposes
	float steps_float;				// incoming steps steps +++++ DIAGNOSTIC ONLY
	float target;					// target position (mm)
	float position;					// measured or counted position	(mm)
	float error;					// error between target and position (mm)
} enEncoder_t;

typedef struct enEncoders {
	magic_t magic_start;
	enEncoder_t en[MOTORS];			// runtime encoder structures
	magic_t magic_end;
} enEncoders_t;

extern enEncoders_t en;


/**** FUNCTION PROTOTYPES ****/

void encoder_init(void);
stat_t en_assertions(void);
void en_reset_encoder(const uint8_t motor);
void en_reset_encoders(void);
void en_update_target(const uint8_t motor, float target);
void en_update_position(const uint8_t motor);
void en_add_incoming_steps(const uint8_t motor, float steps);
//enEncoder_t *en_read_encoder(const uint8_t motor);
void en_print_encoders(void);

#endif	// End of include guard: ENCODER_H_ONCE
