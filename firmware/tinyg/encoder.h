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
 * ERROR CORRECTION
 *
 *	The purpose of this module is to calculate an error term between the programmed position
 *	(target) and the actual measured position (position). The error term is used during move 
 *	execution (exec) to adjust the move to cancel accumulated positional errors.
 *
 *	Positional error occurs due to floating poiunt numerical inaccuracies. TinyG uses 32 bit
 *	floating point (GCC 32 bit, which is NOT IEEE 32 bit). Errors creep in during planning,
 *	move execution, and stepper output phases. Many steps have been taken to minimize errors
 *	at all these stages, but they still occur. In most cases the errors are not noticable as 
 *	they fall below the step resolution for most jobs. For jobs that run > 1 hour the errors 
 *	can accumulate and send results off by as much as a couple millimeters if not corrected.  
 *	Note: Going to doubles would reduce the errors but not eliminate them altogether.
 */
/* 
 * ENCODERS	
 *
 *	Calling this file "encoders" is kind of a lie, at least for now. There are no encoders. 
 *	Instead the steppers count steps to provide a "truth" reference for position. In the 
 *	future when we have real encoders we'll stop counting steps and actually measure the 
 *	position. Which will be a lot easier than how this module currently works.
 *
 *	*** Measuring poosition and getting the error term ***
 *
 *	The challenge is that you can't just go "from here to there" because the system is so
 *	heavily queued (pipelined). (1) The planner has a bunch of moves buffered behind the 
 *	current, running move, and (2) because of stepper sequencing by the time you can get 
 *	an accurate position reading your target has moved on - i.e. your position reading 
 *	relates to "yesterday's target".
 *
 *	Referring to ASCII art in stepper.h and reproduced here:
 *
 *  step/load (~5000uSec)          [L1][segment1][L2][segment2][L3][segment3][L4][segment4][Lb1][segmentb1]
 *  prep (100 uSec)            [P1]       [P2]          [P3]          [P4]          [Pb1]          [Pb2]
 *  exec (400 uSec)         [EXEC1]    [EXEC2]       [EXEC3]       [EXEC4]       [EXECb1]       [EXECb2]
 *  plan (<4ms)  [planmoveA][plan move B][plan move C][plan move D][plan move E] etc.
 *
 *	You can collect the target for moveA as early as the end of [planmoveA]. The system will 
 *	not reach that target position until the end of [segment4]. Data from Segment4 can only be 
 *	processed during the EXECb2 (or Pb2) interval as it's the first time that is not time-critical 
 *	and you actually have enough cycles to calculate the position and error terms. 
 *
 *	Additionally, by this time the target in Gcode model knows about has advanced quite a bit, 
 *	so the moveA target needs to be saved somewhere. Targets are propagagted downward to the planner
 *	runtime (the EXEC), but the exec will haved moved on to move2 by the time we need it. So moveA's
 *	target needs to be saved somewhere.
 *
 *	*** Applying the error term for error correction ***
 *
 *	So if you want to use the error from moveA to correct moveB it has to be done in a region that 
 *	is not already running (i.e. the head, body, or tail) as moveB is already 2 segments into run.
 *	Since most moves in very short line Gcode files are body only, for practical purposes the 
 *	correction will be applied to moveC. (It's possible to recompute the body of moveB, but it may 
 *	not be worth the trouble).
 *
 *	*** How this file works ***
 *
 *	- The encoder structure carries the variables needed to capture targets, count steps
 *	  from the stepper interrupt and calculate the position and error terms.
 *
 *	- Start with initialization (goes without saying)
 *
 *	- Use en_reset_encoder() to reset the encoders at the start of a machining cycle. This zeros 
 *	  all counts and sets the position to the current machine position as known by the Gcode model 
 *	  (i.e. above the planner and runtime models).
 *
 *	- When the move exec sends the first segment of a new Gcode block to the prep function
 *	  it passes the target for the block and a flag indicating that this is a new block.
 *	  The prep function seeds the encoder by calling en_set_target(). This puts the new 
 *	  target in a staging variable.
 *
 *  - The en.position_ready flag indicates that the move is complete. The exec or prep function 
 *	  should check the flag and call en_get_position(). This must be done in the segment window 
 *	  immediately following the flag set or the position will be corrupted as new data arrives.
 *	  The position and error term will remain stable until the next call to en_get_position().
 */
#ifndef ENCODER_H_ONCE
#define ENCODER_H_ONCE


/**** Configs and Constants ****/


/**** Structures ****/

typedef struct enEncoder { 			// one real or virtual encoder per controlled motor
//	uint8_t motor;					// motor encoder is mapped to 
	int8_t step_sign;				// set to +1 or -1
	int16_t steps_run;				// steps counted during stepper interrupt
//	int32_t steps_total;			// steps accumulated from steps_run
//	int32_t steps_total_display;	// total steps saved for display purposes
//	float next_target;				// next target position - for staging (in mm)
//	float target;					// target position (in mm)
//	float position;					// measured or counted position	(in mm)
//	float error;					// error between target and position (in mm)

	int32_t target_steps_next;		// next target position - for staging (in steps)
	int32_t target_steps;			// target position (in steps)
	int32_t position_steps;			// measured or counted position	(in steps)
	int32_t error_steps;			// step error between target and position (in steps)
	int32_t error_distance;			// distance error between target and position (in mm)
	float position_steps_float;		// incoming steps steps +++++ DIAGNOSTIC ONLY
} enEncoder_t;

typedef struct enEncoders {
	magic_t magic_start;
	uint8_t position_ready;			// signal that position is ready.
	enEncoder_t en[MOTORS];			// runtime encoder structures
	magic_t magic_end;
} enEncoders_t;

extern enEncoders_t en;


/**** FUNCTION PROTOTYPES ****/

void encoder_init(void);
stat_t en_assertions(void);
//void en_reset_encoder(const uint8_t motor);
void en_reset_encoders(void);
//void en_set_target(const uint8_t motor, float target);
void en_set_target(const float target[]);
//void en_get_position_error(const uint8_t motor);
void en_get_position_error(void);
void en_add_incoming_steps(const uint8_t motor, float steps);
void en_print_encoders(void);

#endif	// End of include guard: ENCODER_H_ONCE
