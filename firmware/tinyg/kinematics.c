/*
 * kinematics.c - inverse kinematics routines
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
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

#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "stepper.h"
#include "kinematics.h"

#ifdef __cplusplus
extern "C"{
#endif

//static void _inverse_kinematics(float travel[], float joint[]);

/*
 * ik_kinematics() - wrapper routine for inverse kinematics
 *
 *	Calls kinematics function(s).
 *	Performs axis mapping & conversion of length units to steps (and deals with inhibited axes)
 *
 *	The reason steps are returned as floats (as opposed to, say, uint32_t) is to accommodate
 *	fractional DDA steps. The DDA deals with fractional step values as fixed-point binary in
 *	order to get the smoothest possible operation. Steps are passed to the move prep routine
 *	as floats and converted to fixed-point binary during queue loading. See stepper.c for details.
 */

void ik_kinematics(const float travel[], float steps[])
{
	float joint[AXES];

//	_inverse_kinematics(travel, joint);				// you can insert inverse kinematics transformations here
	memcpy(joint, travel, sizeof(float)*AXES);		//...or just do a memcpy for Cartesian machines

	// Map motors to axes and convert length units to steps
	// Most of the conversion math has already been done in during config in steps_per_unit()
	// which takes axis travel, step angle and microsteps into account.
	for (uint8_t axis=0; axis<AXES; axis++) {
		if (cm.a[axis].axis_mode == AXIS_INHIBITED) { joint[axis] = 0;}
		if (st_cfg.mot[MOTOR_1].motor_map == axis) { steps[MOTOR_1] = joint[axis] * st_cfg.mot[MOTOR_1].steps_per_unit;}
		if (st_cfg.mot[MOTOR_2].motor_map == axis) { steps[MOTOR_2] = joint[axis] * st_cfg.mot[MOTOR_2].steps_per_unit;}
		if (st_cfg.mot[MOTOR_3].motor_map == axis) { steps[MOTOR_3] = joint[axis] * st_cfg.mot[MOTOR_3].steps_per_unit;}
		if (st_cfg.mot[MOTOR_4].motor_map == axis) { steps[MOTOR_4] = joint[axis] * st_cfg.mot[MOTOR_4].steps_per_unit;}
#if (MOTORS >= 5)
		if (st_cfg.mot[MOTOR_5].motor_map == axis) { steps[MOTOR_5] = joint[axis] * st_cfg.mot[MOTOR_5].steps_per_unit;}
#endif
#if (MOTORS >= 6)
		if (st_cfg.mot[MOTOR_6].motor_map == axis) { steps[MOTOR_6] = joint[axis] * st_cfg.mot[MOTOR_6].steps_per_unit;}
#endif
	}

/* The above is a loop unrolled version of this:
	for (uint8_t axis=0; axis<AXES; axis++) {
		for (uint8_t motor=0; motor<MOTORS; motor++) {
			if (st_cfg.mot[motor].motor_map == axis) {
				steps[motor] = joint[axis] * st_cfg.mot[motor].steps_per_unit;
			}
		}
	}
*/
}

/*
 * _inverse_kinematics() - inverse kinematics - example is for a cartesian machine
 *
 *	You can glue in inverse kinematics here, but be aware of time budget constrants.
 *	This function is run during the _exec() portion of the cycle and will therefore
 *	be run once per interpolation segment. The total time for the segment load,
 *	including the inverse kinematics transformation cannot exceed the segment time,
 *	and ideally should be no more than 25-50% of the segment time. Currently segments
 *	run avery 5 ms, but this might be lowered. To profile this time look at the
 *	time it takes to complete the mp_exec_move() function.
 */
/*
static void _inverse_kinematics(float travel[], float joint[])
{
	for (uint8_t i=0; i<AXES; i++) {
		joint[i] = travel[i];
	}
}
*/

#ifdef __cplusplus
}
#endif
