/*
 * kinematics.c - inverse kinematics routines
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdlib.h>
#include <stdio.h>
#include "tinyg.h"
#include "gcode.h"
#include "config.h"
#include "kinematics.h"

uint8_t _ik_cartesian_kinematics(double travel[], double joint[], double microseconds);

/*
 * ik_kinematics() - wrapper routine for inverse kinematics
 *
 *	Calls kinematics function(s). 
 *	Performs axis mapping & conversion of length units to steps (see note)
 *	Also deals with inhibited axes
 *
 *	Note: The reason steps are returned as doubles (as opposed to, say,
 *		  uint32_t) is to accommodate fractional steps. The DDA deals 
 *		  with fractional step values in order to get smoothest possible
 *		  operation. See motor_queue.c and steppers.c for details.
 */

uint8_t ik_kinematics(double travel[], double steps[], double microseconds)
{
	uint8_t i,j;
	double joint[AXES];

	// inverse kinematics --> insert kinematics transformations here
	_ik_cartesian_kinematics(travel, joint, microseconds);

	// Map motors to axes and convert length units to steps
	// Most of the conversion math has already been done in steps_per_unit
	// which takes axis travel, step angle and microsteps into account.
	for (i=0; i<AXES; i++) {
		if (cfg.a[i].axis_mode == AXIS_INHIBITED) {
			joint[i] = 0;
		}
		for (j=0; j<MOTORS; j++) {
			if (cfg.m[j].motor_map == i) {
				steps[j] = joint[i] * cfg.m[j].steps_per_unit;
			}
		}
	}
	return (TG_OK);
}

/*
 * _ik_cartesian_kinematics() - inverse kinematics for cartesian machines
 *
 *	Provides inverse kinematics for cartesian machines. Which is none.
 */

uint8_t _ik_cartesian_kinematics(double travel[], double joint[], double microseconds)
{
	uint8_t i;

	for (i=0; i<AXES; i++) {
		joint[i] = travel[i];
	}	
	return (TG_OK);
}


//############## UNIT TESTS ################

#ifdef __UNIT_TESTS

//void _mp_test_medium_line(void);

void ik_unit_tests()
{
//	_mp_test_medium_line();
}

#endif
