/*
 * settings_sacidu93.h - Sacidu93 machine profile
 * Part of TinyG project
 *
 * Copyright (c) 2011 - 2012 Alden S. Hart Jr.
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
/* Note: The values in this file are the default settings that are loaded
 * 		 into a virgin EEPROM, and can be changed using the config commands.
 *		 After initial load the EEPROM values (or changed values) are used.
 *
 *		 System and hardware settings that you shouldn't need to change 
 *		 are in system.h  Application settings that also shouldn't need 
 *		 to be changed are in tinyg.h
 */

// ***> NOTE: The init message must be a single line with no CRs or LFs 
#define INIT_MESSAGE "Initializing configs to Sacidu93 profile"

#undef	STATUS_REPORT_INTERVAL_MS
#define STATUS_REPORT_INTERVAL_MS	0	// in milliseconds - 0=off

#define JUNCTION_ACCELERATION 150000	// centripetal acceleration around corners
//#define JUNCTION_ACCELERATION 20000000	// centripetal acceleration around corners

#define SWITCH_TYPE SW_TYPE_NORMALLY_OPEN

// motor values

#define M1_MOTOR_MAP X					// 1ma
#define M1_STEP_ANGLE 0.72				// 1sa
#define M1_TRAVEL_PER_REV 5.00			// 1tr
#define M1_MICROSTEPS 1					// 1mi		1,2,4,8
#define M1_POLARITY 1					// 1po		0=normal, 1=reversed
#define M1_POWER_MODE 0					// 1pm		TRUE=low power idle enabled 

#define M2_MOTOR_MAP Y
#define M2_STEP_ANGLE 0.72
#define M2_TRAVEL_PER_REV 5.00
#define M2_MICROSTEPS 1
#define M2_POLARITY 1
#define M2_POWER_MODE 0

#define M3_MOTOR_MAP Z
#define M3_STEP_ANGLE 0.72
#define M3_TRAVEL_PER_REV 5.00
#define M3_MICROSTEPS 1
#define M3_POLARITY 0
#define M3_POWER_MODE 0

#define M4_MOTOR_MAP A
#define M4_STEP_ANGLE 0.72
#define M4_TRAVEL_PER_REV 5.00			// degrees per motor rev - 1:1 gearing
#define M4_MICROSTEPS 8
#define M4_POLARITY 1
#define M4_POWER_MODE 0

// axis values							// see canonical_machine.h cmAxisMode for valid values
#define X_AXIS_MODE AXIS_STANDARD		// xam
#define X_FEEDRATE_MAX 3000				// xfr 		G1 max feed rate in mm/min
#define X_VELOCITY_MAX 1000 			// xvm
#define X_TRAVEL_MAX 580				// travel between switches or crashes
#define X_JERK_MAX 20000000				// xjm
#define X_JUNCTION_DEVIATION 0.05		// xjd
#define X_SWITCH_MODE_MIN		SW_MODE_HOMING		// xsn		SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_HOMING_LIMIT, SW_MODE_LIMIT
#define X_SWITCH_MODE_MAX		SW_MODE_LIMIT		// xsx		SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_HOMING_LIMIT, SW_MODE_LIMIT
#define X_SEARCH_VELOCITY -600			// move in negative direction
#define X_LATCH_VELOCITY 80				// mm/min
#define X_LATCH_BACKOFF 5				// mm
#define X_ZERO_BACKOFF 2				// mm

#define Y_AXIS_MODE AXIS_STANDARD
#define Y_FEEDRATE_MAX 3000
#define Y_VELOCITY_MAX 1000
#define Y_TRAVEL_MAX 385
#define Y_JERK_MAX 20000000
#define Y_JUNCTION_DEVIATION 0.05
#define Y_SWITCH_MODE_MIN		SW_MODE_HOMING
#define Y_SWITCH_MODE_MAX		SW_MODE_LIMIT
#define Y_SEARCH_VELOCITY -600
#define Y_LATCH_VELOCITY 80
#define Y_LATCH_BACKOFF 5
#define Y_ZERO_BACKOFF 2

#define Z_AXIS_MODE AXIS_STANDARD
#define Z_FEEDRATE_MAX 3000
#define Z_VELOCITY_MAX 1000
#define Z_TRAVEL_MAX 90
#define Z_JERK_MAX 20000000
#define Z_JUNCTION_DEVIATION 0.05
#define Z_SWITCH_MODE_MIN		SW_MODE_HOMING
#define Z_SWITCH_MODE_MAX		SW_MODE_LIMIT
#define Z_SEARCH_VELOCITY 600
#define Z_LATCH_VELOCITY 80
#define Z_LATCH_BACKOFF 5
#define Z_ZERO_BACKOFF 2

#define A_AXIS_MODE AXIS_STANDARD
#define A_FEEDRATE_MAX 3600
#define A_VELOCITY_MAX 3600
#define A_TRAVEL_MAX -1
#define A_JERK_MAX 20000000
#define A_JUNCTION_DEVIATION 0.05
#define A_RADIUS 1
#define A_SWITCH_MODE_MIN		SW_MODE_HOMING
#define A_SWITCH_MODE_MAX		SW_MODE_LIMIT
#define A_SEARCH_VELOCITY -600
#define A_LATCH_VELOCITY 100
#define A_LATCH_BACKOFF -5
#define A_ZERO_BACKOFF 2

#define B_AXIS_MODE AXIS_DISABLED
#define B_FEEDRATE_MAX 200
#define B_VELOCITY_MAX 250
#define B_TRAVEL_MAX -1
#define B_JERK_MAX 20000000
#define B_JUNCTION_DEVIATION 0.05
#define B_RADIUS 10
#define B_SEARCH_VELOCITY 32
#define B_LATCH_VELOCITY 0.040
#define B_LATCH_BACKOFF -5
#define B_ZERO_BACKOFF -2

#define C_AXIS_MODE AXIS_DISABLED
#define C_FEEDRATE_MAX 200
#define C_VELOCITY_MAX 250
#define C_TRAVEL_MAX -1
#define C_JERK_MAX 20000000
#define C_JUNCTION_DEVIATION 0.05
#define C_RADIUS 10
#define C_SEARCH_VELOCITY 30
#define C_LATCH_VELOCITY 3600
#define C_LATCH_BACKOFF 5
#define C_ZERO_BACKOFF 2


/**** DEFAULT COORDINATE SYSTEM OFFSETS *************************************/

#define G54_X_OFFSET 0			// G54 is traditionally set to all zeros
#define G54_Y_OFFSET 0
#define G54_Z_OFFSET 0
#define G54_A_OFFSET 0
#define G54_B_OFFSET 0
#define G54_C_OFFSET 0

#define G55_X_OFFSET 0
#define G55_Y_OFFSET 0
#define G55_Z_OFFSET 0
#define G55_A_OFFSET 0
#define G55_B_OFFSET 0
#define G55_C_OFFSET 0

#define G56_X_OFFSET 0
#define G56_Y_OFFSET 0
#define G56_Z_OFFSET 0
#define G56_A_OFFSET 0
#define G56_B_OFFSET 0
#define G56_C_OFFSET 0

#define G57_X_OFFSET 0
#define G57_Y_OFFSET 0
#define G57_Z_OFFSET 0
#define G57_A_OFFSET 0
#define G57_B_OFFSET 0
#define G57_C_OFFSET 0

#define G58_X_OFFSET 0
#define G58_Y_OFFSET 0
#define G58_Z_OFFSET 0
#define G58_A_OFFSET 0
#define G58_B_OFFSET 0
#define G58_C_OFFSET 0

#define G59_X_OFFSET 0
#define G59_Y_OFFSET 0
#define G59_Z_OFFSET 0
#define G59_A_OFFSET 0
#define G59_B_OFFSET 0
#define G59_C_OFFSET 0
