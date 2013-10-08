/*
 * settings_othercutter.h - Otherlab Cardboard Cutter
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
/* Note: The values in this file are the default settings that are loaded
 * 		 into a virgin EEPROM, and can be changed using the config commands.
 *		 After initial load the EEPROM values (or changed values) are used.
 *
 *		 System and hardware settings that you shouldn't need to change 
 *		 are in system.h  Application settings that also shouldn't need 
 *		 to be changed are in tinyg.h
 */

/***********************************************************************/
/**** Otherlab profile *************************************************/
/***********************************************************************/

// ***> NOTE: The init message must be a single line with no CRs or LFs 
#define INIT_MESSAGE "Initializing configs to Othercutter settings"

#define JERK_MAX				900	// yes, that's "900,000,000" mm/(min^3)
#define JUNCTION_DEVIATION		0.01		// default value, in mm
#define JUNCTION_ACCELERATION	100000		// centripetal acceleration around corners

// *** settings.h overrides ***

#undef  SR_DEFAULTS
#define SR_DEFAULTS "line","mpox","mpoy","mpoz","mpoa","feed","vel","unit","coor","dist","frmo","momo","stat"

#undef	SWITCH_TYPE
#define SWITCH_TYPE 			SW_TYPE_NORMALLY_CLOSED

#undef	COMM_MODE
#define COMM_MODE				JSON_MODE

#undef	JSON_VERBOSITY
#define JSON_VERBOSITY			JV_CONFIGS

#undef	COM_ENABLE_QR
#define COM_ENABLE_QR			true

#undef 	QR_VERBOSITY
//#define QR_VERBOSITY			QR_FILTERED
#define QR_VERBOSITY			QR_VERBOSE

// *** motor settings ***

#define M1_MOTOR_MAP 		AXIS_X		// 1ma
#define M1_STEP_ANGLE 		1.8			// 1sa
#define M1_TRAVEL_PER_REV 	54.023		// 1tr
#define M1_MICROSTEPS 		8			// 1mi		1,2,4,8
#define M1_POLARITY 		1			// 1po		0=normal, 1=reversed
#define M1_POWER_MODE 		1			// 1pm		TRUE=low power idle enabled 

#define M2_MOTOR_MAP 		AXIS_Y
#define M2_STEP_ANGLE 		1.8
#define M2_TRAVEL_PER_REV 	34.314
#define M2_MICROSTEPS 		8
#define M2_POLARITY 		1
#define M2_POWER_MODE 		1			// hold

#define M3_MOTOR_MAP 		AXIS_Z
#define M3_STEP_ANGLE 		15
#define M3_TRAVEL_PER_REV 	2.438
#define M3_MICROSTEPS 		4
#define M3_POLARITY 		1
#define M3_POWER_MODE 		1			// z-axis leadscrew doesn't need hold, enable low power idle

#define M4_MOTOR_MAP 		AXIS_A
#define M4_STEP_ANGLE 		1.8
#define M4_TRAVEL_PER_REV 	180.0		// degrees moved per motor rev
#define M4_MICROSTEPS 		8
#define M4_POLARITY 		1			
#define M4_POWER_MODE 		1			// hold

// *** axis settings ***

#define X_AXIS_MODE 					AXIS_STANDARD		// xam		see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX 					15000 				// xvm		G0 max velocity in mm/min
#define X_FEEDRATE_MAX 					1000				// xfr 		G1 max feed rate in mm/min
#define X_TRAVEL_MAX 					320					// xtm		travel between switches or crashes
#define X_JERK_MAX 						JERK_MAX			// xjm
#define X_JUNCTION_DEVIATION			JUNCTION_DEVIATION	// xjd
#define X_SWITCH_MODE_MIN 				SW_MODE_HOMING		// xsn		SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_LIMIT, SW_MODE_HOMING_LIMIT
#define X_SWITCH_MODE_MAX 				SW_MODE_DISABLED	// xsx		SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_LIMIT, SW_MODE_HOMING_LIMIT
#define X_SEARCH_VELOCITY 				2000				// xsv
#define X_LATCH_VELOCITY 				500					// xlv		mm/min
#define X_LATCH_BACKOFF 				12					// xlb		mm
#define X_ZERO_BACKOFF 					5					// xzb		mm
#define X_JERK_HOMING					X_JERK_MAX			// xjh

#define Y_AXIS_MODE 					AXIS_STANDARD
#define Y_VELOCITY_MAX 					15000
#define Y_FEEDRATE_MAX 					1000
#define Y_TRAVEL_MAX 					-1					// Y-axis is infinite
#define Y_JERK_MAX 						JERK_MAX
#define Y_JUNCTION_DEVIATION 			JUNCTION_DEVIATION
#define Y_SWITCH_MODE_MIN				SW_MODE_DISABLED	// Y-axis is infinite
#define Y_SWITCH_MODE_MAX				SW_MODE_DISABLED
#define Y_SEARCH_VELOCITY 				1000
#define Y_LATCH_VELOCITY 				1000
#define Y_LATCH_BACKOFF 				1
#define Y_ZERO_BACKOFF 					0
#define Y_JERK_HOMING					Y_JERK_MAX

#define Z_AXIS_MODE 					AXIS_STANDARD
#define Z_VELOCITY_MAX 					10000
#define Z_FEEDRATE_MAX 					1000
#define Z_TRAVEL_MAX 					25
#define Z_JERK_MAX 						JERK_MAX			// 200 Million
#define Z_JUNCTION_DEVIATION 			JUNCTION_DEVIATION
#define Z_SWITCH_MODE_MIN				SW_MODE_DISABLED
#define Z_SWITCH_MODE_MAX				SW_MODE_HOMING
#define Z_SEARCH_VELOCITY 				1000.0
#define Z_LATCH_VELOCITY 				500.0
#define Z_LATCH_BACKOFF 				12
#define Z_ZERO_BACKOFF 					20.5				// based on blade geometry
#define Z_JERK_HOMING					Z_JERK_MAX

#define A_AXIS_MODE 					AXIS_STANDARD
#define A_VELOCITY_MAX 					60000.0				// deg/min
#define A_FEEDRATE_MAX 					7200.0				// deg/min
#define A_TRAVEL_MAX 					375.0
#define A_JERK_MAX 						24000			// yes, 24 Billion
#define A_JUNCTION_DEVIATION 			0.1
#define A_RADIUS 						1.0					// deg
#define A_SWITCH_MODE_MIN				SW_MODE_HOMING		// disable limit switch halt
#define A_SWITCH_MODE_MAX				SW_MODE_DISABLED
#define A_SEARCH_VELOCITY 				6000.0				// deg/min
#define A_LATCH_VELOCITY 				600.0				// deg/min
#define A_LATCH_BACKOFF 				15.0				// deg
#define A_ZERO_BACKOFF 					1					// deg
#define A_JERK_HOMING					A_JERK_MAX

#define B_AXIS_MODE 					AXIS_DISABLED
#define B_VELOCITY_MAX 					3600
#define B_FEEDRATE_MAX 					B_VELOCITY_MAX
#define B_TRAVEL_MAX 					-1
#define B_JERK_MAX 						JERK_MAX
#define B_JUNCTION_DEVIATION 			JUNCTION_DEVIATION
#define B_RADIUS 						1

#define C_AXIS_MODE 					AXIS_DISABLED
#define C_VELOCITY_MAX 					3600
#define C_FEEDRATE_MAX 					C_VELOCITY_MAX
#define C_TRAVEL_MAX 					-1
#define C_JERK_MAX 						JERK_MAX
#define C_JUNCTION_DEVIATION 			JUNCTION_DEVIATION
#define C_RADIUS 						1

// *** PWM SPINDLE CONTROL ***

#define P1_PWM_FREQUENCY                100					// in Hz
#define P1_CW_SPEED_LO                  1000				// in RPM (arbitrary units)
#define P1_CW_SPEED_HI                  2000
#define P1_CW_PHASE_LO                  0.125				// phase [0..1]
#define P1_CW_PHASE_HI                  0.2
#define P1_CCW_SPEED_LO                 1000
#define P1_CCW_SPEED_HI                 2000
#define P1_CCW_PHASE_LO                 0.125
#define P1_CCW_PHASE_HI                 0.2
#define P1_PWM_PHASE_OFF                0.1

// *** DEFAULT COORDINATE SYSTEM OFFSETS ***

#define G54_X_OFFSET 0			// G54 is traditionally set to all zeros
#define G54_Y_OFFSET 0
#define G54_Z_OFFSET 0
#define G54_A_OFFSET 0
#define G54_B_OFFSET 0
#define G54_C_OFFSET 0

#define G55_X_OFFSET 0			// but the again, so is everyting else (at least for start)
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
