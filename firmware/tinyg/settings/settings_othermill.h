/*
 * settings_othermill.h - Other Machine Company Mini Milling Machine
 * This file is part of the TinyG project
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
 *		 are in hardware.h  Application settings that also shouldn't need
 *		 to be changed are in tinyg.h
 */

/***********************************************************************/
/**** Otherlab OtherMill profile ***************************************/
/***********************************************************************/

// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Initializing configs to OMC OtherMill settings"

#define JERK_MAX					500			// 500 million mm/(min^3)
#define JERK_HOMING					1000		// 1000 million mm/(min^3)		// Jerk during homing needs to stop *fast*
#define JUNCTION_DEVIATION			0.01		// default value, in mm
#define JUNCTION_ACCELERATION		100000		// centripetal acceleration around corners
#define LATCH_VELOCITY				25			// reeeeally slow for accuracy

// WARNING: Older Othermill machines use a 15deg can stack for their Z axis.
// new machines use a stepper which has the same config as the other axis.
#define HAS_CANSTACK_Z_AXIS 0

// *** settings.h overrides ***
// Note: there are some commented test values below

#undef MOTOR_POWER_MODE
#define MOTOR_POWER_MODE MOTOR_POWERED_IN_CYCLE

#undef  STATUS_REPORT_DEFAULTS
#define STATUS_REPORT_DEFAULTS  "mpox","mpoy","mpoz","mpoa","ofsx","ofsy","ofsz","ofsa","unit","stat","coor","momo","dist","home","hold","macs","cycs","mots","plan","prbe"

#undef	SWITCH_TYPE
#define SWITCH_TYPE 				SW_TYPE_NORMALLY_CLOSED

#undef	COMM_MODE
#define COMM_MODE					JSON_MODE

#undef	JSON_VERBOSITY
#define JSON_VERBOSITY				JV_CONFIGS		// one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE

#undef  JSON_FOOTER_DEPTH
#define JSON_FOOTER_DEPTH			0				// 0 = new style, 1 = old style

#undef  JSON_SYNTAX_MODE
#define JSON_SYNTAX_MODE 			JSON_SYNTAX_STRICT

#undef	QUEUE_REPORT_VERBOSITY
#define QUEUE_REPORT_VERBOSITY		QR_SINGLE

#undef	STATUS_REPORT_VERBOSITY
#define STATUS_REPORT_VERBOSITY		SR_FILTERED

#undef COM_ENABLE_FLOW_CONTROL
#define COM_ENABLE_FLOW_CONTROL		FLOW_CONTROL_XON

#undef GCODE_DEFAULT_COORD_SYSTEM
#undef GCODE_DEFAULT_UNITS
#undef GCODE_DEFAULT_PLANE
#undef GCODE_DEFAULT_COORD_SYSTEM
#undef GCODE_DEFAULT_PATH_CONTROL
#undef GCODE_DEFAULT_DISTANCE_MODE

#define GCODE_DEFAULT_UNITS			MILLIMETERS		// MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE			CANON_PLANE_XY	// CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM	G55				// G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL 	PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_MODE

// *** motor settings ***

#define M4_MOTOR_MAP 			AXIS_X				// 1ma
#define M4_STEP_ANGLE 			1.8					// 1sa
#define M4_TRAVEL_PER_REV 		5.08				// 1tr
#define M4_MICROSTEPS 			8					// 1mi		1,2,4,8
#define M4_POLARITY 			0					// 1po		0=normal, 1=reversed
#define M4_POWER_MODE 			MOTOR_POWER_MODE	// 1pm		TRUE=low power idle enabled
#define M4_POWER_LEVEL			MOTOR_POWER_LEVEL	// v9 only

#define M3_MOTOR_MAP 			AXIS_Y
#define M3_STEP_ANGLE 			1.8
#define M3_TRAVEL_PER_REV 		5.08
#define M3_MICROSTEPS 			8
#define M3_POLARITY 			1
#define M3_POWER_MODE 			MOTOR_POWER_MODE
#define M3_POWER_LEVEL			MOTOR_POWER_LEVEL

#define M2_MOTOR_MAP 			AXIS_Z
#if HAS_CANSTACK_Z_AXIS
#define M2_STEP_ANGLE 			15
#define M2_TRAVEL_PER_REV 		1.27254
#else
#define M2_STEP_ANGLE 			1.8
#define M2_TRAVEL_PER_REV 		5.08
#endif
#define M2_MICROSTEPS 			8
#define M2_POLARITY 			1
#define M2_POWER_MODE 			MOTOR_POWER_MODE
#define M2_POWER_LEVEL			MOTOR_POWER_LEVEL

#define M1_MOTOR_MAP 			AXIS_A
#define M1_STEP_ANGLE 			1.8
#define M1_TRAVEL_PER_REV 		360					// degrees moved per motor rev
#define M1_MICROSTEPS 			8
#define M1_POLARITY 			1
#define M1_POWER_MODE 			MOTOR_POWER_MODE
#define M1_POWER_LEVEL			MOTOR_POWER_LEVEL

#define M5_POWER_LEVEL			MOTOR_POWER_LEVEL
#define M6_POWER_LEVEL			MOTOR_POWER_LEVEL

// *** axis settings ***

#define X_AXIS_MODE 			AXIS_STANDARD		// xam		see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX 			1500 				// xvm		G0 max velocity in mm/min
#define X_FEEDRATE_MAX 			X_VELOCITY_MAX		// xfr 		G1 max feed rate in mm/min
#define X_TRAVEL_MIN			0					// xtn		minimum travel for soft limits
#define X_TRAVEL_MAX 			138					// xtr		travel between switches or crashes
#define X_JERK_MAX 				JERK_MAX			// xjm
#define X_JUNCTION_DEVIATION	JUNCTION_DEVIATION	// xjd
#define X_SWITCH_MODE_MIN 		SW_MODE_HOMING		// xsn		SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_LIMIT, SW_MODE_HOMING_LIMIT
#define X_SWITCH_MODE_MAX 		SW_MODE_DISABLED	// xsx		SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_LIMIT, SW_MODE_HOMING_LIMIT
#define X_SEARCH_VELOCITY 		(X_FEEDRATE_MAX/3)	// xsv
#define X_LATCH_VELOCITY 		LATCH_VELOCITY		// xlv		mm/min
#define X_LATCH_BACKOFF 		5					// xlb		mm
#define X_ZERO_BACKOFF 			0					// xzb		mm
#define X_JERK_HOMING			JERK_HOMING			// xjh

#define Y_AXIS_MODE 			AXIS_STANDARD
#define Y_VELOCITY_MAX 			X_VELOCITY_MAX
#define Y_FEEDRATE_MAX 			Y_VELOCITY_MAX
#define Y_TRAVEL_MIN			0
#define Y_TRAVEL_MAX 			115
#define Y_JERK_MAX 				JERK_MAX
#define Y_JUNCTION_DEVIATION 	JUNCTION_DEVIATION
#define Y_SWITCH_MODE_MIN		SW_MODE_HOMING
#define Y_SWITCH_MODE_MAX		SW_MODE_DISABLED
#define Y_SEARCH_VELOCITY 		(Y_FEEDRATE_MAX/3)
#define Y_LATCH_VELOCITY 		LATCH_VELOCITY
#define Y_LATCH_BACKOFF 		5
#define Y_ZERO_BACKOFF 			3
#define Y_JERK_HOMING			JERK_HOMING

#define Z_AXIS_MODE 			AXIS_STANDARD
#if HAS_CANSTACK_Z_AXIS
#define Z_VELOCITY_MAX 			1000
#else
#define Z_VELOCITY_MAX 			X_VELOCITY_MAX
#endif
#define Z_FEEDRATE_MAX 			Z_VELOCITY_MAX
#define Z_TRAVEL_MIN			-70
#define Z_TRAVEL_MAX 			0
#define Z_JERK_MAX 				JERK_MAX			// 200 million
#define Z_JUNCTION_DEVIATION 	JUNCTION_DEVIATION
#define Z_SWITCH_MODE_MIN		SW_MODE_DISABLED
#define Z_SWITCH_MODE_MAX		SW_MODE_HOMING
#define Z_SEARCH_VELOCITY 		(Z_FEEDRATE_MAX/3)
#define Z_LATCH_VELOCITY 		LATCH_VELOCITY
#define Z_LATCH_BACKOFF 		5
#define Z_ZERO_BACKOFF 			0
#define Z_JERK_HOMING			JERK_HOMING

// Rotary values are chosen to make the motor react the same as X for testing
#define A_AXIS_MODE 			AXIS_RADIUS
#define A_VELOCITY_MAX 			((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360) // set to the same speed as X axis
#define A_FEEDRATE_MAX 			A_VELOCITY_MAX
#define A_TRAVEL_MIN			-1										// min/max the same means infinite, no limit
#define A_TRAVEL_MAX 			-1
#define A_JERK_MAX 				(X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#define A_JUNCTION_DEVIATION	JUNCTION_DEVIATION
#define A_RADIUS 				(M1_TRAVEL_PER_REV/(2*3.14159628))
#define A_SWITCH_MODE_MIN 		SW_MODE_HOMING
#define A_SWITCH_MODE_MAX 		SW_MODE_DISABLED
#define A_SEARCH_VELOCITY 		600
#define A_LATCH_VELOCITY 		100
#define A_LATCH_BACKOFF 		5
#define A_ZERO_BACKOFF 			2
#define A_JERK_HOMING			A_JERK_MAX

#define B_AXIS_MODE 			AXIS_DISABLED	// DISABLED
#define B_VELOCITY_MAX 			((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360)
#define B_FEEDRATE_MAX 			B_VELOCITY_MAX
#define B_TRAVEL_MIN			-1
#define B_TRAVEL_MAX 			-1
#define B_JERK_MAX 				(X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#define B_JUNCTION_DEVIATION 	JUNCTION_DEVIATION
#define B_RADIUS 				(M1_TRAVEL_PER_REV/(2*3.14159628))
#define B_SWITCH_MODE_MIN 		SW_MODE_HOMING
#define B_SWITCH_MODE_MAX 		SW_MODE_DISABLED
#define B_SEARCH_VELOCITY 		600
#define B_LATCH_VELOCITY 		100
#define B_LATCH_BACKOFF 		5
#define B_ZERO_BACKOFF 			2
#define B_JERK_HOMING			B_JERK_MAX

#define C_AXIS_MODE 			AXIS_DISABLED	// DISABLED
#define C_VELOCITY_MAX 			((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360)
#define C_FEEDRATE_MAX 			C_VELOCITY_MAX
#define C_TRAVEL_MIN			-1
#define C_TRAVEL_MAX 			-1
#define C_JERK_MAX 				(X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#define C_JUNCTION_DEVIATION	JUNCTION_DEVIATION
#define C_RADIUS				(M1_TRAVEL_PER_REV/(2*3.14159628))
#define C_SWITCH_MODE_MIN 		SW_MODE_HOMING
#define C_SWITCH_MODE_MAX 		SW_MODE_DISABLED
#define C_SEARCH_VELOCITY 		600
#define C_LATCH_VELOCITY 		100
#define C_LATCH_BACKOFF 		5
#define C_ZERO_BACKOFF 			2
#define C_JERK_HOMING			C_JERK_MAX

// *** PWM SPINDLE CONTROL ***

#define P1_PWM_FREQUENCY		100					// in Hz
#define P1_CW_SPEED_LO			7900				// in RPM (arbitrary units)
#define P1_CW_SPEED_HI			12800
#define P1_CW_PHASE_LO			0.13				// phase [0..1]
#define P1_CW_PHASE_HI			0.17
#define P1_CCW_SPEED_LO			0
#define P1_CCW_SPEED_HI			0
#define P1_CCW_PHASE_LO			0.1
#define P1_CCW_PHASE_HI			0.1
#define P1_PWM_PHASE_OFF		0.1

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
#define G55_C_OFFSET -20         // this is where we currently store the tool offset

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
