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
 */

/***********************************************************************/
/**** Otherlab OtherMill profile ***************************************/
/***********************************************************************/
// ***> NOTE: The init message must be a single line with no CRs or LFs
#define INIT_MESSAGE "Initializing configs to OMC OtherMill settings"

//**** GLOBAL / GENERAL SETTINGS ******************************************************

#define JUNCTION_ACCELERATION		100000		// centripetal acceleration around corners
#define SWITCH_TYPE                 SW_TYPE_NORMALLY_CLOSED	// one of: SW_TYPE_NORMALLY_OPEN, SW_TYPE_NORMALLY_CLOSED

#define JUNCTION_AGGRESSION         0.75					// cornering - between 0.05 and 1.00 (max)
#define CHORDAL_TOLERANCE           0.01					// chordal accuracy for arc drawing (in mm)

#define SOFT_LIMIT_ENABLE           0						// 0=off, 1=on
#define HARD_LIMIT_ENABLE           1						// 0=off, 1=on
#define SAFETY_INTERLOCK_ENABLE     1						// 0=off, 1=on

#define SPINDLE_ENABLE_POLARITY     1                       // 0=active low, 1=active high
#define SPINDLE_DIR_POLARITY        0                       // 0=clockwise is low, 1=clockwise is high
#define SPINDLE_PAUSE_ON_HOLD       true
#define SPINDLE_DWELL_TIME          1.0

#define COOLANT_MIST_POLARITY       1                       // 0=active low, 1=active high
#define COOLANT_FLOOD_POLARITY      1                       // 0=active low, 1=active high
#define COOLANT_PAUSE_ON_HOLD       false

// Communications and reporting settings

#define TEXT_VERBOSITY              TV_VERBOSE              // one of: TV_SILENT, TV_VERBOSE
#define COMM_MODE                   JSON_MODE               // one of: TEXT_MODE, JSON_MODE

#define XIO_EXPAND_CR               false                   // serial IO settings (AVR only)
#define XIO_ENABLE_ECHO             false
#define XIO_ENABLE_FLOW_CONTROL     FLOW_CONTROL_XON        // FLOW_CONTROL_OFF, FLOW_CONTROL_XON, FLOW_CONTROL_RTS

#define JSON_VERBOSITY              JV_MESSAGES             // one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
#define JSON_SYNTAX_MODE            JSON_SYNTAX_STRICT      // one of JSON_SYNTAX_RELAXED, JSON_SYNTAX_STRICT

#define QUEUE_REPORT_VERBOSITY		QR_OFF		            // one of: QR_OFF, QR_SINGLE, QR_TRIPLE

#define STATUS_REPORT_VERBOSITY     SR_FILTERED             // one of: SR_OFF, SR_FILTERED, SR_VERBOSE
#define STATUS_REPORT_MIN_MS        100                     // milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS   500                     // milliseconds - set $SV=0 to disable

#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","feed","vel","unit","coor","dist","frmo","stat"
//#define STATUS_REPORT_DEFAULTS "mpox","mpoy","mpoz","mpoa","ofsx","ofsy","ofsz","ofsa","unit","stat","coor","momo","dist","home","hold","macs","cycs","mots","plan","prbe"
//#define STATUS_REPORT_DEFAULTS "line","posx","posy","posz","posa","feed","vel","unit","coor","dist","admo","frmo","momo","stat"
// Alternate SRs that report in drawable units
//#define STATUS_REPORT_DEFAULTS "line","vel","mpox","mpoy","mpoz","mpoa","coor","ofsa","ofsx","ofsy","ofsz","dist","unit","stat","homz","homy","homx","momo"

// Gcode startup defaults
#define GCODE_DEFAULT_UNITS         MILLIMETERS             // MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE         CANON_PLANE_XY          // CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM  G54                     // G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL  PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_MODE


// *** Motor settings ************************************************************************************
// WARNING: Older Othermill machines use a 15deg can stack for their Z axis.
// new machines use a stepper which has the same config as the other axis.
#define HAS_CANSTACK_Z_AXIS         0

#define MOTOR_POWER_MODE MOTOR_POWERED_IN_CYCLE
#define MOTOR_POWER_TIMEOUT         2.00                    // motor power timeout in seconds
#define MOTOR_POWER_LEVEL           0.25

#define M4_MOTOR_MAP 			    AXIS_X				    // 1ma
#define M4_STEP_ANGLE 			    1.8					    // 1sa
#define M4_TRAVEL_PER_REV 		    5.08				    // 1tr
#define M4_MICROSTEPS 			    8					    // 1mi  1,2,4,8
#define M4_POLARITY 			    0					    // 1po  0=normal, 1=reversed
#define M4_POWER_MODE 			    MOTOR_POWER_MODE	    // 1pm  TRUE=low power idle enabled
#define M4_POWER_LEVEL			    MOTOR_POWER_LEVEL	    // v9 only

#define M3_MOTOR_MAP 			    AXIS_Y
#define M3_STEP_ANGLE 			    1.8
#define M3_TRAVEL_PER_REV 		    5.08
#define M3_MICROSTEPS 			    8
#define M3_POLARITY 			    1
#define M3_POWER_MODE 			    MOTOR_POWER_MODE
#define M3_POWER_LEVEL			    MOTOR_POWER_LEVEL

#define M2_MOTOR_MAP 			    AXIS_Z
#if HAS_CANSTACK_Z_AXIS
#define M2_STEP_ANGLE 			    15
#define M2_TRAVEL_PER_REV 		    1.27254
#else
#define M2_STEP_ANGLE 			    1.8
#define M2_TRAVEL_PER_REV 		    5.08
#endif
#define M2_MICROSTEPS 			    8
#define M2_POLARITY 			    1
#define M2_POWER_MODE 			    MOTOR_POWER_MODE
#define M2_POWER_LEVEL			    MOTOR_POWER_LEVEL

#define M1_MOTOR_MAP 			    AXIS_A
#define M1_STEP_ANGLE 			    1.8
#define M1_TRAVEL_PER_REV 		    360					// degrees moved per motor rev
#define M1_MICROSTEPS 			    8
#define M1_POLARITY 			    1
#define M1_POWER_MODE 			    MOTOR_POWER_MODE
#define M1_POWER_LEVEL			    MOTOR_POWER_LEVEL


// *** Axis settings **********************************************************************************

#define JERK_MAX					500			            // 500 million mm/(min^3)
#define JERK_HIGH					1000		            // 1000 million mm/(min^3) // Jerk during homing needs to stop *fast*
#define JUNCTION_DEVIATION			0.01		            // default value, in mm
#define LATCH_VELOCITY				25			            // reeeeally slow for accuracy

#define X_AXIS_MODE 			    AXIS_STANDARD		    // xam  see canonical_machine.h cmAxisMode for valid values
#define X_VELOCITY_MAX 			    1500 				    // xvm  G0 max velocity in mm/min
#define X_FEEDRATE_MAX 			    X_VELOCITY_MAX		    // xfr  G1 max feed rate in mm/min
#define X_TRAVEL_MIN			    0					    // xtn  minimum travel for soft limits
#define X_TRAVEL_MAX 			    138					    // xtr  travel between switches or crashes
#define X_JERK_MAX 				    JERK_MAX			    // xjm
#define X_JERK_HIGH			        JERK_HIGH			    // xjh
#define X_JUNCTION_DEVIATION	    JUNCTION_DEVIATION	    // xjd
#define X_SWITCH_MODE_MIN 		    SW_MODE_HOMING		    // xsn  SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_LIMIT, SW_MODE_HOMING_LIMIT
#define X_SWITCH_MODE_MAX 		    SW_MODE_DISABLED	    // xsx  SW_MODE_DISABLED, SW_MODE_HOMING, SW_MODE_LIMIT, SW_MODE_HOMING_LIMIT
#define X_HOMING_INPUT              1                       // xhi  input used for homing or 0 to disable
#define X_HOMING_DIR                0                       // xhd  0=search moves negative, 1= search moves positive
#define X_SEARCH_VELOCITY 		    (X_FEEDRATE_MAX/3)	    // xsv
#define X_LATCH_VELOCITY 		    LATCH_VELOCITY		    // xlv  mm/min
#define X_LATCH_BACKOFF 		    2					    // xlb  mm
#define X_ZERO_BACKOFF 			    1					    // xzb  mm

#define Y_AXIS_MODE 			    AXIS_STANDARD
#define Y_VELOCITY_MAX 			    X_VELOCITY_MAX
#define Y_FEEDRATE_MAX 			    Y_VELOCITY_MAX
#define Y_TRAVEL_MIN			    0
#define Y_TRAVEL_MAX 			    115
#define Y_JERK_MAX 				    JERK_MAX
#define Y_JERK_HIGH			        JERK_HIGH
#define Y_JUNCTION_DEVIATION 	    JUNCTION_DEVIATION
#define Y_SWITCH_MODE_MIN		    SW_MODE_HOMING
#define Y_SWITCH_MODE_MAX		    SW_MODE_DISABLED
#define Y_HOMING_INPUT              3
#define Y_HOMING_DIR                0
#define Y_SEARCH_VELOCITY 		    (Y_FEEDRATE_MAX/3)
#define Y_LATCH_VELOCITY 		    LATCH_VELOCITY
#define Y_LATCH_BACKOFF 		    3
#define Y_ZERO_BACKOFF 			    3

#define Z_AXIS_MODE 			    AXIS_STANDARD
#if HAS_CANSTACK_Z_AXIS
#define Z_VELOCITY_MAX 			    1000
#else
#define Z_VELOCITY_MAX 			    X_VELOCITY_MAX
#endif
#define Z_FEEDRATE_MAX 			    Z_VELOCITY_MAX
#define Z_TRAVEL_MIN			    -70
#define Z_TRAVEL_MAX 			    0
#define Z_JERK_MAX 				    JERK_MAX
#define Z_JERK_HIGH			        JERK_HIGH
#define Z_JUNCTION_DEVIATION 	    JUNCTION_DEVIATION
#define Z_SWITCH_MODE_MIN		    SW_MODE_DISABLED
#define Z_SWITCH_MODE_MAX		    SW_MODE_HOMING
#define Z_HOMING_INPUT              6
#define Z_HOMING_DIR                1
#define Z_SEARCH_VELOCITY 		    (Z_FEEDRATE_MAX/3)
#define Z_LATCH_VELOCITY 		    LATCH_VELOCITY
#define Z_LATCH_BACKOFF 		    2
#define Z_ZERO_BACKOFF 			    0

// Rotary values are chosen to make the motor react the same as X for testing
#define A_AXIS_MODE 			    AXIS_RADIUS
#define A_VELOCITY_MAX 			    ((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360) // set to the same speed as X axis
#define A_FEEDRATE_MAX 			    A_VELOCITY_MAX
#define A_TRAVEL_MIN			    -1										// min/max the same means infinite, no limit
#define A_TRAVEL_MAX 			    -1
#define A_JERK_MAX 				    (X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#define A_JERK_HIGH			        A_JERK_MAX
#define A_JUNCTION_DEVIATION	    JUNCTION_DEVIATION
#define A_RADIUS 				    (M1_TRAVEL_PER_REV/(2*3.14159628))
#define A_SWITCH_MODE_MIN 		    SW_MODE_HOMING
#define A_SWITCH_MODE_MAX 		    SW_MODE_DISABLED
#define A_HOMING_INPUT              0
#define A_HOMING_DIR                0
#define A_SEARCH_VELOCITY 		    600
#define A_LATCH_VELOCITY 		    100
#define A_LATCH_BACKOFF 		    5
#define A_ZERO_BACKOFF 			    2

#define B_AXIS_MODE 			    AXIS_DISABLED	// DISABLED
#define B_VELOCITY_MAX 			    ((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360)
#define B_FEEDRATE_MAX 			    B_VELOCITY_MAX
#define B_TRAVEL_MIN			    -1
#define B_TRAVEL_MAX 			    -1
#define B_JERK_MAX 				    (X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#define B_JERK_HIGH			        B_JERK_MAX
#define B_JUNCTION_DEVIATION 	    JUNCTION_DEVIATION
#define B_RADIUS 				    (M1_TRAVEL_PER_REV/(2*3.14159628))
#define B_SWITCH_MODE_MIN 		    SW_MODE_HOMING
#define B_SWITCH_MODE_MAX 		    SW_MODE_DISABLED
#define B_HOMING_INPUT              0
#define B_HOMING_DIR                0
#define B_SEARCH_VELOCITY 		    600
#define B_LATCH_VELOCITY 		    100
#define B_LATCH_BACKOFF 		    5
#define B_ZERO_BACKOFF 			    2

#define C_AXIS_MODE 			    AXIS_DISABLED	// DISABLED
#define C_VELOCITY_MAX 			    ((X_VELOCITY_MAX/M1_TRAVEL_PER_REV)*360)
#define C_FEEDRATE_MAX 			    C_VELOCITY_MAX
#define C_TRAVEL_MIN			    -1
#define C_TRAVEL_MAX 			    -1
#define C_JERK_MAX 				    (X_JERK_MAX*(360/M1_TRAVEL_PER_REV))
#define C_JERK_HIGH			        C_JERK_MAX
#define C_JUNCTION_DEVIATION	    JUNCTION_DEVIATION
#define C_RADIUS				    (M1_TRAVEL_PER_REV/(2*3.14159628))
#define C_SWITCH_MODE_MIN 		    SW_MODE_HOMING
#define C_SWITCH_MODE_MAX 		    SW_MODE_DISABLED
#define C_HOMING_INPUT              0
#define C_HOMING_DIR                0
#define C_SEARCH_VELOCITY 		    600
#define C_LATCH_VELOCITY 		    100
#define C_LATCH_BACKOFF 		    5
#define C_ZERO_BACKOFF 			    2

//*** Input / output settings *************************************************************************
/*
    INPUT_MODE_DISABLED
    INPUT_ACTIVE_LOW    aka NORMALLY_OPEN
    INPUT_ACTIVE_HIGH   aka NORMALLY_CLOSED

    INPUT_ACTION_NONE
    INPUT_ACTION_STOP
    INPUT_ACTION_FAST_STOP
    INPUT_ACTION_HALT
    INPUT_ACTION_RESET

    INPUT_FUNCTION_NONE
    INPUT_FUNCTION_LIMIT
    INPUT_FUNCTION_INTERLOCK
    INPUT_FUNCTION_SHUTDOWN
    INPUT_FUNCTION_PANIC
*/
// Xmin on v8/v9 boards
#define DI1_MODE                    NORMALLY_CLOSED
#define DI1_ACTION                  INPUT_ACTION_STOP
#define DI1_FUNCTION                INPUT_FUNCTION_LIMIT

// Xmax
#define DI2_MODE                    NORMALLY_CLOSED
#define DI2_ACTION                  INPUT_ACTION_STOP
#define DI2_FUNCTION                INPUT_FUNCTION_LIMIT

// Ymin
#define DI3_MODE                    NORMALLY_CLOSED
#define DI3_ACTION                  INPUT_ACTION_STOP
#define DI3_FUNCTION                INPUT_FUNCTION_LIMIT

// Ymax
#define DI4_MODE                    NORMALLY_CLOSED
#define DI4_ACTION                  INPUT_ACTION_STOP
#define DI4_FUNCTION                INPUT_FUNCTION_LIMIT

// Zmin
#define DI5_MODE                    INPUT_ACTIVE_HIGH   // used for Z probe
#define DI5_ACTION                  INPUT_ACTION_NONE
#define DI5_FUNCTION                INPUT_FUNCTION_NONE

// Zmax
#define DI6_MODE                    NORMALLY_CLOSED
#define DI6_ACTION                  INPUT_ACTION_STOP
#define DI6_FUNCTION                INPUT_FUNCTION_LIMIT

// Amin
#define DI7_MODE                    INPUT_MODE_DISABLED
#define DI7_ACTION                  INPUT_ACTION_NONE
#define DI7_FUNCTION                INPUT_FUNCTION_NONE

// Amax
#define DI8_MODE                    INPUT_MODE_DISABLED
#define DI8_ACTION                  INPUT_ACTION_NONE
#define DI8_FUNCTION                INPUT_FUNCTION_NONE

// Hardware interlock input (v9 only)
#define DI9_MODE                    INPUT_MODE_DISABLED
#define DI9_ACTION                  INPUT_ACTION_NONE
#define DI9_FUNCTION                INPUT_FUNCTION_NONE


//*** Optional modules that may not be in every machine ***********************************************

// *** PWM SPINDLE CONTROL ***

#define P1_PWM_FREQUENCY		    100					// in Hz
#define P1_CW_SPEED_LO			    7900				// in RPM (arbitrary units)
#define P1_CW_SPEED_HI			    12800
#define P1_CW_PHASE_LO			    0.13				// phase [0..1]
#define P1_CW_PHASE_HI			    0.17
#define P1_CCW_SPEED_LO			    0
#define P1_CCW_SPEED_HI			    0
#define P1_CCW_PHASE_LO			    0.1
#define P1_CCW_PHASE_HI			    0.1
#define P1_PWM_PHASE_OFF		    0.1

//*** DEFAULT COORDINATE SYSTEM OFFSETS ***************************************************************
// Our convention is:
//	- leave G54 in machine coordinates to act as a persistent absolute coordinate system
//	- set G55 to be a zero in the middle of the table
//	- no action for the others

#define G54_X_OFFSET 0			// G54 is traditionally set to all zeros
#define G54_Y_OFFSET 0
#define G54_Z_OFFSET 0
#define G54_A_OFFSET 0
#define G54_B_OFFSET 0
#define G54_C_OFFSET 0

#define G55_X_OFFSET (X_TRAVEL_MAX/2)	// set g55 to middle of table
#define G55_Y_OFFSET (Y_TRAVEL_MAX/2)
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


//*** User-Defined Data Defaults **********************************************************************

#define USER_DATA_A0 0
#define USER_DATA_A1 0
#define USER_DATA_A2 0
#define USER_DATA_A3 0
#define USER_DATA_B0 0
#define USER_DATA_B1 0
#define USER_DATA_B2 0
#define USER_DATA_B3 0
#define USER_DATA_C0 0
#define USER_DATA_C1 0
#define USER_DATA_C2 0
#define USER_DATA_C3 0
#define USER_DATA_D0 0
#define USER_DATA_D1 0
#define USER_DATA_D2 0
#define USER_DATA_D3 0
