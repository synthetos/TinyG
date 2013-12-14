/*
 * settings.h - default runtime settings
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
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
/*	The values in this file are the default settings that are loaded into a virgin EEPROM, 
 *	and can be changed using the config commands. After initial load the EEPROM values 
 *	(or changed values) are used.
 *
 *	System and hardware settings that you shouldn't need to change are in hardware.h  
 *	Application settings that also shouldn't need to be changed are in tinyg.h
 */

#ifndef SETTINGS_H_ONCE
#define SETTINGS_H_ONCE

/**** GENERAL SETTINGS ******************************************************/

// **** PLEASE NOTE **** Any of these may be overridden in machine profiles
// Do not assume these are the effective settings. Check the machine profile 

// Machine configuration settings
#define CHORDAL_TOLERANCE 			0.001			// chord accuracy for arc drawing
#define SWITCH_TYPE 				SW_TYPE_NORMALLY_OPEN// one of: SW_TYPE_NORMALLY_OPEN, SW_TYPE_NORMALLY_CLOSED
#define MOTOR_IDLE_TIMEOUT			2.00			// seconds to maintain motor at full power before idling
#define MOTOR_POWER_LEVEL			25				// default motor power level (ARM only)

// Communications and reporting settings
#define COMM_MODE					JSON_MODE		// one of: TEXT_MODE, JSON_MODE
#define NETWORK_MODE				NETWORK_STANDALONE

//#define TEXT_VERBOSITY				TV_SILENT		// one of: TV_SILENT, TV_VERBOSE
#define TEXT_VERBOSITY				TV_VERBOSE		// one of: TV_SILENT, TV_VERBOSE

#define JSON_VERBOSITY				JV_SILENT		// one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
//#define JSON_VERBOSITY				JV_MESSAGES		// one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
//#define JSON_VERBOSITY				JV_VERBOSE		// one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE

#define JSON_FOOTER_DEPTH			0				// 0 = new style, 1 = old style

//#define SR_VERBOSITY				SR_OFF		// one of: SR_OFF, SR_FILTERED, SR_VERBOSE
#define SR_VERBOSITY				SR_FILTERED		// one of: SR_OFF, SR_FILTERED, SR_VERBOSE

#define STATUS_REPORT_MIN_MS		100				// milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS	250				// milliseconds - set $SV=0 to disable

// Must be formatted correctly.or your board won't start.
//#define SR_DEFAULTS "line","posx","posy","posz","posa","feed","vel","unit","coor","dist","frmo","momo","stat"
//#define SR_DEFAULTS "mst1","msp1","msn1","mse1","mst2","msp2","msn2","mse2","mst3","msp3","msn3","mse3","stat"
//#define SR_DEFAULTS "posx","mst1","mse1","posy","mst2","mse2","posz","mst3","mse3","stat"
#define SR_DEFAULTS "posx","mse1","posy","mse2","posz","mse3","stat"
//#define SR_DEFAULTS "line","posx","posy","posz","posa","feed","vel","unit","coor","dist","frmo","stat","momo"
//#define SR_DEFAULTS "line","qr","qi","qo","posx","posy","posz","posa","feed","vel","unit","coor","dist","frmo","momo","stat"

#define QR_VERBOSITY				QR_OFF			// one of: QR_OFF, QR_SINGLE, QR_TRIPLE
//#define QR_VERBOSITY				QR_TRIPLE

// Gcode startup defaults
#define GCODE_DEFAULT_UNITS			MILLIMETERS		// MILLIMETERS or INCHES
#define GCODE_DEFAULT_PLANE			CANON_PLANE_XY	// CANON_PLANE_XY, CANON_PLANE_XZ, or CANON_PLANE_YZ
#define GCODE_DEFAULT_COORD_SYSTEM	G54				// G54, G55, G56, G57, G58 or G59
#define GCODE_DEFAULT_PATH_CONTROL 	PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_MODE

// Comm mode and echo levels
#define COM_IGNORE_CRLF				IGNORE_OFF		// 0=accept either CR or LF, 1=ignore CR, 2=ignoreLF
#define COM_EXPAND_CR				false
#define COM_ENABLE_ECHO				false
#define COM_ENABLE_FLOW_CONTROL		FLOW_CONTROL_XON // FLOW_CONTROL_OFF, FLOW_CONTROL_XON, FLOW_CONTROL_RTS

/**** MACHINE PROFILES ******************************************************/

// machine default profiles - choose only one:

//#include "settings/settings_default.h"				// Default settings for release
//#include "settings/settings_hammer.h"					// Hammer torque demo
//#include "settings/settings_pendulum.h"				// Pendulum motion demo
//#include "settings/settings_lumenlabMicRoV3.h"		// Lumenlabs micRo v3
//#include "settings/settings_openpnp.h"				// OpenPnP
//#include "settings/settings_othercutter.h"			// OMC OtherCutter
//#include "settings/settings_othermill.h"				// OMC OtherMill
#include "settings/settings_probotixV90.h"			// Probotix FireballV90
//#include "settings/settings_shapeoko375.h"			// Shapeoko 375mm kit
//#include "settings/settings_ultimaker.h"				// Ultimaker 3D printer
//#include "settings/settings_zen7x12.h"				// Zen Toolworks 7x12

/*** Handle optional modules that may not be in every machine ***/

// If PWM_1 is not defined fill it with default values
#ifndef	P1_PWM_FREQUENCY

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
#endif//P1_PWM_FREQUENCY


/*** User-Defined Data Defaults ***/

#define USER_DATA_A0	0
#define USER_DATA_A1	0
#define USER_DATA_A2	0
#define USER_DATA_A3	0
#define USER_DATA_B0	0
#define USER_DATA_B1	0
#define USER_DATA_B2	0
#define USER_DATA_B3	0
#define USER_DATA_C0	0
#define USER_DATA_C1	0
#define USER_DATA_C2	0
#define USER_DATA_C3	0
#define USER_DATA_D0	0
#define USER_DATA_D1	0
#define USER_DATA_D2	0
#define USER_DATA_D3	0

#endif // End of include guard: SETTINGS_H_ONCE
