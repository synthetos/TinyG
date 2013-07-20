/*
 * settings.h - default runtime settings
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
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
/*	The values in this file are the default settings that are loaded into 
 *	a virgin EEPROM, and can be changed using the config commands.
 *	After initial load the EEPROM values (or changed values) are used.
 *
 *	System and hardware settings that you shouldn't need to change 
 *	are in system.h  Application settings that also shouldn't need 
 *	to be changed are in tinyg.h
 */

#ifndef _SETTINGS_H_
#define _SETTINGS_H_

/**** GENERAL SETTINGS ******************************************************/

// **** PLEASE NOTE **** Any of these may be overridden in machine profiles
// Do not assume these are the effective settings. Check the machine profile 

// Machine configuration settings
#define CHORDAL_TOLERANCE 			0.001			// chord accuracy for arc drawing
#define SWITCH_TYPE 				SW_TYPE_NORMALLY_OPEN	// one of: SW_TYPE_NORMALLY_OPEN, SW_TYPE_NORMALLY_CLOSED
#define MOTOR_DISABLE_TIMEOUT		60				// seconds

// Communications and reporting settings
#define COMM_MODE					TEXT_MODE		// one of: TEXT_MODE, JSON_MODE
#define NETWORK_MODE				NETWORK_STANDALONE
#define TEXT_VERBOSITY				TV_VERBOSE		// one of: TV_SILENT, TV_VERBOSE
#define JSON_VERBOSITY				JV_MESSAGES		// one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
//#define JSON_VERBOSITY				JV_LINENUM		// one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE
//#define JSON_VERBOSITY				JV_VERBOSE		// one of: JV_SILENT, JV_FOOTER, JV_CONFIGS, JV_MESSAGES, JV_LINENUM, JV_VERBOSE

#define SR_VERBOSITY				SR_FILTERED		// one of: SR_OFF, SR_FILTERED, SR_VERBOSE
#define STATUS_REPORT_MIN_MS		50				// milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS	250				// milliseconds - set $SV=0 to disable
#define SR_DEFAULTS "line","posx","posy","posz","posa","feed","vel","unit","coor","dist","frmo","momo","stat"

#define QR_VERBOSITY				QR_OFF			// one of: QR_OFF, QR_FILTERED, QR_VERBOSE
#define QR_HI_WATER					20
#define QR_LO_WATER					2

// Gcode startup defaults
#define GCODE_DEFAULT_PLANE			CANON_PLANE_XY
#define GCODE_DEFAULT_UNITS			MILLIMETERS
#define GCODE_DEFAULT_COORD_SYSTEM	G54
#define GCODE_DEFAULT_PATH_CONTROL 	PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_MODE

// Comm mode and echo levels
#define COM_IGNORE_CRLF				IGNORE_OFF	// 0=accept either CR or LF, 1=ignore CR, 2=ignoreLF
#define COM_EXPAND_CR				false
#define COM_ENABLE_ECHO				false
//#define COM_ENABLE_FLOW_CONTROL		FLOW_CONTROL_XON
#define COM_ENABLE_FLOW_CONTROL		FLOW_CONTROL_RTS

/**** MACHINE PROFILES ******************************************************/

// machine default profiles - chose only one:

#include "settings/settings_default.h"					// Default settings for release
//#include "settings/settings_lumenlabMicRoV3.h"		// Lumenlabs micRo v3
//#include "settings/settings_mcgyver.h"				// debugging
//#include "settings/settings_openpnp.h"				// OpenPnP
//#include "settings/settings_othercutter.h"			// Otherfab OtherCutter
//#include "settings/settings_othermill.h"				// Otherfab OtherMill
//#include "settings/settings_pocketcnc_linear.h"		// PocketCNC 5 axis mill - linear settings
//#include "settings/settings_pocketcnc_rotary.h"		// PocketCNC 5 axis mill - rotary settings
//#include "settings/settings_probotixV90.h"			// Probotix FireballV90
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

#endif // _SETTINGS_H_
