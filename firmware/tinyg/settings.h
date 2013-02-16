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

#ifndef settings_h
#define settings_h

/**** GENERAL SETTINGS ******************************************************/
// These can be overridden in machine profiles by using #undef
// ADVICE: Check your machine profile before continuing

#define STATUS_REPORT_MIN_MS		50		// milliseconds - enforces a viable minimum
#define STATUS_REPORT_INTERVAL_MS	50		// milliseconds - set to 0 to disable
#define SR_DEFAULTS "line","vel","posx","posy","posz","posa","mpox","mpoy","mpoz","mpoa","ofsx","ofsy","ofsz","ofsa","unit","momo","coor","stat","homx","homy","homz","homa"

#define SWITCH_TYPE SW_TYPE_NORMALLY_OPEN
//#define SWITCH_TYPE SW_TYPE_NORMALLY_CLOSED

//#define SR_VERBOSITY				SR_OFF
#define SR_VERBOSITY				SR_FILTERED
//#define SR_VERBOSITY				SR_VERBOSE

#define CHORDAL_TOLERANCE 0.01		// chord accuracy for arc drawing

// Gcode defaults
#define GCODE_DEFAULT_PLANE			CANON_PLANE_XY
#define GCODE_DEFAULT_UNITS			MILLIMETERS
#define GCODE_DEFAULT_COORD_SYSTEM	G54
#define GCODE_DEFAULT_PATH_CONTROL 	PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_MODE

// Comm mode and echo levels - CAUTION: may be overridden by machine profiles
#define COM_IGNORE_CRLF				IGNORE_OFF			// 0=accept either CR or LF, 1=ignore CR, 2=ignoreLF
#define COM_EXPAND_CR				false
#define COM_ENABLE_ECHO				false
#define COM_ENABLE_XON				true

#define COMM_MODE					JSON_MODE
#define TEXT_VERBOSITY				TV_VERBOSE
//#define JSON_VERBOSITY			JV_SILENT			// no response is provided for any command
//#define JSON_VERBOSITY			JV_OMIT_BODY		// response contains no body - footer only
//#define JSON_VERBOSITY			JV_OMIT_GCODE_BODY	// body returned for configs; omitted for Gcode commands
//#define JSON_VERBOSITY			JV_GCODE_LINENUM_ONLY// body returned for configs; Gcode returns line number as 'n', otherwise body is omitted
//#define JSON_VERBOSITY			JV_GCODE_MESSAGES	// body returned for configs; Gcode returns line numbers and messages only
#define JSON_VERBOSITY			JV_VERBOSE			// body returned for configs and Gcode - Gcode comments removed

// Queue report settings
#define QR_VERBOSITY				QR_OFF
//#define QR_VERBOSITY				QR_FILTERED
//#define QR_VERBOSITY				QR_VERBOSE
#define QR_HI_WATER					20
#define QR_LO_WATER					2

/**** MACHINE PROFILES ******************************************************/

// default machine profiles - chose only one:

//#include "settings/settings_default.h"			// Default settings for shipment
//#include "settings/settings_lumenlabMicRoV3.h"	// Lumenlabs micRo v3
//#include "settings/settings_otherlab.h"			// Otherlab Othercutter
//#include "settings/settings_probotixV90.h"		// Probotix FireballV90
//#include "settings/settings_sacidu93.h"			// related to Issue #12
#include "settings/settings_shapeoko375.h"		// Shapeoko 375mm kit
//#include "settings/settings_ultimaker.h"			// Ultimaker 3D printer
//#include "settings/settings_zen7x12.h"			// Zen Toolworks 7x12

/*** Handle optional modules that may not be in every machine ***/

// If PWM_1 is not defined fill it with dummy values
#ifndef	P1_PWM_FREQUENCY
#define P1_PWM_FREQUENCY	0	// Hz
#define P1_CW_SPEED_LO		0	// in RPM (arbitrary units)
#define P1_CW_SPEED_HI		0
#define P1_CW_PHASE_LO		0	// phase [0..1]
#define P1_CW_PHASE_HI		0
#define P1_CCW_SPEED_LO		0
#define P1_CCW_SPEED_HI		0
#define P1_CCW_PHASE_LO		0
#define P1_CCW_PHASE_HI		0
#define P1_PWM_PHASE_OFF	0
#endif//P1_PWM_FREQUENCY

#endif
