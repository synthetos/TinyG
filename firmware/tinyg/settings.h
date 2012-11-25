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

#define STATUS_REPORT_MIN_MS		200		// ms - enforces a viable minimum

//#define STATUS_REPORT_INTERVAL_MS	200	// in milliseconds
#define STATUS_REPORT_INTERVAL_MS	0	// in milliseconds

#define GCODE_DEFAULT_PLANE			CANON_PLANE_XY
#define GCODE_DEFAULT_UNITS			MILLIMETERS
#define GCODE_DEFAULT_COORD_SYSTEM	G54
#define GCODE_DEFAULT_PATH_CONTROL 	PATH_CONTINUOUS
#define GCODE_DEFAULT_DISTANCE_MODE ABSOLUTE_MODE

#define COM_APPEND_TX_CR			false
#define COM_IGNORE_CRLF				IGNORE_OFF		// 0=accept either CR or LF, 1=ignore CR, 2=ignoreLF
#define COM_ENABLE_ECHO				false
#define COM_ENABLE_XON				true

//#define COM_ENABLE_QR				true
#define COM_ENABLE_QR				false

//#define COM_COMMUNICATIONS_MODE	TG_TEXT_MODE	// alternately: TG_TEXT_MODE
#define COM_COMMUNICATIONS_MODE		TG_JSON_MODE	// alternately: TG_TEXT_MODE
//#define COM_ENABLE_JSON_ECHO		false
#define COM_ENABLE_JSON_ECHO		true

//#define ENABLE_ACCELERATION 1			// *** NOTE: this feature is disabled in 338.11 - always equal to 1 


/**** MACHINE PROFILES ******************************************************/

// default machine profiles - chose only one:

//#include "settings/settings_default.h"			// Default settings for shipment
//#include "settings/settings_zen7x12.h"			// Zen Toolworks 7x12
#include "settings/settings_shapeoko375.h"		// Shapeoko 375mm kit
//#include "settings/settings_Ultimaker.h"			// Ultimaker 3D printer
//#include "settings/settings_probotixV90.h"		// Probotix FireballV90
//#include "settings/settings_lumenlabMicRoV3.h"	// Lumenlabs micRo v3
//#include "settings/settings_sacidu93.h"			// related to Issue #12
//#include "settings/settings_otherlab.h"			// Othwrlab Cardboard cutter

#endif
