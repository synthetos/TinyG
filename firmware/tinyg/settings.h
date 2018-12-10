/*
 * settings.h - default runtime settings
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2016 Alden S. Hart Jr.
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
/*	The values referenced by this file are the default settings that are loaded into a
 *  virgin EEPROM, and can be changed using the config commands. After initial load
 *  the EEPROM values (or changed values) are used.
 *
 *	System and hardware settings that you shouldn't need to change are in hardware.h
 *	Application settings that also shouldn't need to be changed are in tinyg.h
 */

#ifndef SETTINGS_H_ONCE
#define SETTINGS_H_ONCE

/**** MACHINE PROFILES ******************************************************
 *
 * Provide a SETTINGS_FILE in the makefile or compiler command line, e.g:
 *	SETTINGS_FILE="settings_shopbot_test.h"
 *
 * If no file is specified the indicated default settings file will be used
 */

#ifdef SETTINGS_FILE
#define SETTINGS_FILE_PATH <settings/SETTINGS_FILE>
#include SETTINGS_FILE_PATH
#else // SELECT DEFAULT SETTINGS FILE - SELECT ONE OF:

#include "settings/settings_default.h"        // Default settings for release
//#include "settings/settings_cnc3040.h"        // Generic CNC3040 machine
//#include "settings/settings_othermill.h"      // OMC OtherMill
//#include "settings/settings_probotixV90.h"    // Probotix Fireball V90
//#include "settings/settings_shapeoko2.h"      // Shapeoko2 - standard kit
//#include "settings/settings_ultimaker.h"      // Ultimaker 3D printer
//#include "settings/settings_zen7x12.h"        // Zen Toolworks 7x12

//#include "settings/settings_test_001.h"       // test settings
#endif

#endif // End of include guard: SETTINGS_H_ONCE
