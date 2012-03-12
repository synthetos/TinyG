/*
 * test.c - tinyg test sets
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for details. You should have received a copy of the GNU General Public 
 * License along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* See the wiki for additional information about tests:
 *	 http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Developer-Info
 */

#include <stdio.h>				// precursor for xio.h
#include <avr/pgmspace.h>		// precursor for xio.h
#include "tinyg.h"				// #1 unfortunately, there are some dependencies
#include "util.h"				// #2
#include "config.h"				// #3
#include "controller.h"
#include "test.h"
#include "xio/xio.h"

// regression test files
#include "tests/test_001_smoke.h" 	// basic functionality
#include "tests/test_002_init.h"	// test initialization, not initialize tests
#include "tests/test_003_homing.h"
#include "tests/test_004_motion.h"	// square and circle moves
#include "tests/test_005_dwell.h"
#include "tests/test_006_feedhold.h"
#include "tests/test_007_Mcodes.h"
#include "tests/test_008_json.h"
#include "tests/test_009_inverse_time.h"

/*
 * tg_test() - system tests from FLASH invoked by $test=n command
 *
 * 	By convention the character array containing the test must have the same 
 *	name as the file name.
 */
uint8_t tg_test(cmdObj *cmd)
{
	switch ((uint8_t)cmd->value) {
		case 0: { return (TG_OK);}
		case 1: { xio_open_pgm(PGMFILE(&test_smoke)); break;}
		case 2: { xio_open_pgm(PGMFILE(&test_init)); break;}
		case 3: { xio_open_pgm(PGMFILE(&test_homing)); break;}
		case 4: { xio_open_pgm(PGMFILE(&test_motion)); break;}
		case 5: { xio_open_pgm(PGMFILE(&test_dwell)); break;}
		case 6: { xio_open_pgm(PGMFILE(&test_feedhold)); break;}
		case 7: { xio_open_pgm(PGMFILE(&test_Mcodes)); break;}
		case 8: { xio_open_pgm(PGMFILE(&test_json)); break;}
		case 9: { xio_open_pgm(PGMFILE(&test_inverse_time)); break;}
		default: {
			fprintf_P(stderr,PSTR("Test #%d not found\n"),(uint8_t)cmd->value);
			return (TG_ERROR);
		}
	}
	tg_set_active_source(XIO_DEV_PGM);
	return (TG_OK);
}

/*
 * _canned_startup() - run a string on startup
 *
 *	Pre-load the USB RX (input) buffer with some test strings that will be called 
 *	on startup. Be mindful of the char limit on the read buffer (RX_BUFFER_SIZE).
 *	It's best to create a test file for really complicated things.
 */
void tg_canned_startup()	// uncomment in tinyg.h if you want to run this
{
#ifdef __CANNED_STARTUP

	xio_queue_RX_string_usb("N6 G92X0.327Y-33.521Z-1.000\n");
	xio_queue_RX_string_usb("N7 G0Z4.000\n");
	xio_queue_RX_string_usb("N8 F400.0\n");
	xio_queue_RX_string_usb("N9 G1X0.327Y-33.521\n");
	xio_queue_RX_string_usb("N10 G1Z-1.000\n");

/* Run test file */
//	xio_queue_RX_string_usb("$test=1\n");		// run test file
//	xio_queue_RX_string_usb("$test=2\n");
//	xio_queue_RX_string_usb("$test=3\n");
//	xio_queue_RX_string_usb("$test=4\n");
//	xio_queue_RX_string_usb("$test=5\n");
//	xio_queue_RX_string_usb("$test=6\n");
//	xio_queue_RX_string_usb("$test=7\n");
//	xio_queue_RX_string_usb("$test=8\n");
//	xio_queue_RX_string_usb("$t=9\n");

/* Other command sequences */
//	xio_queue_RX_string_usb("H\n");				// show help file
//	xio_queue_RX_string_usb("\n\n");			// 2 null lines
//	xio_queue_RX_string_usb("%\n");				// opening percent character
//	xio_queue_RX_string_usb("$\n");				// display general group
//	xio_queue_RX_string_usb("?\n");				// report
//	Test signals - Note: requires test chars to be enabled
//	xio_queue_RX_string_usb("^\n");				// abort 
//	xio_queue_RX_string_usb("!\n");				// feedhold
//	xio_queue_RX_string_usb("~\n");				// cycle start

/* Configs and controls */
//	xio_queue_RX_string_usb("$\n");				// print general group
//	xio_queue_RX_string_usb("$x\n");			// print x axis
//	xio_queue_RX_string_usb("$1\n");			// print motor #1 group
//	xio_queue_RX_string_usb("$m\n");			// print all motor groups 
//	xio_queue_RX_string_usb("$n\n");			// print all axis groups
//	xio_queue_RX_string_usb("$o\n");			// print offset groups
//	xio_queue_RX_string_usb("$$\n");			// print everything
//	xio_queue_RX_string_usb("$xam\n");			// print x axis mode
//	xio_queue_RX_string_usb("$sys\n");			// print system settings
//	xio_queue_RX_string_usb("$unit\n");
//	xio_queue_RX_string_usb("$sr\n");

//	xio_queue_RX_string_usb("$xfr=1000\n");
//	xio_queue_RX_string_usb("$2mi=4\n");
//	xio_queue_RX_string_usb("$xjm 1000000\n");
//	xio_queue_RX_string_usb("$xvm\n");			// config with no data
//	xio_queue_RX_string_usb("$ja\n");			// config with no data
//	xio_queue_RX_string_usb("$aam = 3\n");		// set A to radius mode
//	xio_queue_RX_string_usb("$aam 10\n");		// set A to SLAVE_XYZ mode
//	xio_queue_RX_string_usb("(MSGtest message in comment)\n");

/* M code tests */
//	xio_queue_RX_string_usb("m4\n");			// spindle CCW
//	xio_queue_RX_string_usb("m5\n");			// spindle OFF

/* G0's */
//	xio_queue_RX_string_usb("g0 x0.2\n");		// shortest drawable line
//	xio_queue_RX_string_usb("g0 x0\n");
//	xio_queue_RX_string_usb("g0 x2\n");
//	xio_queue_RX_string_usb("g0 x3\n");
//	xio_queue_RX_string_usb("g0 y3\n");
//	xio_queue_RX_string_usb("g0 x3 y4 z5.5\n");
//	xio_queue_RX_string_usb("g0 x10 y10 z10 a10\n");
//	xio_queue_RX_string_usb("g0 x2000 y3000 z4000 a5000\n");

/* G1's */
//	xio_queue_RX_string_usb("g1 f300 x100\n");
//	xio_queue_RX_string_usb("g1 f10 x100\n");
//	xio_queue_RX_string_usb("g1 f450 x10 y13\n");
//	xio_queue_RX_string_usb("g1 f450 x10 y13\n");
//	xio_queue_RX_string_usb("g1 f0 x10\n");

/* G2/G3's */
//	xio_queue_RX_string_usb("g3 f500 x100 y100 z25 i50 j50\n");	// arcs
//	xio_queue_RX_string_usb("g2 f2000 x50 y50 z2 i25 j25\n");	// arcs
//	xio_queue_RX_string_usb("g2 f300 x10 y10 i8 j8\n");
//	xio_queue_RX_string_usb("g2 f300 x10 y10 i5 j5\n");
//	xio_queue_RX_string_usb("g2 f300 x3 y3 i1.5 j1.5\n");

/* G4 tests (dwells) */
//	xio_queue_RX_string_usb("g0 x20 y23 z10\n");
//	xio_queue_RX_string_usb("g4 p0.1\n");
//	xio_queue_RX_string_usb("g0 x10 y10 z-10\n");

/* G53 tests */
//	xio_queue_RX_string_usb("g56\n");			// assumes G55 is different than machine coords
//	xio_queue_RX_string_usb("g0 x0 y0\n");		// move to zero in G55
//	xio_queue_RX_string_usb("g53 g0 x0 y0\n");	// should move off G55 zero back to machine zero

/* G54-G59 tests */
//	xio_queue_RX_string_usb("g54\n");
//	xio_queue_RX_string_usb("g55\n");
//	xio_queue_RX_string_usb("g10 p2 l2 x10 y10 z-10\n");

/* G92 tests */
//	xio_queue_RX_string_usb("g92 x20 y20\n");	// apply offsets
//	xio_queue_RX_string_usb("g0 x0 y0\n");		// should move diagonally to SouthWest
//	xio_queue_RX_string_usb("g92.1\n");			// cancel offsets
//	xio_queue_RX_string_usb("g0 x0 y0\n");		// should move NW back to original coordinates
//	xio_queue_RX_string_usb("g92.2\n");
//	xio_queue_RX_string_usb("g92.3\n");

/* G28 and G30 homing tests */
//	xio_queue_RX_string_usb("g28x0y0z0\n");
//	xio_queue_RX_string_usb("g30x0y0z0\n");
//	xio_queue_RX_string_usb("g30x42\n");

/* Other Gcode tests */
//	xio_queue_RX_string_usb("g20\n");			// inch mode
//	xio_queue_RX_string_usb("g21\n");			// mm mode
//	xio_queue_RX_string_usb("g18\n");			// plane select
//	xio_queue_RX_string_usb("g10 l2 p4 x20 y20 z-10\n"); // test G10

/* Feedhold tests */
// Consider dropping PLANNER_BUFFER_POOL_SIZE down to something like 4 for these tests
//	xio_queue_RX_string_usb("g0 x3 y4 z5.5\n");
//	xio_queue_RX_string_usb("g0 x1 y1 z1\n");
//	xio_queue_RX_string_usb("g0 x0.1\n");
//	xio_queue_RX_string_usb("g0 x0.2\n");
//	xio_queue_RX_string_usb("g0 x0.3\n");
//	xio_queue_RX_string_usb("g0 x0.4\n");
//	xio_queue_RX_string_usb("g0 x0.5\n");
//	xio_queue_RX_string_usb("@\n");		// issue feedhold - uncomment __ENABLE_DEBUG_CHARS
//	xio_queue_RX_string_usb("#\n");		// end feedhold - uncomment __ENABLE_DEBUG_CHARS
//	xio_queue_RX_string_usb("!");				// issue feedhold
//	xio_queue_RX_string_usb("~");				// end feedhold
//	xio_queue_RX_string_usb("g0 x0 y0 z0\n");
//	xio_queue_RX_string_usb("g0 x50\n");
//	xio_queue_RX_string_usb("g0 y5\n");
//	See 331.19 or earlier for some more lengthy feedhold tests

/* JSON tests */
// If you want to use all these you need to set RX buffer to 1024 in xio_usart.h
/*	xio_queue_RX_string_usb("{\"x_feedrate\":1200}\n");
	xio_queue_RX_string_usb("{\"xfr\":1200, \"yfr\":1201, \"zfr\":600}\n");
	xio_queue_RX_string_usb("{\"err_1\":36000}\n");
	xio_queue_RX_string_usb("{\"1sa\":3.6.000}\n");
	xio_queue_RX_string_usb("{\"gcode\":\"g0 x3 y4 z5.5 (comment line)\"}\n");
	xio_queue_RX_string_usb("{\"config_version\":null}\n");	// simple null test
	xio_queue_RX_string_usb("{\"config_profile\":true}\n");	// simple true test
	xio_queue_RX_string_usb("{\"prompt\":false}\n");		// simple false test
	xio_queue_RX_string_usb("{\"gcode\":\"g0 x3 y4 z5.5 (comment line)\"}\n");// string test w/comment
	xio_queue_RX_string_usb("{\"x_feedrate\":1200}\n");		// numeric test
	xio_queue_RX_string_usb("{\"y_feedrate\":-1456}\n");	// numeric test
	xio_queue_RX_string_usb("{\"Z_velocity_maximum\":null}\n");// axis w/null
	xio_queue_RX_string_usb("{\"m1_microsteps\":null}\n");	// motor w/null
	xio_queue_RX_string_usb("{\"2mi\":8}\n");				// motor token w/null
	xio_queue_RX_string_usb("{\"no-token\":12345}\n");		// non-token w/number
	xio_queue_RX_string_usb("{\"firmware_version\":329.26,		\"config_version\":0.93}\n");
	xio_queue_RX_string_usb("{\"1mi\":8, \"2mi\":8,\"3mi\":8,\"4mi\":8}\n");	// 4 elements
	xio_queue_RX_string_usb("{\"status_report\":{\"ln\":true, \"x_pos\":true, \"y_pos\":true, \"z_pos\":true}}\n");
	xio_queue_RX_string_usb("{\"parent_case1\":{\"child_null\":null}}\n");	// parent w/single child
	xio_queue_RX_string_usb("{\"parent_case2\":{\"child_num\":23456}}\n");	// parent w/single child
	xio_queue_RX_string_usb("{\"parent_case3\":{\"child_str\":\"stringdata\"}}\n");// parent w/single child
	xio_queue_RX_string_usb("{\"err_1\":36000x\n}");		// illegal number 
	xio_queue_RX_string_usb("{\"err_2\":\"text\n}");		// no string termination
	xio_queue_RX_string_usb("{\"err_3\":\"12345\",}\n");	// bad } termination
	xio_queue_RX_string_usb("{\"err_4\":\"12345\"\n");		// no } termination
*/
//	xio_queue_RX_string_usb("{\"x\":\"\"}\n");				// x axis group display
//	xio_queue_RX_string_usb("{\"1\":\"\"}\n");				// motor 1 group display
//	xio_queue_RX_string_usb("{\"sys\":\"\"}\n");			// system group display
//	xio_queue_RX_string_usb("{\"x\":null}\n");				// group display
//	xio_queue_RX_string_usb("{\"x\":{\"am\":1,\"fr\":800.000,\"vm\":800.000,\"tm\":100.000,\"jm\":100000000.000,\"jd\":0.050,\"sm\":1,\"sv\":800.000,\"lv\":100.000,\"zo\":3.000,\"abs\":0.000,\"pos\":0.000}}\n");
//	xio_queue_RX_string_usb("{\"sys\":{\"fv\":0.930,\"fb\":330.390,\"si\":250,\"gpl\":0,\"gun\":1,\"gco\":1,\"gpa\":2,\"gdi\":0,\"ea\":1,\"ja\":200000.000,\"ml\":0.080,\"ma\":0.100,\"mt\":10000.000,\"ic\":0,\"il\":0,\"ec\":0,\"ee\":1,\"ex\":1}}\n");

//	xio_queue_RX_string_usb("{\"  xfr  \":null}\n");		// JSON string normalization tests
//	xio_queue_RX_string_usb("{\"gcode\":\"G1 x100 (Title Case Comment)   \"}\n");
//	xio_queue_RX_string_usb("{\"sr\":{\"ln\":true,\"vl\":true,\"ms\":true}}\n");  // set status report
//	xio_queue_RX_string_usb("{\"sr\":{\"line\":true,\"posx\":true,\"stat\":true}}\n"); // set status report
//	xio_queue_RX_string_usb("{\"sr\":\"\"}\n");				// get status report
//	xio_queue_RX_string_usb("g0 x10\n");
//	xio_queue_RX_string_usb("{\"gc\":\"g0 x2\"}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"g1 f243.543 x22.3456 y32.2134 z-0.127645\"}\n");
/*
	xio_queue_RX_string_usb("{\"gc\":\"N1 T1M6\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N2 G17\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N3 G21 (mm)\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N4 (S8000)\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N5 (M3)\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N6 G92X0.327Y-33.521Z-1.000\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N7 G0Z4.000\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N8 F300.0\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N9 G1X0.327Y-33.521\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N10 G1Z-1.000\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N11 X0.654Y-33.526\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N12 X0.980Y-33.534\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N13 X1.304Y-33.546\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N14 X1.626Y-33.562\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N15 X1.946Y-33.580\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N16 X2.262Y-33.602\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N17 X2.574Y-33.628\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N18 X2.882Y-33.656\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N19 X3.185Y-33.688\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N20 X3.483Y-33.724\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N21 X3.775Y-33.762\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N22 X4.060Y-33.805\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N23 X4.339Y-33.850\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N24 X4.610Y-33.898\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N25 X4.874Y-33.950\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N26 X5.130Y-34.005\"}\n");
	xio_queue_RX_string_usb("{\"gc\":\"N27 X5.376Y-34.064\"}\n");
*/
#endif
}
