/*
 * test.c - tinyg test sets
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
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
/* See the wiki for additional information about tests:
 *	 http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Developer-Info
 */

#include <stdio.h>				// precursor for xio.h
#include <avr/pgmspace.h>		// precursor for xio.h
#include "tinyg.h"				// #1 unfortunately, there are some dependencies
#include "util.h"				// #2
#include "config.h"				// #3
#include "controller.h"
#include "planner.h"
#include "test.h"
#include "xio/xio.h"

// regression test files
#include "tests/test_001_homing.h"			// G28.1 homing cycles
#include "tests/test_002_smoke.h" 			// basic functionality
#include "tests/test_003_squares.h"			// square moves
#include "tests/test_004_arcs.h"			// arc moves
#include "tests/test_005_dwell.h"			// dwells embedded in move sequences
#include "tests/test_006_feedhold.h"		// feedhold - requires manual ! and ~ entry
#include "tests/test_007_Mcodes.h"			// M codes synchronized w/moves (planner queue)
#include "tests/test_008_json.h"			// JSON parser and IO
#include "tests/test_009_inverse_time.h"	// inverse time mode
#include "tests/test_010_rotary.h"			// ABC axes
#include "tests/test_011_small_moves.h"		// small move test
#include "tests/test_012_slow_moves.h"		// slow move test
#include "tests/test_013_coordinate_offsets.h"	// what it says
#include "tests/test_050_mudflap.h"			// mudflap test - entire drawing
#include "tests/test_051_braid.h"			// braid test - partial drawing

/*
 * tg_test() - system tests from FLASH invoked by $test=n command
 *
 * 	By convention the character array containing the test must have the same 
 *	name as the file name.
 */
uint8_t tg_test(cmdObj_t *cmd)
{
	switch ((uint8_t)cmd->value) {
		case 0: { return (TG_OK);}
		case 1: { xio_open(XIO_DEV_PGM, PGMFILE(&test_homing),PGM_FLAGS); break;}
		case 2: { xio_open(XIO_DEV_PGM, PGMFILE(&test_smoke),PGM_FLAGS); break;}
		case 3: { xio_open(XIO_DEV_PGM, PGMFILE(&test_squares),PGM_FLAGS); break;}
		case 4: { xio_open(XIO_DEV_PGM, PGMFILE(&test_arcs),PGM_FLAGS); break;}
		case 5: { xio_open(XIO_DEV_PGM, PGMFILE(&test_dwell),PGM_FLAGS); break;}
		case 6: { xio_open(XIO_DEV_PGM, PGMFILE(&test_feedhold),PGM_FLAGS); break;}
		case 7: { xio_open(XIO_DEV_PGM, PGMFILE(&test_Mcodes),PGM_FLAGS); break;}
		case 8: { xio_open(XIO_DEV_PGM, PGMFILE(&test_json),PGM_FLAGS); break;}
		case 9: { xio_open(XIO_DEV_PGM, PGMFILE(&test_inverse_time),PGM_FLAGS); break;}
		case 10: { xio_open(XIO_DEV_PGM, PGMFILE(&test_rotary),PGM_FLAGS); break;}
		case 11: { xio_open(XIO_DEV_PGM, PGMFILE(&test_small_moves),PGM_FLAGS); break;}
		case 12: { xio_open(XIO_DEV_PGM, PGMFILE(&test_slow_moves),PGM_FLAGS); break;}
		case 13: { xio_open(XIO_DEV_PGM, PGMFILE(&test_coordinate_offsets),PGM_FLAGS); break;}
		case 50: { xio_open(XIO_DEV_PGM, PGMFILE(&test_mudflap),PGM_FLAGS); break;}
		case 51: { xio_open(XIO_DEV_PGM, PGMFILE(&test_braid),PGM_FLAGS); break;}
		default: {
			fprintf_P(stderr,PSTR("Test #%d not found\n"),(uint8_t)cmd->value);
			return (TG_ERROR);
		}
	}
	tg_set_active_source(XIO_DEV_PGM);
	return (TG_OK);
}

/*
 * tg_canned_startup() - run a string on startup
 *
 *	Pre-load the USB RX (input) buffer with some test strings that will be called 
 *	on startup. Be mindful of the char limit on the read buffer (RX_BUFFER_SIZE).
 *	It's best to create a test file for really complicated things.
 */
void tg_canned_startup()	// uncomment in tinyg.h if you want to run this
{
#ifdef __CANNED_STARTUP

// avrdude -p x192a3 -c avr109 -b 115200 -P COM19
// avrdude -e -p atxmega192a3 -c avrispmkii -P usb -U boot:w:xboot-boot.hex

//	xio_queue_RX_string_usb("{\"sr\":{\"vel\":true,\"mpox\":true,\"mpoy\":true}}\n");

	xio_queue_RX_string_usb("$test=4\n");

//	xio_queue_RX_string_usb("$qf\n");
//	xio_queue_RX_string_usb("$defau=1\n");
//	xio_queue_RX_string_usb("$id\n");
//	xio_queue_RX_string_usb("{\n");
//	xio_queue_RX_string_usb("G3 X28.949238578680202 Y33.51776649746193 I2.1091370558375635 J-2.1091370558375635 F1524\n");

//	xio_queue_RX_string_usb("g0x2\n");			// G0 smoke test
//	xio_queue_RX_string_usb("{\"gc\":\"g2\"}\n");// G0 smoke test in JSON

	// text parser test cases
//	xio_queue_RX_string_usb("?\n");				// text mode status report
//	xio_queue_RX_string_usb("$$\n");			// show all request
//	xio_queue_RX_string_usb("$ec=1\n");			// turn CR expansion on
//	xio_queue_RX_string_usb("$qr\n");			// invoke QR report
//	xio_queue_RX_string_usb("$ej=1\n");			// enable JSON mode
//	xio_queue_RX_string_usb("$n\n");			// ubergroup request
//	xio_queue_RX_string_usb("$xvm=16,000\n");	// comma skipping
//	xio_queue_RX_string_usb("$a\n");			// match a group
//	xio_queue_RX_string_usb("$xabcdefghij\n");	// overrun
//	xio_queue_RX_string_usb("$x=1\n");			// trying to set a group

//	xio_queue_RX_string_usb("$hv=6\n");				// set to text mode
//	xio_queue_RX_string_usb("$hv=7\n");				// set to text mode
//	xio_queue_RX_string_usb("N100 (MSG*** message test with line number and gcode command ***)\n");
//	xio_queue_RX_string_usb("(MSG*** message test with no line number or gcode command ***)\n");
//	xio_queue_RX_string_usb("N100\n");			// just the line number
//	xio_queue_RX_string_usb("N100 g0 x1\n");	// line number and command
//	xio_queue_RX_string_usb("N100 g0x1 (MSG*** message test with gcode command and line number ***)\n");

//	xio_queue_RX_string_usb("$test=2\n");
//	xio_queue_RX_string_usb("$$\n");
//	xio_queue_RX_string_usb("m3\n");
//	xio_queue_RX_string_usb("$sys\n");
//	xio_queue_RX_string_usb("$xvm\n");
//	xio_queue_RX_string_usb("$xasasas=42\n");// bad command
//	xio_queue_RX_string_usb("$test=51\n");	// run braid fragment to test short-line handling

//	xio_queue_RX_string_usb("{\"sr\":{\"vel\":true,\"posa\":true,\"posx\":true,\"gc\":true,\"feed\":true,\"posy\":true,\"line\":true,\"stat\":true,\"posz\":true}}\n");
//	xio_queue_RX_string_usb("{\"sr\":\"\"}\n");
//	xio_queue_RX_string_usb("{\"ej\":1}\n");
//	xio_queue_RX_string_usb("{\"z\":{\"sn\":1}}\n");
//	xio_queue_RX_string_usb("{\"zsn\":1}\n");
//	xio_queue_RX_string_usb("{\"hom\":\"\"}\n");

//	xio_queue_RX_string_usb("{\"rx\":\"\"}\n");
//	xio_queue_RX_string_usb("{\"x\":20}\n");
//	xio_queue_RX_string_usb("{\"1\":{\"po\":\"\"}}\n");
//	xio_queue_RX_string_usb("{\"c\":\"\"}\n");

//	xio_queue_RX_string_usb("{\"sr\":{\"posx\":true}}\n");
//	xio_queue_RX_string_usb("{\"baud\":6}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"n20g0x1y1.1\"}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"g0x20y30z40\"}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"g0x30\"}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"g0x40\"}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"g0x0\"}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"g20\"}\n");
//	xio_queue_RX_string_usb("{\"qr\":\"\"}\n");
//	xio_queue_RX_string_usb("{\"sys\":\"\"}\n");
//	xio_queue_RX_string_usb("{\"sys\":\"\"}\n");
//	xio_queue_RX_string_usb("{\"sr\":\"\"}\n");
//	xio_queue_RX_string_usb("{\"mpo\":\"\"}\n");

//	xio_queue_RX_string_usb("g92a0\n");
//	xio_queue_RX_string_usb("g0a3\n");			// should be a=3. Instead it's a=1

//	xio_queue_RX_string_usb("t3\n");			// change tool

//	xio_queue_RX_string_usb("$h\n");
//	xio_queue_RX_string_usb("$m\n");
//	xio_queue_RX_string_usb("$$\n");
//	xio_queue_RX_string_usb("?\n");
//	xio_queue_RX_string_usb("$sr\n");
//	xio_queue_RX_string_usb("$x\n");
//	xio_queue_RX_string_usb("\n");

//	xio_queue_RX_string_usb("g00xqwrsdfs\n");
//	xio_queue_RX_string_usb("g0x1000\n");

//	xio_queue_RX_string_usb("g00g17g21g40g49g80g90\n");
//	xio_queue_RX_string_usb("g2x0y0i1f2000\n");
//	xio_queue_RX_string_usb("f3000\n");
//	xio_queue_RX_string_usb("g80\n");

/*
	xio_queue_RX_string_usb("N0000 G91\n");
	xio_queue_RX_string_usb("N0010 M6 T1\n");
	xio_queue_RX_string_usb("N0020 G00 Z2.0000\n");
	xio_queue_RX_string_usb("N0030 M03\n");
	xio_queue_RX_string_usb("N0040 G00 X21.5000 Y13.0000\n");
	xio_queue_RX_string_usb("N0050 G00 Z0.5000\n");
	xio_queue_RX_string_usb("N0060 G01 Z-0.5000 F100 S1000\n");
	xio_queue_RX_string_usb("N0070 G02 I20.0000 J13.0000 F100.0\n");
*/

//	xio_queue_RX_string_usb("$aam=3\n");
//	xio_queue_RX_string_usb("$\n");
//	xio_queue_RX_string_usb("$4tr=720\n");
//	xio_queue_RX_string_usb("g0 a360\n");
//	xio_queue_RX_string_usb("m3\n");
//	xio_queue_RX_string_usb("g0 x10\n");

/* Run test file */
//	xio_queue_RX_string_usb("$test=12\n");		// run test file

/* Other command sequences */
//	xio_queue_RX_string_usb("H\n");				// show help file
//	xio_queue_RX_string_usb("\n\n");			// 2 null lines
//	xio_queue_RX_string_usb("%\n");				// opening percent character
//	xio_queue_RX_string_usb("$\n");				// display general group
//	xio_queue_RX_string_usb("?\n");				// report
//	Test signals - Note: requires test chars to be enabled
//	xio_queue_RX_string_usb("^\n");				// reset 
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
//	xio_queue_RX_string_usb("g3 f500 x100 y100 z25 i50 j50\n");
//	xio_queue_RX_string_usb("g2 x0 y0 i30 j30 f2000\n");
//	xio_queue_RX_string_usb("g2 f2000 x50 y50 z2 i25 j25\n");
//	xio_queue_RX_string_usb("g2 f300 x10 y10 i8 j8\n");
//	xio_queue_RX_string_usb("g2 f300 x10 y10 i5 j5\n");
//	xio_queue_RX_string_usb("g2 f300 x3 y3 i1.5 j1.5\n");
//	xio_queue_RX_string_usb("g2 f300 i10 j10\n");				// G2 pocket arc
//	xio_queue_RX_string_usb("f400\n");							// set feed rate while in arc motion mode
//	xio_queue_RX_string_usb("g3 f300 i10 j10\n");				// G3 pocket arc

/* G4 tests (dwells) */
//	xio_queue_RX_string_usb("g0 x20 y23 z10\n");
//	xio_queue_RX_string_usb("g4 p0.1\n");
//	xio_queue_RX_string_usb("g0 x10 y10 z-10\n");

/* G10 coordinate offsets */
//	xio_queue_RX_string_usb("g10 L2 p2 x10 y11 z12\n");

/* G20 / G21 units */
//	xio_queue_RX_string_usb("g20\n");
//	xio_queue_RX_string_usb("g21\n");

/* G28 and G30 homing tests */
//	xio_queue_RX_string_usb("g28.2x0\n");
//	xio_queue_RX_string_usb("g28.1\n");			// G28.1 OK
//	xio_queue_RX_string_usb("g28.1x10y10\n");	// G28.1 specification error
//	xio_queue_RX_string_usb("g28.2x0y0z0\n");
//	xio_queue_RX_string_usb("g28.2y0\n");
//	xio_queue_RX_string_usb("g28.2x0y0z0a0\n");
//	xio_queue_RX_string_usb("g28.2 z0\n");
//	xio_queue_RX_string_usb("g30x0y0z0\n");
//	xio_queue_RX_string_usb("g30x42\n");

/* G48-G51 override tests */
//	xio_queue_RX_string_usb("m50 P1\n");		// enable feed override

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

/* Other Gcode tests */
//	xio_queue_RX_string_usb("g20\n");			// inch mode
//	xio_queue_RX_string_usb("g21\n");			// mm mode
//	xio_queue_RX_string_usb("g18\n");			// plane select
//	xio_queue_RX_string_usb("g10 l2 p4 x20 y20 z-10\n"); // test G10

/* M code tests */
//	xio_queue_RX_string_usb("m3\n");			// spindle CW
//	xio_queue_RX_string_usb("m4\n");			// spindle CCW
//	xio_queue_RX_string_usb("m5\n");			// spindle off
//	xio_queue_RX_string_usb("m7\n");			// mist coolant on
//	xio_queue_RX_string_usb("m8\n");			// flood coolant on
//	xio_queue_RX_string_usb("m9\n");			// all coolant off

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

/* JSON TEST CASES */
// If you want to run multi-line cases you need to set RX buffer to 1024 in xio_usart.h

// JSON parser tests		  // set a group
//	xio_queue_RX_string_usb("{\"x\":{\"am\":2,\"vm\":601.000,\"fr\":1201.000,\"tm\":476.000,\"jm\":20000001.000,\"jd\":0.051,\"sn\":2,\"sv\":-502.000,\"lv\":101.000,\"lb\":2.001,\"zb\":1.001}}\n");
//	xio_queue_RX_string_usb("{\"x\":\"\"}\n"); // retrieve a group

//	xio_queue_RX_string_usb("{\"gc\":\"g0 x3 y4 z5.5 (comment line)\"}\n");
//	xio_queue_RX_string_usb("{\"xfr\":1200}\n");
//	xio_queue_RX_string_usb("{\"xfr\":1200, \"yfr\":1201, \"zfr\":600}\n");
//	xio_queue_RX_string_usb("{\"err_1\":36000}\n");
//	xio_queue_RX_string_usb("{\"1sa\":3.6.000}\n");
//	xio_queue_RX_string_usb("{\"sr\":\"\"}\n");				// invoke a status report
//	xio_queue_RX_string_usb("{\"sr\":{\"line\":true,\"posx\":true,\"posy\":true}}\n");	// set status report
//	xio_queue_RX_string_usb("{\"sr\":{\"line\":null,\"posx\":null,\"posy\":null}}\n");	// set status report
//	xio_queue_RX_string_usb("{\"x\":{\"am\":2,\"vm\":601.000,\"fr\":1201.000,\"tm\":476.000,\"jm\":20000001.000,\"jd\":0.051,\"sm\":2,\"sv\":-502.000,\"lv\":101.000,\"lb\":2.001,\"zb\":1.001}}\n");

//	xio_queue_RX_string_usb("{\"x\":\"\"}\n");				// x axis group display
//	xio_queue_RX_string_usb("{\"c\":\"\"}\n");				// c axis group display
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

//	xio_queue_RX_string_usb("{\"gc\":\"n10000 g0 x20\"}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"n100000 g0 x0\"}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"n1000000 g0 x20\"}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"n10000000 g0 x0\"}\n");
//	xio_queue_RX_string_usb("{\"gc\":\"n100000000 g0 x20\"}\n");

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

/***** Debug Functions ******/
#ifdef __DEBUG

/* Note: these dump routines pack a lot of characters into the USART TX buffer
 * and can kill the running instance. I'll have to figure out how to prevent that,
 * but in the mean time if you might want to use them you should go into xio_usart.h 
 * and temporarily change to the following settings:
 * 
 *	//#define BUFFER_T uint8_t		// faster, but limits buffer to 255 char max
 *	#define BUFFER_T uint16_t		// slower, but larger buffers
 *
 *	// USART ISR TX buffer size
 *	//#define TX_BUFFER_SIZE (BUFFER_T)64
 *	//#define TX_BUFFER_SIZE (BUFFER_T)128
 *	//#define TX_BUFFER_SIZE (BUFFER_T)255
 *	//#define TX_BUFFER_SIZE (BUFFER_T)256	// uint16_t buffer type is required
 *	//#define TX_BUFFER_SIZE (BUFFER_T)1024
 *	#define TX_BUFFER_SIZE (BUFFER_T)2048
 */

void dump_everything()
{
//	tg_dump_controller_state();
	mp_dump_running_plan_buffer();	
	mp_dump_runtime_state();
	st_dump_stepper_state();

	for (uint8_t i=0; i<PLANNER_BUFFER_POOL_SIZE; i++) {
		mp_dump_plan_buffer_by_index(i);
	}
}

void roll_over_and_die()
{
	tg_system_reset();
	tg_application_reset();
}

void print_scalar(const char *label, double value)
{
	fprintf_P(stderr,PSTR("%S %8.4f\n"),label,value); 
}

void print_vector(const char *label, double vector[], uint8_t count)
{
	fprintf_P(stderr,PSTR("%S"),label); 
	for (uint8_t i=0; i<count; i++) {
		fprintf_P(stderr,PSTR("  %5.4f"),vector[i]);
	} 	
	fprintf_P(stderr,PSTR("\n"));
}

#endif	// __DEBUG
