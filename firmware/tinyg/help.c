/*
 * help.h - collected help routines
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "tinyg.h"		// #1
#include "config.h"		// #2
#include "report.h"
#include "help.h"

#ifdef __cplusplus
extern "C"{
#endif

// help helper functions (snicker)

stat_t help_stub(nvObj_t *nv) {return (STAT_OK);}

#ifdef __HELP_SCREENS

static void _status_report_advisory()
{
fprintf_P(stderr, PSTR("\n\
Note: TinyG generates automatic status reports by default\n\
This can be disabled by entering $sv=0\n\
See the wiki below for more details.\n\
"));
}

static void _postscript()
{
fprintf_P(stderr, PSTR("\n\
For detailed TinyG info see: https://github.com/synthetos/TinyG/wiki/\n\
For the latest firmware see: https://github.com/synthetos/TinyG\n\
Please log any issues at http://www.synthetos.com/forums\n\
Have fun\n"));
}

/*
 * help_general() - help invoked as h from the command line
 */
uint8_t help_general(nvObj_t *nv)
{
fprintf_P(stderr, PSTR("\n\n\n### TinyG Help ###\n"));
fprintf_P(stderr, PSTR("\
These commands are active from the command line:\n\
 ^x             Reset (control x) - software reset\n\
  ?             Machine position and gcode model state\n\
  $             Show and set configuration settings\n\
  !             Feedhold - stop motion without losing position\n\
  ~             Cycle Start - restart from feedhold\n\
  h             Show this help screen\n\
  $h            Show configuration help screen\n\
  $test         List self-tests\n\
  $test=N       Run self-test N\n\
  $home=1       Run a homing cycle\n\
  $defa=1       Restore all settings to \"factory\" defaults\n\
"));
_status_report_advisory();
_postscript();
rpt_print_system_ready_message();
return(STAT_OK);
}

/*
 * help_config() - help invoked as $h
 */
stat_t help_config(nvObj_t *nv)
{
fprintf_P(stderr, PSTR("\n\n\n### TinyG CONFIGURATION Help ###\n"));
fprintf_P(stderr, PSTR("\
These commands are active for configuration:\n\
  $sys Show system (general) settings\n\
  $1   Show motor 1 settings (or whatever motor you want 1,2,3,4)\n\
  $x   Show X axis settings (or whatever axis you want x,y,z,a,b,c)\n\
  $m   Show all motor settings\n\
  $q   Show all axis settings\n\
  $o   Show all offset settings\n\
  $$   Show all settings\n\
  $h   Show this help screen\n\n\
"));

fprintf_P(stderr, PSTR("\
Each $ command above also displays the token for each setting in [ ] brackets\n\
To view settings enter a token:\n\n\
  $<token>\n\n\
For example $yfr to display the Y max feed rate\n\n\
To update settings enter token equals value:\n\n\
  $<token>=<value>\n\n\
For example $yfr=800 to set the Y max feed rate to 800 mm/minute\n\
For configuration details see: https://github.com/synthetos/TinyG/wiki/TinyG-Configuration\n\
"));
_status_report_advisory();
_postscript();
return(STAT_OK);
}

/*
 * help_test() - help invoked for tests
 */
stat_t help_test(nvObj_t *nv)
{
fprintf_P(stderr, PSTR("\n\n\n### TinyG SELF TEST Help ###\n"));
fprintf_P(stderr, PSTR("\
Invoke self test by entering $test=N where N is one of:\n\
  $test=1  smoke test\n\
  $test=2  homing test   (you must trip homing switches)\n\
  $test=3  square test   (a series of squares)\n\
  $test=4  arc test      (some large circles)\n\
  $test=5  dwell test    (moves spaced by 1 second dwells)\n\
  $test=6  feedhold test (enter ! and ~ to hold and restart, respectively)\n\
  $test=7  M codes test  (M codes intermingled with moves)\n\
  $test=8  JSON test     (motion test run using JSON commands)\n\
  $test=9  inverse time test\n\
  $test=10 rotary motion test\n\
  $test=11 small moves test\n\
  $test=12 slow moves test\n\
  $test=13 coordinate system offset test (G92, G54-G59)\n\
\n\
Tests assume a centered XY origin and at least 80mm clearance in all directions\n\
Tests assume Z has at least 40mm posiitive clearance\n\
Tests start with a G0 X0 Y0 Z0 move\n\
Homing is the exception. No initial position or clearance is assumed\n\
"));
_postscript();
return(STAT_OK);
}

/*
 * help_defa() - help invoked for defaults
 */
stat_t help_defa(nvObj_t *nv)
{
fprintf_P(stderr, PSTR("\n\n\n### TinyG RESTORE DEFAULTS Help ###\n"));
fprintf_P(stderr, PSTR("\
Enter $defa=1 to reset the system to the factory default values.\n\
This will overwrite any changes you have made.\n"));
_postscript();
return(STAT_OK);
}

/*
 * help_boot_loader()
 */
stat_t help_boot_loader(nvObj_t *nv)
{
fprintf_P(stderr, PSTR("\n\n\n### TinyG BOOT LOADER Help ###\n"));
fprintf_P(stderr, PSTR("\
Enter $boot=1 to enter the boot loader.\n"));
_postscript();
return(STAT_OK);
}

#endif // __HELP_SCREENS

#ifdef __cplusplus
}
#endif
