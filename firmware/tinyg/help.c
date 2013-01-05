/*
 * help.h - collected help routines
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

//#include <ctype.h>
#include <stdio.h>
#include <avr/pgmspace.h>

#include "tinyg.h"
#include "util.h"
#include "config.h"
#include "controller.h"
#include "help.h"

static void _status_report_advisory(void);
static void _postscript(void);

/*
 * print_general_help() - help invoked as h from the command line
 */
uint8_t print_general_help()
{
fprintf_P(stderr, PSTR("\n\n\n#### TinyG Help ####\n"));
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
  $test-N       Run self-test N\n\
  $home=1       Run a homing cycle\n\
  $defaults=1   Restore all settings to \"factory\" defaults\n\
"));
_status_report_advisory();
_postscript();
tg_print_system_ready_message();
return(TG_OK);
}

/*
 * print_config_help() - help invoked as $h
 */
uint8_t print_config_help(cmdObj *cmd)
{
fprintf_P(stderr, PSTR("\n\n\n#### TinyG CONFIGURATION Help ####\n"));
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
return(TG_OK);
}

/*
 * print_test_help() - help invoked for tests
 */
uint8_t print_test_help(cmdObj *cmd)
{
fprintf_P(stderr, PSTR("\n\n\n#### TinyG SELF TEST Help ####\n"));
fprintf_P(stderr, PSTR("\
Invoke self test by entering $test=N where N is one of:\n\
  $test=1  homing test   (you must trip homing switches)\n\
  $test=2  smoke test\n\
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
"));
_postscript();
return(TG_OK);
}

/*
 * print_defaults_help() - help invoked for defaults
 */
uint8_t print_defaults_help(cmdObj *cmd)
{
fprintf_P(stderr, PSTR("\n\n\n#### TinyG RESTORE DEFAULTS Help ####\n"));
fprintf_P(stderr, PSTR("\
Enter $defaults=1 to reset the system to the factory default values.\n\
This will overwrite any changes you have made.\n"));
_postscript();
return(TG_OK);
}

/*
 * print_boot_loader_help()
 */
uint8_t print_boot_loader_help(cmdObj *cmd)
{
fprintf_P(stderr, PSTR("\n\n\n#### TinyG BOOT LOADER Help ####\n"));
fprintf_P(stderr, PSTR("\
Enter $boot=1 to enter the boot loader.\n"));
_postscript();
return(TG_OK);
}

// help helper functions (snicker)

static void _status_report_advisory()
{
fprintf_P(stderr, PSTR("\n\
Note: TinyG generates automatic status reports by default\n\
This can be disabled by entering $si=0\n\
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


//****************************
//***** diagnostic dumps *****
//****************************

void dump_set_f_dda(double f_dda,
					double dda_substeps, 
					double major_axis_steps, 
					double microseconds,
					double f_dda_base)
{
/* UNCOMMENT IF YOU NEED THIS
fprintf_P(stderr, PSTR("dump_set_f_dda()\n\
  f_dda            %f\n\
  f_dda_base       %f\n\
  dda_substeps     %f\n\
  major_axis_steps %f\n\
  microseconds     %f\n"),

  f_dda, 
  f_dda_base,
  dda_substeps, 
  major_axis_steps, 
  microseconds);
*/
}

