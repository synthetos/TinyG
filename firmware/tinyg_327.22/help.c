/*
 * help.h - collected help routines
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */

//#include <ctype.h>
#include <stdio.h>
#include <avr/pgmspace.h>

#include "tinyg.h"
#include "controller.h"
#include "help.h"

static void _help_postscript(void);
static void _help_status_report_advisory(void);

/*
 * help_print_general_help() - help invoked as h form the command line
 */
void help_print_general_help()
{
fprintf_P(stderr, PSTR("\n#### TinyG Help #### ["));
tg_print_version_string();
fprintf_P(stderr, PSTR("]\n"));

fprintf_P(stderr, PSTR("\
These commands are active from the GCODE command line:\n\
  !    Emergency Stop (resets EVERYTHING!)\n\
  ?    Show machine position and gcode model state\n\
  $    See and set configuration settings\n\
  $h   Show configuration help screen\n\
  h    Show this help screen\n\
"));

_help_status_report_advisory();
_help_postscript();
}
//  @    Pause and resume motion

/*
 * help_print_config_help() - help invoked as $h
 */
void help_print_config_help()
{
fprintf_P(stderr, PSTR("\n#### TinyG CONFIGURATION Help #### ["));
tg_print_version_string();
fprintf_P(stderr, PSTR("]\n"));

fprintf_P(stderr, PSTR("\
These commands are active for configuration:\n\
  $    Show general settings\n\
  $1   Show motor 1 settings (or whatever motor you want 1,2,3,4)\n\
  $x   Show X axis settings (or whatever axis you want x,y,z,a,b,c)\n\
  $m   Show all motor settings\n\
  $n   Show all axis settings\n\
  $$   Show all settings\n\
  $h   Show this help screen\n\n\
"));

fprintf_P(stderr, PSTR("\
To update settings enter a token and a value:\n\n\
  $ <token> <value>\n\n\
For example $yfr800 to set the Y max feed rate to 800 mm/minute\n\
For example $2sa1.8 to set motor 2 step angle to 1.8 degrees per step\n\
Input is very forgiving of caps, spaces and extra characters\n\
The value taken will be echoed back to the console\n\
"));

_help_status_report_advisory();
_help_postscript();
}

/*
 * help_print_test_mode_help() - help invoked as h from test mode
 */
void help_print_test_mode_help()
{
fprintf_P(stderr, PSTR("\n#### TinyG TEST MODE Help #### ["));
tg_print_version_string();
fprintf_P(stderr, PSTR("]\n"));

fprintf_P(stderr, PSTR("\
Commands supported in TEST mode:\n\
  g    Re-enter Gcode mode with an of G, M, N, F, Q, $\n\
  t    Run a test (1 - n)\n\
  d    Enter direct drive mode\n\
  h    Show this help screen\n\
"));

_help_postscript();
}

// help helper functions (snicker)

static void _help_postscript()
{
fprintf_P(stderr, PSTR("\n\
For detailed TinyG info see: http://www.synthetos.com/wiki/index.php?title=Projects:TinyG\n\
For the latest firmware see: https://github.com/synthetos/TinyG\n\
Please log any issues at http://www.synthetos.com/forums\n\
Have fun\n"));
} 

static void _help_status_report_advisory()
{
fprintf_P(stderr, PSTR("\n\
Note: As of version 0.911 TinyG generates a status report by default\n\
which looks something like: {xyz:[0.813,1.292,-0.125], vel:23.62, mm:0}\n\
This can be disabled by entering $se0\n\
See the wiki for more details.\n\
"));
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

