/*
 * help.h - collected help and assorted display routines
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

static void _help_postscript(void);

void help_print_config_help()
{
fprintf_P(stderr, PSTR("*** TinyG Configuration Help ***\n\
These commands are active for configuration:\n\
  $    Show general settings\n\
  $1   Show motor 1 settings (or whatever motor you want 1,2,3,4)\n\
  $x   Show X axis settings (or whatever axis you want x,y,z,a,b,c)\n\
  $m   Show all motor settings\n\
  $n   Show all axis settings\n\
  $$   Show all settings\n\
  $h   Show this help screen\n\n\
To update settings enter a token and a value:\n\n\
  $ <token> <value>\n\n\
For example $yfr800 to set the Y max feed rate to 800 mm/minute\n\
For example $2sa1.8 to set motor 2 step angle to 1.8 degrees per step\n\
Input is very forgiving of caps, spaces and extra characters\n\n\
The value taken will be echoed back to the console\n"));
_help_postscript();
}


void help_print_gcode_help()
{
fprintf_P(stderr, PSTR("*** TinyG GCODE Help ***\n\
These commands are active from the GCODE command line:\n\
  !    Emergency Stop\n\
  @    Pause and resume motion\n\
  ?    Show robot position and gcode model state\n\
  $    See or set config settings\n\
  $h   Show config help screen\n\
  h    Show this help screen\n\n"));
_help_postscript();
}


void help_print_test_help()
{
fprintf_P(stderr, PSTR("*** TinyG Test Screen Help ***\n\
Commands supported in TEST mode:\n\
  g    Re-enter Gcode mode with an of G, M, N, F, Q, $\n\
  t    Run a test (1 - n)\n\
  d    Enter direct drive mode\n\
  h    Show this help screen\n"));
_help_postscript();
}

// help helper functions

static void _help_postscript()
{
fprintf_P(stderr, PSTR("Please log any issues at http://synthetos.com/forums\n\
Have fun\n"));
} 

//----- diagnostic dumps ----

void dump_set_f_dda(double f_dda,
					double dda_substeps, 
					double major_axis_steps, 
					double microseconds,
					double f_dda_base)
{
fprintf_P(stderr, PSTR("Dump set_f_dda\n\
  f_dda              %f\n\
  f_dda_base         %f\n\
  dda_substeps       %f\n\
  major_axis_steps   %f\n\
  microseconds       %f\n"));
}
