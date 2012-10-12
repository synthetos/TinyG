/*
 * xio_signals.c  - tinyg signal handling
 * Part of TinyG project
 *
 * Copyright (c) 2010 2012 - Alden S. Hart Jr.
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
 *-----
 *
 *	This file is isolated from the other xio files as it can have a lot 
 *	of application specific code.
 *
 */

#include <stdio.h>						// precursor for xio.h
#include <stdbool.h>					// true and false
#include <avr/pgmspace.h>				// precursor for xio.h
#include "../tinyg.h"
#include "xio.h"

/*
 * sig_init()			init signals
 * sig_abort()			end program (hard)
 * sig_feedhold()		stop motion
 * sig_cycle_start()	start or resume motion
 */

void sig_init()
{
	sig.sig_abort = false;
	sig.sig_feedhold = false;
	sig.sig_cycle_start = false;
}

inline void sig_abort()					// reset
{
	sig.sig_abort = true;
}

inline void sig_feedhold()				// pause
{
	sig.sig_feedhold = true;
}

inline void sig_cycle_start()			// start or resume
{
	sig.sig_cycle_start = true;
}
