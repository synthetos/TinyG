/*
 * util.c - a random assortment of useful functions
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

#include <ctype.h>
#include <stdio.h>				// precursor for xio.h
#include <stdlib.h>
//#include <math.h>
//#include <string.h>
//#include <avr/pgmspace.h>		// precursor for xio.h

#include "tinyg.h"
#include "util.h"

/*
 * isnumber() - isdigit that also accepts '.' and '-'
 */

uint8_t isnumber(char c)
{
	if (c == '.') { return (TRUE); }
	if (c == '-') { return (TRUE); }
	return (isdigit(c));
}
/*
double min3(double a, double b, double c) 
{
	if (a<b && a<c) { 
		return (a); 
	}
	if (b<a && b<c) { 
		return (b); 
	}
	return (c);
}
*/
