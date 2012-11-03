/*
 * system.c - general hardware support functions
 * Part of TinyG project
 *
 * Copyright (c) 2011 - 2012 Alden S. Hart Jr.
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
 *
 * ------
 * Notes:
 *	- add full interrupt tables and dummy interrupt routine (maybe)
 *	- add crystal oscillator failover
 *	- add watchdog timer functions
 *
 */

#include <stdio.h>
#include <stddef.h> 
#include <avr\pgmspace.h> 

#include "tinyg.h"
#include "system.h"
#include "xmega/xmega_init.h"

/*
 * sys_init() - lowest level hardware init
 */

void sys_init() 
{
	xmega_init();
}

/*
 * sys_read_calibration_byte() - low-level read for system parameters
 *
 *	(from Boston Android xmega-adc-wcal.c)
 */
/*
uint8_t sys_read_calibration_byte(uint8_t index)
{ 
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc; // Load NVM Command register to read the calibration row
	uint8_t result = pgm_read_byte(index); 		 // Clean up NVM Command register 
	NVM_CMD = NVM_CMD_NO_OPERATION_gc; 
	return(result); 
}
*/
/*
 * sys_read_signature() - return the 11 byte signature row as an array
 */
/*

uint8_t ReadSignatureByte(uint16_t Address) { 
  NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc; 
  uint8_t Result; 
  __asm__ ("lpm %0, Z\n" : "=r" (Result) : "z" (Address)); 
//  __asm__ ("lpm \n  mov %0, r0 \n" : "=r" (Result) : "z" (Address) : "r0"); 
  NVM_CMD = NVM_CMD_NO_OPERATION_gc; 
  return Result; 
} 

void sys_read_signature(uint8_t sig[])
{
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc; // Load NVM Command register to read the calibration row
	uint8_t result = pgm_read_byte(index); 		
	NVM_CMD = NVM_CMD_NO_OPERATION_gc; 	// Clean up NVM Command register 


	return
}
*/
/*
void NVM_GetGUID() { 
enum { 
  LOTNUM0=8,  // Lot Number Byte 0, ASCII 
  LOTNUM1,    // Lot Number Byte 1, ASCII 
  LOTNUM2,    // Lot Number Byte 2, ASCII 
  LOTNUM3,    // Lot Number Byte 3, ASCII 
  LOTNUM4,    // Lot Number Byte 4, ASCII 
  LOTNUM5,    // Lot Number Byte 5, ASCII 
  WAFNUM =16, // Wafer Number 
  COORDX0=18, // Wafer Coordinate X Byte 0 
  COORDX1,    // Wafer Coordinate X Byte 1 
  COORDY0,    // Wafer Coordinate Y Byte 0 
  COORDY1,    // Wafer Coordinate Y Byte 1 
}; 
  byte b[11]; 
  b[ 0]=ReadSignatureByte(LOTNUM0); 
  b[ 1]=ReadSignatureByte(LOTNUM1); 
  b[ 2]=ReadSignatureByte(LOTNUM2); 
  b[ 3]=ReadSignatureByte(LOTNUM3); 
  b[ 4]=ReadSignatureByte(LOTNUM4); 
  b[ 5]=ReadSignatureByte(LOTNUM5); 
  b[ 6]=ReadSignatureByte(WAFNUM ); 
  b[ 7]=ReadSignatureByte(COORDX0); 
  b[ 8]=ReadSignatureByte(COORDX1); 
  b[ 9]=ReadSignatureByte(COORDY0); 
  b[10]=ReadSignatureByte(COORDY1); 
*/
