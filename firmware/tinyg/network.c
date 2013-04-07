/*
 * network.c - tinyg networking protocol
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
/* 	This module is really nothing mre than a placeholder at this time.
 * 	"Networking" refers to a planned RS485 broadcast network to support
 *	multi-board configs and external RS485 devices such as extruders.
 *	Basic operation of RS485 on the TinyG hardware has been verified
 *	using what's in this file, but you won;t find much more.
 */

#include <stdio.h>					// precursor for xio.h
#include <stdbool.h>				// true and false
#include <avr/pgmspace.h>			// precursor for xio.h

#include "network.h"
#include "controller.h"
#include "xio/xio.h"

/*
 * Local Scope Functions and Data
 */

/*
 * net_init()
 */
void net_init() 
{
	// re-point IO if in slave mode
	if (tg.network_mode == NET_SLAVE) {
		tg_init(XIO_DEV_NET, XIO_DEV_NET, XIO_DEV_NET);
	}
}

/*
void net_forward(unsigned char c)
{
	xio_putc(XIO_DEV_RS485, c);	// write to RS485 port
}
*/

/* 
 * tg_repeater()
 * tg_receiver()
 */
/*
void tg_repeater()
{
//	uint8_t full_duplex = false;
	uint8_t full_duplex = true;
	unsigned char tx = 'Z';
	unsigned char rx;

	while (true) {
		tx = _nextchar(tx);
		xio_putc(XIO_DEV_RS485, tx);	// write to RS485 port
		if (full_duplex) {
			while ((rx = xio_getc(XIO_DEV_RS485)) == -1);	// blocking read
			xio_putc(XIO_DEV_USB, rx);	// echo RX to USB port
		} else {
			xio_putc(XIO_DEV_USB, tx);	// write TX to USB port
		}	
//		gpio_toggle_port(1);
//		_delay_ms(10);
	}
}

void tg_receiver()
{
//	tg_controller();	// this node executes gcode blocks received via RS485

//	int	getc_code = 0;
	int rx;

	xio_queue_RX_string_usart(XIO_DEV_RS485, "Z");		// simulate an RX char

	while (true) {
		while ((rx = xio_getc(XIO_DEV_RS485)) == -1);
		xio_putc(XIO_DEV_RS485, rx);	// write to RS485 port
//		xio_putc_rs485(rx, fdev_rs485);	// alternate form of above
//		gpio_toggle_port(1);
	}
}

static char _nextchar(char c)
{
//	uint8_t cycle = false;
	uint8_t cycle = true;
	char n = c;

	if ((cycle) && ((n = ++c) > 'z')) {
		n = '0';
	}
	return (n);
}
*/
