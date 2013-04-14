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
#include <util/delay.h>				// for tests

#include "tinyg.h"
#include "network.h"
#include "controller.h"
#include "gpio.h"
#include "system.h"
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
	if (tg.network_mode == NETWORK_SLAVE) {
		tg_init(XIO_DEV_RS485, XIO_DEV_USB, XIO_DEV_USB);
	}
	xio_enable_rs485_rx();		// needed for clean start
}

void net_forward(unsigned char c)
{
	xio_putc(XIO_DEV_RS485, c);	// write to RS485 port
}

/* 
 * net_test_rxtx() - test transmission from master to slave
 * net_test_loopback() - test transmission from master to slave and looping back
 */

uint8_t net_test_rxtx(uint8_t c) 
{
	int d;

	// master operation
	if (tg.network_mode == NETWORK_MASTER) {
		if ((c < 0x20) || (c >= 0x7F)) { c = 0x20; }
		c++;
		xio_putc(XIO_DEV_RS485, c);			// write to RS485 port
		xio_putc(XIO_DEV_USB, c);			// write to USB port
		_delay_ms(2);

	// slave operation
	} else {
		if ((d = xio_getc(XIO_DEV_RS485)) != _FDEV_ERR) {
			xio_putc(XIO_DEV_USB, d);
		}
	}
	return (c);
}

uint8_t net_test_loopback(uint8_t c)
{
	if (tg.network_mode == NETWORK_MASTER) {
		// send a character
		if ((c < 0x20) || (c >= 0x7F)) { c = 0x20; }
		c++;
		xio_putc(XIO_DEV_RS485, c);			// write to RS485 port
		
		// wait for loopback character
		while (true) {
			if ((c = xio_getc(XIO_DEV_RS485)) != _FDEV_ERR) {
				xio_putc(XIO_DEV_USB, c);			// write to USB port
			}
		}
	} else {
		if ((c = xio_getc(XIO_DEV_RS485)) != _FDEV_ERR) {
			xio_putc(XIO_DEV_RS485, c);			// write back to master
			xio_putc(XIO_DEV_USB, c);			// write to slave USB
		}
	}
	_delay_ms(2);
	return (c);
}

