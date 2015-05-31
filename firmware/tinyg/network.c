/*
 * network.c - tinyg networking protocol
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* 	This module is really nothing mre than a placeholder at this time.
 * 	"Networking" refers to a planned RS485 broadcast network to support
 *	multi-board configs and external RS485 devices such as extruders.
 *	Basic operation of RS485 on the TinyG hardware has been verified
 *	using what's in this file, but you won;t find much more.
 */

#include <util/delay.h>				// for tests

#include "tinyg.h"
#include "network.h"
#include "controller.h"
#include "gpio.h"
#include "hardware.h"
#include "xio.h"

/*
 * Local Scope Functions and Data
 */

/*
 * network_init()
 */
void network_init()
{
	// re-point IO if in slave mode
	if (cs.network_mode == NETWORK_SLAVE) {
		controller_init(XIO_DEV_RS485, XIO_DEV_USB, XIO_DEV_USB);
		tg_set_secondary_source(XIO_DEV_USB);
	}
	xio_enable_rs485_rx();		// needed for clean start for RS-485;
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
	if (cs.network_mode == NETWORK_MASTER) {
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
	if (cs.network_mode == NETWORK_MASTER) {
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

