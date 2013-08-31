/*
 * xio.c - Xmega IO devices - common code file
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
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
/* ----- XIO - Xmega Device System ----
 *
 * XIO provides common access to native and derived xmega devices (see table below) 
 * XIO devices are compatible with avr-gcc stdio and also provide some special functions 
 * that are not found in stdio.
 *
 * Stdio support:
 *	- http://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html
 * 	- Stdio compatible putc() and getc() functions provided for each device
 *	- This enables fgets, printf, scanf, and other stdio functions
 * 	- Full support for formatted printing is provided (including floats)
 * 	- Assignment of a default device to stdin, stdout & stderr is provided 
 *	- printf() and printf_P() send to stdout, so use fprintf() to stderr
 *		for things that should't go over RS485 in SLAVE mode 
 *
 * Facilities provided beyond stdio:
 *	- Supported devices include:
 *		- USB (derived from USART)
 *		- RS485 (derived from USART)
 *		- SPI devices and slave channels
 *		- Program memory "files" (read only)
 *	- Stdio FILE streams are managed as bindings to the above devices
 *	- Additional functions provided include:
 *		- open() - initialize parameters, addresses and flags
 *		- gets() - non-blocking input line reader - extends fgets
 *		- ctrl() - ioctl-like knockoff for setting device parameters (flags)
 *		- signal handling: interrupt on: feedhold, cycle_start, ctrl-x software reset
 *		- interrupt buffered RX and TX functions 
 *		- XON/XOFF software flow control
 */
/* ----- XIO - Some Internals ----
 *
 * XIO layers are: (1) xio virtual device (root), (2) xio device type, (3) xio devices
 *
 * The virtual device has the following methods:
 *	xio_init() - initialize the entire xio system
 *	xio_open() - open a device indicated by the XIO_DEV number
 *	xio_ctrl() - set control flags for XIO_DEV device
 *	xio_gets() - get a string from the XIO_DEV device (non blocking line reader)
 *	xio_getc() - read a character from the XIO_DEV device (not stdio compatible)
 *	xio_putc() - write a character to the XIO_DEV device (not stdio compatible)
 *  xio_set_baud() - set baud rates for devices for which this is meaningful
 *
 * The device type layer currently knows about USARTS, SPI, and File devices. Methods are:
 *	xio_init_<type>() - initializes the devices of that type
 *
 * The device layer currently supports: USB, RS485, SPI channels, PGM file reading. methods:
 *	xio_open<device>() - set up the device for use or reset the device
 *	xio_ctrl<device>() - change device flag controls
 *	xio_gets<device>() - get a string from the device (non-blocking)
 *	xio_getc<device>() - read a character from the device (stdio compatible)
 *	xio_putc<device>() - write a character to the device (stdio compatible)
 *
 * The virtual level uses XIO_DEV_xxx numeric device IDs for reference. 
 * Lower layers are called using the device structure pointer xioDev_t *d
 * The stdio compatible functions use pointers to the stdio FILE structs.
 */
#include <string.h>					// for memset()
#include <stdio.h>					// precursor for xio.h
#include <avr/pgmspace.h>			// precursor for xio.h

#include "xio.h"					// all device includes are nested here
#include "../tinyg.h"				// needed by init() for default source
#include "../config.h"				// needed by init() for default source
#include "../controller.h"			// needed by init() for default source

//
typedef struct xioSingleton {
	FILE * stderr_shadow;			// used for stack overflow / memory integrity checking
} xioSingleton_t;
xioSingleton_t xio;

/*
 * xio_init() - initialize entire xio sub-system
 * xio_reset_working_flags()
 */
void xio_init()
{
	// set memory integrity check
	xio_set_stderr(0);				// set a bogus value; may be overwritten with a real value			

	// setup device types
	xio_init_usart();
	xio_init_spi();
	xio_init_file();

	// open individual devices (file device opens occur at time-of-use)
	xio_open(XIO_DEV_USB,  0, USB_FLAGS);
	xio_open(XIO_DEV_RS485,0, RS485_FLAGS);
	xio_open(XIO_DEV_SPI1, 0, SPI_FLAGS);
	xio_open(XIO_DEV_SPI2, 0, SPI_FLAGS);
}

void xio_reset_working_flags(xioDev_t *d)
{
	d->signal = 0;
	d->flag_in_line = 0;
	d->flag_eol = 0;
	d->flag_eof = 0;
}

/*
 * xio_init_device() - generic initialization function for any device
 *
 *	This binds the main fucntions and sets up the stdio FILE structure
 *	udata is used to point back to the device struct so it can be gotten 
 *	from getc() and putc() functions. 
 *
 *	Requires device open() to be run prior to using the device
 */
void xio_open_generic(uint8_t dev, x_open_t x_open, 
								   x_ctrl_t x_ctrl, 
								   x_gets_t x_gets, 
								   x_getc_t x_getc, 
								   x_putc_t x_putc, 
								   x_flow_t x_flow)
{
	xioDev_t *d = &ds[dev];
	memset (d, 0, sizeof(xioDev_t));
	d->magic_start = MAGICNUM;
	d->magic_end = MAGICNUM;
	d->dev = dev;

	// bind functions to device structure
	d->x_open = x_open;
	d->x_ctrl = x_ctrl;
	d->x_gets = x_gets;
	d->x_getc = x_getc;	// you don't need to bind getc & putc unless you are going to use them directly
	d->x_putc = x_putc;	// they are bound into the fdev stream struct
	d->x_flow = x_flow;

	// setup the stdio FILE struct and link udata back to the device struct
	fdev_setup_stream(&d->file, x_putc, x_getc, _FDEV_SETUP_RW);
	fdev_set_udata(&d->file, d);		// reference yourself for udata 
}

/* 
 * PUBLIC ENTRY POINTS - acces the functions via the XIO_DEV number
 * xio_open() - open function 
 * xio_gets() - entry point for non-blocking get line function
 * xio_getc() - entry point for getc (not stdio compatible)
 * xio_putc() - entry point for putc (not stdio compatible)
 *
 * It might be prudent to run an assertion such as below, but we trust the callers:
 * 	if (dev < XIO_DEV_COUNT) blah blah blah
 *	else  return (_FDEV_ERR);	// XIO_NO_SUCH_DEVICE
 */
FILE *xio_open(uint8_t dev, const char *addr, flags_t flags)
{
	return (ds[dev].x_open(dev, addr, flags));
}

int xio_gets(const uint8_t dev, char *buf, const int size) 
{
	return (ds[dev].x_gets(&ds[dev], buf, size));
}

int xio_getc(const uint8_t dev) 
{ 
	return (ds[dev].x_getc(&ds[dev].file)); 
}

int xio_putc(const uint8_t dev, const char c)
{
	return (ds[dev].x_putc(c, &ds[dev].file)); 
}

/*
 * xio_ctrl() - PUBLIC set control flags (top-level XIO_DEV access)
 * xio_ctrl_generic() - PRIVATE but generic set-control-flags
 */
int xio_ctrl(const uint8_t dev, const flags_t flags)
{
	return (xio_ctrl_generic(&ds[dev], flags));
}

#define SETFLAG(t,f) if ((flags & t) != 0) { d->f = true; }
#define CLRFLAG(t,f) if ((flags & t) != 0) { d->f = false; }

int xio_ctrl_generic(xioDev_t *d, const flags_t flags)
{
	SETFLAG(XIO_BLOCK,		flag_block);
	CLRFLAG(XIO_NOBLOCK,	flag_block);
	SETFLAG(XIO_XOFF,		flag_xoff);
	CLRFLAG(XIO_NOXOFF,		flag_xoff);
	SETFLAG(XIO_ECHO,		flag_echo);
	CLRFLAG(XIO_NOECHO,		flag_echo);
	SETFLAG(XIO_CRLF,		flag_crlf);
	CLRFLAG(XIO_NOCRLF,		flag_crlf);
	SETFLAG(XIO_IGNORECR,	flag_ignorecr);
	CLRFLAG(XIO_NOIGNORECR,	flag_ignorecr);
	SETFLAG(XIO_IGNORELF,	flag_ignorelf);
	CLRFLAG(XIO_NOIGNORELF,	flag_ignorelf);
	SETFLAG(XIO_LINEMODE,	flag_linemode);
	CLRFLAG(XIO_NOLINEMODE,	flag_linemode);
	return (XIO_OK);
}

/*
 * xio_set_baud() - PUBLIC entry to set baud rate
 *	Currently this only works on USART devices
 */
int xio_set_baud(const uint8_t dev, const uint8_t baud)
{
	xioUsart_t *dx = (xioUsart_t *)&us[dev - XIO_DEV_USART_OFFSET];
	xio_set_baud_usart(dx, baud);
	return (XIO_OK);
}

/*
 * xio_fc_null() - flow control null function
 */
void xio_fc_null(xioDev_t *d)
{
	return;
}

/*
 * xio_set_stdin()  - set stdin from device number
 * xio_set_stdout() - set stdout from device number
 * xio_set_stderr() - set stderr from device number
 *
 *	stderr is defined in stdio as __iob[2]. Turns out stderr is the last RAM 
 *	allocated by the linker for this project. We usae that to keep a shadow 
 *	of __iob[2] for stack overflow detection and other memory corruption.
 */
void xio_set_stdin(const uint8_t dev) { stdin  = &ds[dev].file; }
void xio_set_stdout(const uint8_t dev) { stdout = &ds[dev].file; }
void xio_set_stderr(const uint8_t dev)
{
	stderr = &ds[dev].file; 
	xio.stderr_shadow = stderr;		// this is the last thing in RAM, so we use it as a memory corruption canary
}

/*
 * xio_assertions() - validate operating state
 *
 *	Returns status code (0 if everything is OK) 
 *	and sets a value if there is a failure.
 */
uint8_t xio_assertions(uint8_t *value)
{
	if (ds[XIO_DEV_USB].magic_start		!= MAGICNUM) { *value = 100; }
	if (ds[XIO_DEV_USB].magic_end		!= MAGICNUM) { *value = 101; }
	if (ds[XIO_DEV_RS485].magic_start	!= MAGICNUM) { *value = 102; }
	if (ds[XIO_DEV_RS485].magic_end		!= MAGICNUM) { *value = 103; }
	if (ds[XIO_DEV_SPI1].magic_start	!= MAGICNUM) { *value = 104; }
	if (ds[XIO_DEV_SPI1].magic_end		!= MAGICNUM) { *value = 105; }
	if (ds[XIO_DEV_SPI2].magic_start	!= MAGICNUM) { *value = 106; }
	if (ds[XIO_DEV_SPI2].magic_end		!= MAGICNUM) { *value = 107; }
	if (ds[XIO_DEV_PGM].magic_start		!= MAGICNUM) { *value = 108; }
	if (ds[XIO_DEV_PGM].magic_end		!= MAGICNUM) { *value = 109; }
	if (stderr != xio.stderr_shadow) 				 { *value = 200; } 

	if (*value != 0) { return (STAT_MEMORY_FAULT); }
	return (STAT_OK);
}

/*****************************************************************************
 * UNIT TESTS 
 *****************************************************************************/

#if defined (__UNIT_TESTS) && defined (__UNIT_TEST_XIO)

static void _spi_putc(void);
static void _spi_loopback(void);
static void _pgm_test(void);	

void xio_unit_tests()
{
//	_spi_putc();
	_spi_loopback();
//	_pgm_test();
}

static void _spi_putc()
{
	FILE * fdev;
	char buf[12];
//	char c;


	fdev = xio_open(XIO_DEV_SPI1, 0, SPI_FLAGS);
	while (true) {
//		xio_putc_spi(0x55, fdev);
//		c = xio_getc_spi(fdev);
		xio_gets(XIO_DEV_SPI1, buf, 12);
	}
}

#define DELAY 1000		// approx delay in uSec in -O0 mode

static void _spi_loopback()
{
	FILE * fdev;
	char buf[10] = "tester\n";
//	uint8_t i=0;
	uint32_t j=0;

	fdev = xio_open(XIO_DEV_SPI1, 0, SPI_FLAGS);
	xio_set_stdout(XIO_DEV_SPI1);
	while (true) {
//		for (i=0; i<7; i++) { xio_putc_spi(buf[i], fdev);}
		printf("%s",buf);
		for (j=(DELAY*0.76); j>0; j--);
	}
}

static void _pgm_test()	
{
	FILE * fdev;

	fdev = xio_open(XIO_DEV_PGM, 0, PGM_FLAGS);
//	xio_puts_pgm("ABCDEFGHIJKLMNOP\n", fdev);
	xio_putc_pgm('A', fdev);
	xio_putc_pgm('B', fdev);
	xio_putc_pgm('C', fdev);
	xio_getc_pgm(fdev);
	xio_getc_pgm(fdev);
	xio_getc_pgm(fdev);

}

//	fdev = xio_open(XIO_DEV_USB, 0, USB_FLAGS);
//	xio_getc_usart(fdev);


#endif
