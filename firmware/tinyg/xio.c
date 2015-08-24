/*
 * xio.c - Xmega IO devices - common code file
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart Jr.
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
#include "tinyg.h"					// needed by init() for default source
#include "config.h"					// needed by init() for default source
#include "controller.h"				// needed by init() for default source
#include "report.h"
#include "util.h"

#define __LINEMODE

//char single_char_buffer[2] = { NUL, NUL };

static char_t *_readline_stream(devflags_t *flags, uint16_t *size);
static char_t *_readline_packet(devflags_t *flags, uint16_t *size);
static char_t *_readline_linemode(devflags_t *flags, uint16_t *size);

static void _init_readline_stream(void);
static void _init_readline_packet(void);
//static void _init_readline_charmode(void);
static void _init_readline_linemode(void);

/********************************************************************************
 * XIO Initializations, Resets and Assertions
 */
/*
 * xio_init() - initialize entire xio sub-system
 */
void xio_init()
{
	// set memory integrity check
	xio_set_stderr(0);				// set a bogus value; may be overwritten with a real value

	memset(&xio, 0, sizeof(xioSingleton_t));	// clear all values

	// setup device types
	xio_init_usart();
	xio_init_spi();
	xio_init_file();

	// open individual devices (file device opens occur at time-of-use)
	xio_open(XIO_DEV_USB,  0, USB_FLAGS);
	xio_open(XIO_DEV_RS485,0, RS485_FLAGS);
	xio_open(XIO_DEV_SPI1, 0, SPI_FLAGS);
	xio_open(XIO_DEV_SPI2, 0, SPI_FLAGS);

    // set up XIO buffers and pointers
    _init_readline_stream();
    _init_readline_packet();
    _init_readline_linemode();

	xio_init_assertions();
}

/*
 * xio_init_assertions()
 * xio_test_assertions() - validate operating state
 *
 * NOTE: xio device assertions are set up as part of xio_open_generic()
 *		 This system is kind of brittle right now because if a device is
 *		 not set up then it will fail in the assertions test. Need to fix this.
 */

void xio_init_assertions()
{
    xio.magic_start = MAGICNUM;
    xio.magic_end = MAGICNUM;
}

uint8_t xio_test_assertions()
{
	if (xio.magic_start					!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (xio.magic_end					!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_USB].magic_start		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_USB].magic_end		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_RS485].magic_start	!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_RS485].magic_end		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_SPI1].magic_start	!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_SPI1].magic_end		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_SPI2].magic_start	!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (ds[XIO_DEV_SPI2].magic_end		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
//	if (ds[XIO_DEV_PGM].magic_start		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
//	if (ds[XIO_DEV_PGM].magic_end		!= MAGICNUM) return (STAT_XIO_ASSERTION_FAILURE);
	if (stderr != xio.stderr_shadow) 				 return (STAT_XIO_ASSERTION_FAILURE);
	return (STAT_OK);
}

/*
 * xio_isbusy() - return TRUE if XIO sub-system is busy
 *
 *	This function is here so that the caller can detect that the serial system is active
 *	and therefore generating interrupts. This is a hack for the earlier AVRs that require
 *	interrupts to be disabled for EEPROM write so the caller can see if the XIO system is
 *	quiescent. This is used by the G10 deferred writeback persistence functions.
 *
 *	Idle conditions:
 *	- The serial RX buffer is empty, indicating with some probability that data is not being sent
 *	- The serial TX buffers are empty
 */

uint8_t xio_isbusy()
{
	if (xio_get_rx_bufcount_usart(&USBu) != 0) return (false);
	if (xio_get_tx_bufcount_usart(&USBu) != 0) return (false);
	return (true);
}

/*
 * xio_reset_working_flags()
 */

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

/********************************************************************************
 * PUBLIC ENTRY POINTS - access the functions via the XIO_DEV number
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

/***************************************************************************************
 * readline() - serial reader wrapper
 *
 *  Arguments:
 *   *flags - One of: DEV_IS_CTRL, DEV_IS_DATA, DEV_IS_BOTH
 *   *size  - Set max size for buffer
 *
 *  Returns:
 *    *char - which will be one of:
 *          - a valid char pointer to buffer containing a full line of text
 *          - NULL (0) if there is no text to process
 *          - _FDEV_ERR (-1) if the line overflowed the input buffer
 *
 *   *flags - returns set to one of: DEV_IS_CTRL, DEV_IS_DATA, DEV_IS_NONE
 */

char_t *readline(devflags_t *flags, uint16_t *size)
{
    if (xio.rx_mode == RX_MODE_CHAR) {
        return (_readline_stream(flags, size));
    }
#ifdef __LINEMODE
    return (_readline_linemode(flags, size));
#else
    return (_readline_packet(flags, size));
#endif
}

// ****************************************************************************
// **** LINE MODE FUNCTIONS ***************************************************
// ****************************************************************************
/*
 * Terms:
 *  - "Header" refers to the buffer control structure buf_hdr_t
 *  - "Buffer" refers to the actual character buffer RAW allocated from the rx pool
 *  - "Pool" refers to a char array from which buffers are allocated
 *  - "Free" refers to a header that is unallocated and available for use
 *  - "Used" refers to a header that has an allocated buffer and is in some state of use
 *  - "Base" refers to the bottom of the memory pool or header list
 *  - "Top" refers to the top of the memory pool or header list
 *
 * Operation:
 *  - The header list (buf) is a circular FIFO implemented as a forward linked list
 *      - "used" headers point to the dynamically allocated memory buffers (bufp)
 *      - "free" headers have no allocated memory
 *  - Headers are added to the top (newest element) and removed from the base (oldest element).
 *      - When 2 or more headers in use the used_base and used_top are 2 distinct headers
 *      - If only one header is in use then used_base and used_top point to the same header whose state != BUFFER_FREE
 *      - If no headers are in use then used_base and used_top point to the same header whose state == BUFFER_FREE
 *      - If the header queue is full (maxed out) used_base and used_top are adjacent, with no free headers in between
 *      - The used_top header is usually FILLING
 *      - The used_base header is usually FULL
 *  - Free (unused) headers start above used_top and are advanced "upwards"
 *      - All data in a free header is invalid except the BUFFER_FREE state
 *
 * Assumptions and Constraints. We can guarantee or must anticipate the following behaviors:
 *  - There can only be zero or on BUFFER_FILLING header, otherwise this in a system error
 *  - There can only be zero or one BUFFER_PROCESSING header, otherwise this in a system error
 *  - When readline() is called it should always attempt free the current BUFFER_PROCESSING, if one exists
 *      - Based on assumption that readline is only called after the controller is done with the PROCESSING buffer
 */
static void _init_readline_linemode()
{
    bm.pool_base = rx_pool;                         // base address of RX buffer pool
    bm.pool_top = rx_pool + sizeof(rx_pool);        // address of top of RX buffer pool

    bm.used_base = bm.buf;                          // initialize to first header block
    bm.used_top = bm.buf;                           // same
    bm.estd_buffers_available = RX_HEADERS;         // estimated buffers available
    bm.requested_size = RX_BUFFER_REQUESTED_SIZE;   // this parameter may be overwritten later
    for (uint8_t i=0; i<RX_HEADERS; i++) {          // initialize buffer headers
        bm.buf[i].bufnum = i;                       // ++++++ NUMBER THE HEADER AS A DIAGNOSTIC
        bm.buf[i].size = 0;
        bm.buf[i].state = BUFFER_FREE;
        bm.buf[i].bufp = bm.pool_base;              // point all bufs to the base of RAM
        bm.buf[i].pv = &(bm.buf[i-1]);              // link the buffers via pv
        bm.buf[i].nx = &(bm.buf[i+1]);              // link the buffers via nx
    }
    bm.buf[0].pv = &bm.buf[RX_HEADERS-1];           // close the pv loop
    bm.buf[RX_HEADERS-1].nx = bm.buf;               // close the nx loop

    // +++++ DIAGNOSTIC
    char fake_rx[] = {"g0x10\n g0x0\n"}; // cannot exceed the length of the RX buffer
    xio_queue_RX_string_usb(fake_rx);
}

#pragma GCC optimize ("O0")

/*
 * _get_free_buffer() - get lowest free buffer from _CTRL or _DATA. Allocate space
 *
 *   Args:
 *    - requested_size - will fail if the requested size cannot be allocated
 *
 *   Returns:
 *    - char * pointer to a buffer of size at least 'size'
 *    - NULL if this the request is not possible
 */

static char *_get_free_buffer(uint16_t requested_size)
{
    buf_mgr_t *b = &bm;                             // pointer to buffer manager

    // setup base and top pointers and look for no-free-headers case
    buf_hdr_t *h = b->used_top;                             // top of used list
    buf_hdr_t *f = h->nx;                                   // pointer to free header
    if ((h == b->used_base) && (h->state == BUFFER_FREE)) { // if there are zero used buffers
        f = h;
    }
    if (f->state != BUFFER_FREE) {                          // buffer headers are maxed out
        return (NULL);                                      // this happens if there are no more free headers left
    }

    // attempt to allocate free RAM above the used_top
    f->bufp = h->bufp + h->size +1;                         // set the buffer pointer just past the used_top
    if ((f->bufp < b->pool_base) || (f->bufp > b->pool_top)) { // never supposed to happen, but protects against memory faults
        f->bufp = b->pool_base;
    }
    if ((b->pool_top - f->bufp) > requested_size) {         // is there enough RAM at the top?
        f->size = b->pool_top - f->bufp;                    // claim all available RAM (no reason not to)

    // if not, attempt to allocate free RAM below the used_base (wraparound case)
    } else if ((b->used_base->bufp - b->pool_base) > requested_size) {
        f->bufp = b->pool_base;                             // reset the pointer to the base
        f->size = b->used_base->bufp - b->pool_base;        // claim all available RAM

    // this happens if there is insufficient RAM left
    } else {
        return (NULL);
    }

    f->state = BUFFER_FILLING;      // claim the header
    b->used_top = f;                // advance the top to the filling buffer
    return (f->bufp);
}

/*
 * _get_filling_buffer() - get char * pointer to the currently filling buffer or NULL if none found
 *
 * If there is a filling buffer it will always be found at the top of the used headers
 */

static char *_get_filling_buffer()
{
    if (bm.used_top->state == BUFFER_FILLING) {
        return (bm.used_top->bufp);
    }
    return (NULL);  // This is OK. Didn't have an open buffer that was filling
}

/*
 * _post_buffer() - post a FILLING buffer to FULL status
 *
 *  Perform the following line pre-processing functions:
 *    - Strip whitespace and leading terminations (CR/LF)
 *  Set they flags and truncate the size, giving back unused chars to the pool.
 */

static void _post_buffer(char *bufp)
{
    char c;
    buf_mgr_t *b = &bm;
    buf_hdr_t *h = b->used_top;                     // posting buffer is always at top of used list

    // clean up the buffer by cursoring past any leading white space and blank lines
    for (uint8_t i=0; i<h->size; i++, h->bufp++) {  // shouldn't ever finish the iteration - here for protection
        c = *(h->bufp);
        if ((c == CR) || (c == LF)) {               // blank line. Undo the buffer and return
            h->state = BUFFER_FREE;
            if (h->pv != BUFFER_FREE) {
                b->used_top = h->pv;
            }
            return;
        }
        if (c <= ' ') {                             // white space
            continue;
        }
        break;
    }
    h->size = strlen(bufp) + 1;                     // set actual size; account for terminating NUL

    // set flags for buffer
    if (strchr("{$?!~%Hh", c) != NULL) {            // a match indicates control line
        h->flags = DEV_IS_CTRL;
    } else {
        h->flags = DEV_IS_DATA;
    }
    h->state = BUFFER_FULL;
}

/*
 * _next_buffer_to_process() - Search ctrl first, then data; send NULL if nothing to process
 *
 * Assumes that the buffer to process is always lowest ctrl or data of the used list
 */

static char *_next_buffer_to_process(devflags_t *flags)
{
    buf_hdr_t *h = bm.used_base;                        // pointer to first used block

    // scan the used list for the lowest ctrl header
    if (*flags & DEV_IS_CTRL) {                         // use '&' to allow DEV_IS_BOTH
        for (uint8_t i=0; i<RX_HEADERS; i++, h=h->nx) { // only process headers once
            if (h->state == BUFFER_FREE) {              // terminate the scan
                break;
            }
            if ((h->state == BUFFER_FULL) && (h->flags & DEV_IS_CTRL)) {
                *flags = (DEV_IS_CTRL);                 // set as control only
                h->state = BUFFER_PROCESSING;
                return (h->bufp);
            }
        }
    }

    // next scan the used list for the lowest data header
    h = bm.used_base;                                   // reset pointer to first used block
    if (*flags & DEV_IS_DATA) {                         // use '&' to allow DEV_IS_BOTH
        for (uint8_t i=0; i<RX_HEADERS; i++, h=h->nx) { // make sure you only process the header queue once
            if (h->state == BUFFER_FREE) {              // exit the scan
                break;
            }
            if ((h->state == BUFFER_FULL) && (h->flags & DEV_IS_DATA)) {
                *flags = (DEV_IS_DATA);
                h->state = BUFFER_PROCESSING;
                return (h->bufp);
            }
        }
    }

    // This is OK. Didn't have anything to process
    *flags = (DEV_IS_NONE);
    return (NULL);
}

/*
 * _free_processed_buffer() - return the processing buffer to the free list or exit silently
 *
 *  The buffer to free will be at the base if the used list.
 *  Invalidate bufp because we can't know what it's eventually going to become
 */

static void _free_processed_buffer()
{
    buf_mgr_t *b = &bm;
    buf_hdr_t *h = b->used_base;                        // pointer to first used block

    // scan the used list for the PROCESSING header - if one exists. It may not
    for (uint8_t i=0; i<RX_HEADERS; i++, h=h->nx) {     // make sure you only process the header queue once
        if (h->state == BUFFER_PROCESSING) {            // free it - with conditions
            if (h->state == BUFFER_FREE) {              // exit the scan
                break;
            }
            if (h == b->used_base) {                    // processing buffer is the base
                h->bufp = NULL;
                h->state = BUFFER_FREE;
                if (h != b->used_top) {                 // adjust base
                    b->used_base = b->used_base->nx;
                }
            } else if (h == b->used_top) {              // processing buffer is the top
                h->bufp = NULL;
                h->state = BUFFER_FREE;
                if (h->pv->state == BUFFER_FREE) {      // adjust top
                    b->used_top = h->pv;
                }
            } else {                                    // buffer is in the middle of used list
                h->state = BUFFER_FRAGMENT;
                b->fragments++;
            }
            break;
        }
    }
}

/*
 * _readline_linemode() - read a line using dynamic allocation
 *
 * Operation summarized as:
 *    1. free PROCESSING buffer (either frees or no-op)
 *    2. get FILLING buffer and continue to fill if one was returned
 *    3. if no FILLING buffer get a FREE buffer (which becomes a new FILLING buffer)
 *    4. read from RX into FILLING buffer
 *      4a. if buffer is not complete, exit
 *      4b. if buffer is complete: post buffer
 *    5. get next buffer to process and return as char *
 */

static char *_readline_linemode(devflags_t *flags, uint16_t *size)
{
    char *bufp;                 // pointer to allocated buffer
    stat_t status;
    buf_mgr_t *b = &bm;

	// Free a previously processing buffer (assumes calling readline means a free should occur)
    _free_processed_buffer();                           // 1. Free a buffer is one is processing

	// Resume a partially filled buffer if one exists
	// NB: xio_gets_usart() can return overflowed lines, these are truncated and terminated
    if ((bufp = _get_filling_buffer()) != NULL) {       // 2. Get a buffer if one is filling
    	status = xio_gets_usart(&ds[XIO_DEV_USB], bufp, b->requested_size);
    	if (status == XIO_EAGAIN) {
            return(_next_buffer_to_process(flags));     // no more chars to read
    	}
    	if (status == XIO_BUFFER_FULL) {
        	return ((char *)_FDEV_ERR);                 // buffer overflow occurred
    	}
        _post_buffer(bufp);                             // post your newly filled buffer
    }

    // Get a new free buffer
    if ((bufp = _get_free_buffer(b->requested_size)) == NULL) {   // 4.
        return(_next_buffer_to_process(flags));         // no buffer available
    }
    if ((status = xio_gets_usart(&ds[XIO_DEV_USB], bufp, b->requested_size)) == XIO_EAGAIN) {
        return(_next_buffer_to_process(flags));     // buffer is not yet full
    }
    if (status == XIO_BUFFER_FULL) {
        return ((char *)_FDEV_ERR);                 // buffer overflow occurred
    }
    _post_buffer(bufp);                             // post your newly filled buffer
    return(_next_buffer_to_process(flags));
}

#pragma GCC reset_options

// ****************************************************************************
// **** STREAMING MODE FUNCTIONS **********************************************
// ****************************************************************************
/*
 * _readline_stream() - character-mode serial reader (streaming)
 *
 *	Arguments:
 *	  - Flags request DEV_IS_CTRL, DEV_IS_CTRL, or either (both); returns type detected.
 *	  - Size is ignored on input, set to line length on return.
 *	  - Returns pointer to buffer or NULL pointer if no data.
 *
 *	Function:
 *	  - Read active RX device(s). Return an input line or a NULL pointer if no completed line.
 *		THe *flags arg is set to indicate the type of line returned, and *size is set to
 *		length of the returned line. The *size returned includes the space taken by the
 *		terminating CR or LF, so this is one (1) more than a standard strlen().
 *
 *		Currently this function does no special handling for doubly terminated lines e.g. CRLF.
 *		In this case the first termination returns the line; the second returns a null line of *size = 1
 *
 *	  - Data Blocking: This function has a special behavior to support sending CTRLs while
 *		executing cycles. If the flags request ctrl but not data and a data line is read from
 *		the RX device, the buffer (containing a data line) will not be returned. The buffer
 *		will be held until a call is made that requests data (may request both DATA and CTRL).
 */

/*
 *_parse_control() - parse the buffer to see if its a control
 *
 * Parsing for control is somewhat naiive. This may need to get better
 * Note: this function is used by both streaming mode and packet mode
 */
static bool _parse_control(char *p)
{
    if (strchr("{$?!~%Hh", *p) != NULL) {		        // a match indicates control line
        return (true);
    }
    return (false);
};

static void _init_readline_stream()
{
    xio.bufp = xio.in_buf;                              // pointer for streaming readline
}

static char_t *_exit_line(devflags_t flag, devflags_t *flags, uint16_t *size)
{
	*flags = flag;
	*size = xio.buf_size;
	return (xio.bufp);
}

static char_t *_exit_null(devflags_t *flags, uint16_t *size)
{
	*size = 0;
	*flags = DEV_IS_NONE;
	return ((char_t *)NULL);
}

static char_t *_readline_stream(devflags_t *flags, uint16_t *size)
{
	// Handle cases where you are already holding a completed buffer
	if (xio.buf_state == BUFFER_FULL) {
		if (*flags & DEV_IS_DATA) {
			xio.buf_state = BUFFER_FREE;				// indicates it's OK to start filling this buffer again
			return (_exit_line(DEV_IS_DATA, flags, size));
		} else {
			return(_exit_null(flags, size));
		}
	}
	// Read the input device and process the line
	stat_t status;
	if ((status = xio_gets(xio.primary_src, xio.bufp, RX_CHAR_BUFFER_LEN)) == XIO_EAGAIN) {
		return(_exit_null(flags, size));
	}
	xio.buf_size = strlen(xio.bufp)+1;                  // set size. Add 1 to account for the terminating CR or LF

	//*** got a full buffer ***
	if (status == STAT_EOF) {							// EOF can come from file devices only
		if (cs.comm_mode == TEXT_MODE) {
			fprintf_P(stderr, PSTR("End of command file\n"));
		} else {
//			rpt_exception(STAT_EOF, NULL);				// not really an exception
			rpt_exception(STAT_EOF);				    // not really an exception
		}
		controller_reset_source();						// reset to active source to default source
	}
	if (*(xio.bufp) == NUL) {                           // look for lines with no data (nul)
		return (_exit_line(DEV_IS_NONE, flags, size));
	}
	if (_parse_control(xio.bufp)) {                     // true indicates control line
		return (_exit_line(DEV_IS_CTRL, flags, size));
	}
	if (*flags & DEV_IS_DATA) {							// got a data line
		return (_exit_line(DEV_IS_DATA, flags, size));	// case where it's OK to return the data line
	}
	xio.buf_state = BUFFER_FULL;						// case where it's not OK to return the DATA line
	return(_exit_null(flags, size));
}


// ****************************************************************************
// **** PACKET MODE FUNCTIONS *************************************************
// ****************************************************************************
/*
 * readline_packet() - line-mode reader (packet mode)
 *
 *	This function reads a full line of characters from an input device (e.g. USB) into a line
 *  buffer (packet). It keeps multiple line buffers, and returns a completed buffer according
 *  to the rules below.
 *
 *	Single Device Reads (reads from USB port only)
 *	 This case reads both ctrl and data packets from the USB device
 *	  - Step 1) Read all data from the input device:
 *		- Read from the USB's RX queue into the currently filling slot buffer. Keep reading
 *		  and filling up slot buffers until the USB device has no more characters or there
 *		  are no more slots available.
 *		- As completed lines are read parse them and mark them as a control or data packet.
 *		- Discard blank lines (single NUL character)
 *		- Annotate stored packets with an incrementing sequence number.
 *	  - Step 2) When done reading:
 *		- Return the control packet with the lowest sequence number
 *		- If there are no control packets return the data packet with the lowest sequence number
 *		- Return with no data if there are no pending control or data packets
 *
 *	Multiple Device Reads (USB port and FLASH or other mass storage device)
 *	 In this case the USB port is treated as ctrl and the mass storage port is treated as data
 *	  - Step 1) Read from USB:
 *		- Read from the USB's RX queue into the currently filling slot buffer. Keep reading
 *		  and filling up slot buffers until the USB device has no more characters or there
 *		  are no more slots available.
 *		- As completed lines are read parse them as a control or data packet.
 *		- Discard blank lines and data lines
 *		- Annotate stored packets with an incrementing sequence number.
 *	  - Step 2) When done reading:
 *		- Return the control packet with the lowest sequence number.
 *		- If there are no control packet read and return a packet from the data device
 *		- Return with no data and an EOF flag if there are no pending control or data packets
 *		- When EOF is encountered revert to Single Device mode (USB only)
 *
 *	ARGS:
 *
 *	 flags - Returns the type of packet returned - DEV_IS_CTRL, DEV_IS_DATA,
 *                                                 or DEV_IS_NONE (0) if no packet is returned.
 *
 *   size -  Does nothing. Returns zero. Here for compatibility with ARM readline()
 *
 *	 char_t * Returns a pointer to the buffer containing the packet,
 *            or NULL (*0) if no text,
 *            of _FDEV_ERR (-1) if a buffer overflow occurred
 *
 *  Notes:
 *
 *  - Only Single Device Read is currently implemented
 *
 *	- Accepts CR or LF as line terminator. Replaces CR or LF with NUL in the returned string.
 *
 *  - Assumes synchronous operation. The readline() caller (controller) must completely finish
 *		with the the returned line (PROCESSING state) before calling readline() again. When
 *		readline() is called it frees the PROCESSING buffer.
 *
 *	- The number of available (free) buffers reported back in the footer or the packet report
 *      will always be 2 less than the number of free buffers you think you should have.
 *      This is because there is always a buffer FILLING, and there is always a buffer PROCESSING.
 */

uint8_t xio_get_packet_slots()
{
    uint8_t free = 0;

    for (uint8_t i=0; i<RX_PACKET_SLOTS; i++) {
        if (xio.slot[i].state == BUFFER_FREE) {
            free++;
        }
    }
    return (free);
}

void _init_readline_packet()
{
    for (uint8_t i=0; i<RX_PACKET_SLOTS; i++) {
        xio.slot[i].bufp = packet_bufs[i];
    }
}

#ifndef __LINEMODE

// starting on slot s, return the index of the first slot with a given state
static int8_t _get_next_slot(int8_t s, cmBufferState state)
{
	while (s < RX_PACKET_SLOTS) {
		if (xio.slot[s].state == state) {
            return (s);
        }
		s++;
	}
	return (-1);
}

// return lowest sequence-numbered slot for for the given state
static int8_t _get_lowest_seqnum_slot(cmBufferState state)
{
	int8_t slot = -1;
	uint32_t seqnum = MAX_ULONG;

	for (uint8_t s=0; s < RX_PACKET_SLOTS; s++) {
		if ((xio.slot[s].state == state) && (xio.slot[s].seqnum < seqnum)) {
			seqnum = xio.slot[s].seqnum;
			slot = s;
		}
	}
	return (slot);
}

// read slot contents. discard nuls, mark as CTRL or DATA, set seqnum
static void _mark_slot(int8_t s)
{
    char *p = xio.slot[s].bufp;                          // buffer pointer

    // discard null buffers
	if (*p == NUL) {
		xio.slot[s].state = BUFFER_FREE;
		return;											// return if no data present
	}

    // skip leading whitespace & quotes
    while ((*p == SPC) || (*p == TAB) || (*p == '"')) { p++; }

	// mark slot w/sequence number and command type
	xio.slot[s].seqnum = xio.next_slot_seqnum++;
	if (_parse_control(p)) {		                    // true indicates control line
		xio.slot[s].state = BUFFER_CTRL;
	} else {
		xio.slot[s].state = BUFFER_DATA;
	}
}

static char_t *_return_slot(devflags_t *flags) // return the lowest seq ctrl, then the lowest seq data, or NULL
{
	int8_t s;

    if (*flags & DEV_IS_CTRL) {                                 // scan for CTRL slots
	    if ((s = _get_lowest_seqnum_slot(BUFFER_CTRL)) != -1) {
		    xio.slot[s].state = BUFFER_PROCESSING;
		    *flags = DEV_IS_CTRL;
		    return (xio.slot[s].bufp);                           // return CTRL slot
	    }
    }
    if (*flags & DEV_IS_DATA) {                                 // scan for DATA slots
	    if ((s = _get_lowest_seqnum_slot(BUFFER_DATA)) != -1) {
		    xio.slot[s].state = BUFFER_PROCESSING;
		    *flags = DEV_IS_DATA;
		    return (xio.slot[s].bufp);                           // return DATA slot
	    }
    }
	*flags = DEV_IS_NONE;										// got no data
    return ((char_t *)NULL);									// there was no slot to return
}

static char_t *_return_on_overflow(devflags_t *flags, int8_t slot)  // buffer overflow return
{
    rpt_exception(STAT_BUFFER_FULL); // when porting this to 0.98 include a truncated version of the line in the exception report
//    printf(xio.slot[slot].buf);
    xio.slot[slot].state = BUFFER_FREE;
    *flags = DEV_IS_NONE;                                       // got no data
    return ((char_t *)_FDEV_ERR);                               // buffer overflow occurred
}

static char_t *_readline_packet(devflags_t *flags, uint16_t *size)
{
	int8_t s=0;										// slot index
    stat_t stat;

	// Free a previously processing slot (assumes calling readline() means a free should occur)
	if ((s = _get_next_slot(0, BUFFER_PROCESSING)) != -1) {  // this is OK. skip the free
		xio.slot[s].state = BUFFER_FREE;
	}

	// Look for a partially filled slot if one exists
	// NB: xio_gets_usart() can return overflowed lines, these are truncated and terminated
	if ((s = _get_next_slot(0, BUFFER_FILLING)) != -1) {
        stat = xio_gets_usart(&ds[XIO_DEV_USB], xio.slot[s].bufp, RX_PACKET_LEN);
    	if (stat == (stat_t)XIO_EAGAIN) {
        	return (_return_slot(flags));			// no more characters to read. Return an available slot
    	}
    	if (stat == (stat_t)XIO_BUFFER_FULL) {
            return (_return_on_overflow(flags, s));
        }
    	_mark_slot(s);								// mark the completed line as ctrl or data or reject blank lines
	}

	// Now fill free slots until you run out of slots or characters
	s=0;
	while ((s = _get_next_slot(s, BUFFER_FREE)) != -1) {
        stat = xio_gets_usart(&ds[XIO_DEV_USB], xio.slot[s].bufp, RX_PACKET_LEN);
        if (stat == XIO_EAGAIN) {
            xio.slot[s].state = BUFFER_FILLING;	// got some characters. Declare the buffer to be filling
            return (_return_slot(flags));			// no more characters to read. Return an available slot
        }
        if (stat == XIO_BUFFER_FULL) {
            return (_return_on_overflow(flags, s));
        }
    	_mark_slot(s++);							// mark the completed line as ctrl or data or reject blank lines
	}
	return (_return_slot(flags));
}

#endif  // __LINEMODE
