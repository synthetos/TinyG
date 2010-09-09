/*
 * xio_usb.c - FTDI USB device driver for xmega family
 * 			- works with avr-gcc stdio library
 *
 * Copyright (c) 2010 Alden S. Hart, Jr.
 */

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>			// needed for blocking character reads

#include "xio.h"
#include "xio_usb.h"

/* Global scope data and functions */
// define a (statically allocated) stdio FILE struct for the USB device
FILE dev_usb = FDEV_SETUP_STREAM(xio_usb_putc, xio_usb_getc, _FDEV_SETUP_RW);

/* External data this module needs */
extern uint8_t bsel[];			// shared baud rate selection values
extern uint8_t bscale[];		// shared baud rate scaling values

/* Local scope data and functions */
static struct xioUSART f;		// create a local USART control struct
static int c;					// used locally for the following helper routines
static int _got_char(void);		// characters dispatch routines
static int _got_KILL(void);	
static int _got_PAUSE(void);
static int _got_RESUME(void);
static int _got_NUL(void);
static int _got_NEWLINE(void);
static int _got_SEMICOLON(void);
static int _got_DELETE(void);
static int _got_SHIFTOUT(void);
static int _got_SHIFTIN(void);
static void _usb_dequeue_TX(void); // shared ISR/non-ISR TX dequeue routine

/* 
 *	xio_usb_init() - initialize and set controls for USB device 
 *
 *	Control		   Arg	  Default		Notes
 *	
 *	XIO_RD		  <null>	Y	Enable device for reads
 *	XIO_WR		  <null>	Y	Enable device for write
 *	XIO_BLOCK	  <null>	Y	Enable blocking reads
 *	XIO_NOBLOCK   <null>		Disable blocking reads
 *	XIO_ECHO	  <null>	Y	Enable echo
 *	XIO_NOECHO	  <null>		Disable echo
 *	XIO_CRLF	  <null>		Send <cr><lf> if <lf> detected
 *	XIO_NOCRLF	  <null>	Y	Do not convert <lf> to <cr><lf>
 *	XIO_LINEMODE  <null>		Apply special <cr><lf> read handling
 *	XIO_NOLINEMODE <null>	Y	Do not apply special <cr><lf> read handling
 *	XIO_SEMICOLONS <null>		Treat semicolons as line breaks
 *	XIO_NOSEMICOLONS <null>	Y	Don't treat semicolons as line breaks
 *
 *	XIO_BAUD_xxxxx <null>		One of the supported baud rate enums
 */

void xio_usb_init(uint16_t control)
{
	uint8_t baud = (uint8_t)(control & XIO_BAUD_gm);

	// transfer control flags to internal flag bits
	f.flags = XIO_FLAG_USB_DEFS_gm;		// set flags to defaults & initial state
	if (control & XIO_RD) {
		f.flags |= XIO_FLAG_RD_bm;
	}
	if (control & XIO_WR) {
		f.flags |= XIO_FLAG_WR_bm;
	}
	if (control & XIO_BLOCK) {
		f.flags |= XIO_FLAG_BLOCK_bm;
	}
	if (control & XIO_NOBLOCK) {
		f.flags &= ~XIO_FLAG_BLOCK_bm;
	}
	if (control & XIO_ECHO) {
		f.flags |= XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_NOECHO) {
		f.flags &= ~XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_CRLF) {
		f.flags |= XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_NOCRLF) {
		f.flags &= ~XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_LINEMODE) {
		f.flags |= XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_NOLINEMODE) {
		f.flags &= ~XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_SEMICOLONS) {
		f.flags |= XIO_FLAG_SEMICOLONS_bm;
	}
	if (control & XIO_NOSEMICOLONS) {
		f.flags &= ~XIO_FLAG_SEMICOLONS_bm;
	}

	// character signals and error returns
	f.signals = 0;
	dev_usb.udata = &(f.signals);			// bind signals register to device FILE struct

	// setup internal RX/TX buffers
	f.rx_buf_head = 1;						// can't use location 0
	f.rx_buf_tail = 1;
	f.tx_buf_head = 1;
	f.tx_buf_tail = 1;
	f.tx_busy = FALSE;						// initialize TX queue semaphore

	// device assignment
	f.usart = &USB_USART;					// bind USART
	f.port = &USB_PORT;						// bind PORT

	// baud rate and USART setup
	if (baud == XIO_BAUD_UNSPECIFIED) {
		baud = XIO_BAUD_DEFAULT;
	}
	f.usart->BAUDCTRLA = (uint8_t)pgm_read_byte(&bsel[baud]);
	f.usart->BAUDCTRLB = (uint8_t)pgm_read_byte(&bscale[baud]);
	f.usart->CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
	// enable transmit and receive interrupts at medium level
	f.usart->CTRLA = USART_RXCINTLVL_MED_gc;
//	f.usart->CTRLA |= USART_TXCINTLVL_LO_gc;
	f.usart->CTRLA |= USART_DREINTLVL_LO_gc;

	f.port->DIRCLR = USB_RX_bm;	 			// clr RX pin as input
	f.port->DIRSET = USB_TX_bm; 			// set TX pin as output
	f.port->OUTSET = USB_TX_bm;				// set TX HI as initial state
	f.port->DIRCLR = USB_CTS_bm; 			// set CTS pin as input
	f.port->DIRSET = USB_RTS_bm; 			// set RTS pin as output
	f.port->OUTSET = USB_RTS_bm; 			// set RTS HI initially (RTS enabled)
//	f.port->OUTCLR = USB_RTS_bm; 			// set RTS HI initially (RTS enabled)
}

/*	
 *	xio_usb_control() - set controls for USB device 
 *
 *	Control		   Arg	  Default		Notes
 *	
 *	XIO_BLOCK	  <null>	Y	Enable blocking reads
 *	XIO_NOBLOCK   <null>		Disable blocking reads
 *	XIO_ECHO	  <null>	Y	Enable echo
 *	XIO_NOECHO	  <null>		Disable echo
 *	XIO_CRLF	  <null>		Send <cr><lf> if <lf> detected
 *	XIO_NOCRLF	  <null>	Y	Do not convert <lf> to <cr><lf>
 *	XIO_LINEMODE  <null>		Apply special <cr><lf> read handling
 *	XIO_NOLINEMODE <null>	Y	Do not apply special <cr><lf> read handling
 *	XIO_SEMICOLONS <null>		Treat semicolons as line breaks
 *	XIO_NOSEMICOLONS <null>	Y	Don't treat semicolons as line breaks
 *
 *	XIO_BAUD_xxxxx	<null>		One of the supported baud rate enums
 */

int8_t xio_usb_control(uint16_t control, int16_t arg)
{
	uint8_t baud = (uint8_t)(control & XIO_BAUD_gm);

	// group 1 commands (do not have argument)
	if (baud != XIO_BAUD_UNSPECIFIED) {
		f.usart->BAUDCTRLA = (uint8_t)pgm_read_byte(&bsel[baud]);
		f.usart->BAUDCTRLB = (uint8_t)pgm_read_byte(&bscale[baud]);
	}
	if (control & XIO_BLOCK) {
		f.flags |= XIO_FLAG_BLOCK_bm;
	}
	if (control & XIO_NOBLOCK) {
		f.flags &= ~XIO_FLAG_BLOCK_bm;
	}
	if (control & XIO_ECHO) {
		f.flags |= XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_NOECHO) {
		f.flags &= ~XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_CRLF) {
		f.flags |= XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_NOCRLF) {
		f.flags &= ~XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_LINEMODE) {
		f.flags |= XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_NOLINEMODE) {
		f.flags &= ~XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_SEMICOLONS) {
		f.flags |= XIO_FLAG_SEMICOLONS_bm;
	}
	if (control & XIO_NOSEMICOLONS) {
		f.flags &= ~XIO_FLAG_SEMICOLONS_bm;
	}

	// group 2 commands (have argument)	// no argument commands for this device
	return (0);
}

/* 
 * USB_RX_ISR - USB receiver interrupt (RX)
 *
 *	RX buffer states can be one of:
 *	- buffer has space	(CTS should be asserted)
 *	- buffer is full 	(CTS should be not_asserted)
 *	- buffer becomes full with this character (write char and assert CTS)
 *
 *	Flow control is not implemented. Need to work RTS line.
 *	Flow control should cut off at high water mark, re-enable at low water mark
 *	High water mark should have about 4 - 8 bytes left in buffer (~95% full) 
 *	Low water mark about 50% full
 *
 * 	See end notes in xio.h for a discussion of how the circular bufers work
 */

ISR(USB_RX_ISR_vect)		//ISR(USARTC0_RXC_vect)	// serial port C0 RX interrupt 
{
	// normal path
	if ((--f.rx_buf_head) == 0) { 				// advance buffer head with wrap
		f.rx_buf_head = RX_BUFFER_SIZE-1;		// -1 avoids the off-by-one error
	}
	if (f.rx_buf_head != f.rx_buf_tail) {		// write char unless buffer full
		f.rx_buf[f.rx_buf_head] = f.usart->DATA;// (= USARTC0.DATA;)
		return;
	}
	// buffer-full handling
	if ((++f.rx_buf_head) > RX_BUFFER_SIZE-1) { // reset the head
		f.rx_buf_head = 1;
	}
	// activate flow control here or before it gets to this level
}

/*
 * xio_usb_fake_RX_ISR() - fake ISR to put a char in the RX buffer
 */

void xio_usb_fake_RX_ISR(char cin)
{
	// normal path
	if ((--f.rx_buf_head) == 0) { 				// wrap condition
		f.rx_buf_head = RX_BUFFER_SIZE-1;		// -1 avoids the off-by-one error
	}
	if (f.rx_buf_head != f.rx_buf_tail) {		// write char unless buffer full
		f.rx_buf[f.rx_buf_head] = cin;			// FAKE INPUT DATA
		return;
	}
	// buffer-full handling
	if ((++f.rx_buf_head) > RX_BUFFER_SIZE-1) { // reset the head
		f.rx_buf_head = 1;
	}
}

/* 
 * USB_TX_ISR - USB transmitter interrupt (TX)
 */

//ISR(USARTC0_TXC_vect)
//ISR(USB_TX_ISR_vect)		//ISR(USARTC0_TXC_vect)	// serial port C0 TX interrupt 
//{
//	_usb_dequeue_TX();
//}

//ISR(USARTC0_DRE_vect)
ISR(USB_TX_ISR_vect)		//ISR(USARTC0_TXC_vect)	// serial port C0 TX interrupt 
{
	_usb_dequeue_TX();
}

/*
 * _usb_dequeue_TX() - dequeue TX buffer and write to USART
 */

void _usb_dequeue_TX()
{
	// don't re-order this code region - from here...
	if (f.tx_busy) { 	// prevents ISR from clobbering non-ISR invocation
		return;		// see explanation in stepper.c, st_execute_line()
	}
	f.tx_busy = TRUE;
	if (!(f.usart->STATUS & USART_DREIF_bm)) {
//	if (!(f.usart->STATUS & (USART_DREIF_bm | USART_TXCIF_bm))) {
		f.tx_busy = FALSE;
		return;
	}
//	f.usart->STATUS = (USART_DREIF_bm | USART_TXCIF_bm); // reset them
	if (f.tx_buf_head == f.tx_buf_tail) {		// buffer empty
		f.tx_busy = FALSE;
		return;
	}
	//...to here

	// You have the floor, the TX buffer has data, and TX DATA reg is not occupied
	// Do the write.
	if (--(f.tx_buf_tail) == 0) {				// advance tail and wrap if needed
		f.tx_buf_tail = TX_BUFFER_SIZE-1;		// -1 avoids off-by-one error (OBOE)
	}
	f.usart->DATA = f.tx_buf[f.tx_buf_tail];	// write char to TX DATA register
	f.tx_busy = FALSE;
}

/* 
 * xio_usb_putc() - blocking and nonblocking char writer for USB device 
 *
 * Notes: Originally I had the routine advancing the buffer head and comparing 
 * against the buffer tail to detect buffer full (it would sleep if the buffer was 
 * full). This unfortunately collides with the buffer empty detection in the dequeue
 * routine - causing the dequeing ISR to lock up when the buffer was full. Using 
 * a local next_tx_buffer_head prevents this.
 */

int xio_usb_putc(char c, FILE *stream)
{
	uint8_t next_tx_buf_head;

	if ((next_tx_buf_head = f.tx_buf_head-1) == 0) { 	// advance head and handle wrap
		next_tx_buf_head = TX_BUFFER_SIZE-1;		// -1 avoids the off-by-one error
	}
	while(next_tx_buf_head == f.tx_buf_tail) { 	// TX buffer full. sleep or return.
		if (BLOCKING(f.flags)) {
			_usb_dequeue_TX();
//			sleep_mode();						// SLEEP fails below 70 char buffer
			continue;							//...some kind of ISR timing bug???
		} else {
			f.signals = XIO_SIG_WOULDBLOCK;
			return(_FDEV_ERR);
		}
	};
	f.tx_buf_head = next_tx_buf_head;			// accept the next buffer head value
	f.tx_buf[f.tx_buf_head] = c;				// write char to buffer

	if (CRLF(f.flags) && (c == '\n')) {			// follow <cr> with <lf>
		return xio_usb_putc('\r', stream);		// recursion.
	}
	_usb_dequeue_TX();
	return 0;
}

/* 
 * Array of dispatch functions for received characters for xio_usb_getc()
 *
 *  Functions take no input but use static 'c', f.signals, and others
 *  Returns c (may be translated depending on the function)
 */

static int (*getcFuncs[])(void) PROGMEM = { 	// use if you want it in FLASH
//static int (*getcFuncs[])(void) = {			// ALTERNATE: put table in SRAM

						// dec  hex symbol
		_got_NUL, 		//	0	00	NUL	(Null char)
		_got_char, 		//	1	01	SOH	(Start of Header)
		_got_char, 		//	2	02	STX	(Start of Text)
		_got_KILL,	 	//	3	03	ETX (End of Text) ^c
		_got_char, 		//	4	04	EOT	(End of Transmission)
		_got_char, 		//	5	05	ENQ	(Enquiry)
		_got_char, 		//	6	06	ACK	(Acknowledgment)
		_got_char, 		//	7	07	BEL	(Bell)
		_got_DELETE, 	//	8	08	BS	(Backspace)
		_got_char, 		//	9	09	HT	(Horizontal Tab)
		_got_NEWLINE, 	//	10	0A	LF	(Line Feed)
		_got_char, 		//	11	0B	VT	(Vertical Tab)
		_got_char, 		//	12	0C	FF	(Form Feed)
		_got_NEWLINE, 	//	13	0D	CR	(Carriage Return)
		_got_SHIFTOUT,	//	14	0E	SO	(Shift Out)
		_got_SHIFTIN, 	//	15	0F	SI	(Shift In)
		_got_char, 		//	16	10	DLE	(Data Link Escape)
		_got_RESUME, 	//	17	11	DC1 (XON) (Device Control 1) ^q	
		_got_char, 		//	18	12	DC2	(Device Control 2)
		_got_PAUSE, 	//	19	13	DC3 (XOFF)(Device Control 3) ^s	
		_got_char, 		//	20	14	DC4	(Device Control 4)
		_got_char, 		//	21	15	NAK (Negativ Acknowledgemnt)	
		_got_char, 		//	22	16	SYN	(Synchronous Idle)
		_got_char, 		//	23	17	ETB	(End of Trans. Block)
		_got_KILL,	 	//	24	18	CAN	(Cancel) ^x
		_got_char, 		//	25	19	EM	(End of Medium)
		_got_char, 		//	26	1A	SUB	(Substitute)
		_got_KILL, 		//	27	1B	ESC	(Escape)
		_got_char, 		//	28	1C	FS	(File Separator)
		_got_char, 		//	29	1D	GS	(Group Separator)
		_got_char, 		//	30	1E	RS  (Reqst to Send)(Record Sep.)	
		_got_char, 		//	31	1F	US	(Unit Separator)
		_got_char, 		//	32	20	SP	(Space)
		_got_char, 		//	33	21	!	(exclamation mark)
		_got_char, 		//	34	22	,	(double quote)	
		_got_char, 		//	35	23	#	(number sign)
		_got_char, 		//	36	24	$	(dollar sign)
		_got_char, 		//	37	25	%	(percent)
		_got_char, 		//	38	26	&	(ampersand)
		_got_char, 		//	39	27	'	(single quote)
		_got_char, 		//	40	28	(	(left/open parenthesis)
		_got_char, 		//	41	29	)	(right/closing parenth.)
		_got_char, 		//	42	2A	*	(asterisk)
		_got_char, 		//	43	2B	+	(plus)
		_got_char, 		//	44	2C		(comma)
		_got_char,	 	//	45	2D	-	(minus or dash)
		_got_char, 		//	46	2E	.	(dot)
		_got_char,	 	//	47	2F	/	(forward slash)
		_got_char, 		//	48	30	0	
		_got_char, 		//	49	31	1	
		_got_char, 		//	50	32	2	
		_got_char, 		//	51	33	3	
		_got_char, 		//	52	34	4	
		_got_char, 		//	53	35	5	
		_got_char, 		//	54	36	6	
		_got_char, 		//	55	37	7	
		_got_char, 		//	56	38	8	
		_got_char, 		//	57	39	9	
		_got_char, 		//	58	3A	:	(colon)
		_got_SEMICOLON, //	59	3B	;	(semi-colon)
		_got_char, 		//	60	3C	<	(less than)
		_got_char, 		//	61	3D	=	(equal sign)
		_got_char, 		//	62	3E	>	(greater than)
		_got_char, 		//	63	3F	?	(question mark)
		_got_char, 		//	64	40	@	(AT symbol)
		_got_char,		//	65	41	A	
		_got_char,		//	66	42	B	
		_got_char,		//	67	43	C	
		_got_char,		//	68	44	D	
		_got_char,		//	69	45	E	
		_got_char,		//	70	46	F	
		_got_char,		//	71	47	G	
		_got_char,		//	72	48	H	
		_got_char,		//	73	49	I	
		_got_char,		//	74	4A	J	
		_got_char,		//	75	4B	K	
		_got_char,		//	76	4C	L	
		_got_char,		//	77	4D	M	
		_got_char,		//	78	4E	N	
		_got_char,		//	79	4F	O	
		_got_char,		//	80	50	P	
		_got_char,		//	81	51	Q	
		_got_char,		//	82	52	R	
		_got_char,		//	83	53	S	
		_got_char,		//	84	54	T	
		_got_char,		//	85	55	U	
		_got_char,		//	86	56	V	
		_got_char,		//	87	57	W	
		_got_char,		//	88	58	X	
		_got_char,		//	89	59	Y	
		_got_char,		//	90	5A	Z	
		_got_char,		//	91	5B	[	(left/opening bracket)
		_got_char,		//	92	5C	\	(back slash)
		_got_char,		//	93	5D	]	(right/closing bracket)
		_got_char,		//	94	5E	^	(caret/circumflex)
		_got_char,		//	95	5F	_	(underscore)
		_got_char,		//	96	60	`	
		_got_char,		//	97	61	a	
		_got_char,		//	98	62	b	
		_got_char,		//	99	63	c	
		_got_char,		//	100	64	d	
		_got_char,		//	101	65	e	
		_got_char,		//	102	66	f	
		_got_char,		//	103	67	g	
		_got_char,		//	104	68	h	
		_got_char,		//	105	69	i	
		_got_char,		//	106	6A	j	
		_got_char,		//	107	6B	k	
		_got_char,		//	108	6C	l	
		_got_char,		//	109	6D	m	
		_got_char,		//	110	6E	n	
		_got_char,		//	111	6F	o	
		_got_char,		//	112	70	p	
		_got_char,		//	113	71	q	
		_got_char,		//	114	72	r	
		_got_char,		//	115	73	s	
		_got_char,		//	116	74	t	
		_got_char,		//	117	75	u	
		_got_char,		//	118	76	v	
		_got_char,		//	119	77	w	
		_got_char,		//	120	78	x	
		_got_char,		//	121	79	y	
		_got_char,		//	122	7A	z	
		_got_char,		//	123	7B	{	(left/opening brace)
		_got_char,		//	124	7C	|	(vertical bar)
		_got_char,		//	125	7D	}	(right/closing brace)
		_got_char,		//	126	7E	~	(tilde)
		_got_DELETE		//	127	7F	DEL	(delete)
};

/* 
 * xio_usb_getc helper routines
 */

 #define OUTPUT(c) if (ECHO(f.flags)) { xio_usb_putc(c, stdout); } return(c);

static int _got_char(void)
{
	OUTPUT(c)
	if (ECHO(f.flags)) {
		xio_usb_putc(c, stdout);
	}
	return(c);
}

static int _got_KILL(void)
{
	f.signals = XIO_SIG_KILL;
	return(_FDEV_ERR);
}

static int _got_PAUSE(void)
{
	f.signals = XIO_SIG_PAUSE;
	return(_FDEV_ERR);
}

static int _got_RESUME(void)
{
	f.signals = XIO_SIG_RESUME;
	return(_FDEV_ERR);
}

static int _got_NUL(void)			// convert NULs to newlines if line mode
{
	if (LINEMODE(f.flags)) {
		c = '\n';
	} 
	if (ECHO(f.flags)) {
		xio_usb_putc(c, stdout);
	}
	return(c);
}


static int _got_NEWLINE(void)		// convert CRs and LFs to newlines if line mode
{
	if (LINEMODE(f.flags)) {
		c = '\n';
	} 
	if (ECHO(f.flags)) {
		xio_usb_putc(c, stdout);
	}
	return(c);
}

static int _got_SEMICOLON(void)		// convert ;'s newlines if line mode & semi mode
{
	if ((LINEMODE(f.flags)) && (SEMICOLONS(f.flags))) {
		c = '\n';
	} 
	if (ECHO(f.flags)) {
		xio_usb_putc(c, stdout);
	}
	return(c);
}

static int _got_DELETE(void)
{
	f.signals = XIO_SIG_DELETE;
	return(_FDEV_ERR);
}

static int _got_SHIFTOUT(void)
{
	f.signals = XIO_SIG_SHIFTOUT;
	return(_FDEV_ERR);
}

static int _got_SHIFTIN(void)
{
	f.signals = XIO_SIG_SHIFTIN;
	return(_FDEV_ERR);
}

/*
 *  xio_usb_getc_unbuffered() - unbuffered char reader for USB device
 *
 *  Get next character from RX buffer.
 *	See "Notes on the circular buffers" at end of xio.h for buffer details.
 *
 *	This routine returns a single character from the RX buffer to the caller.
 *	It's typically called by fgets() and is useful for single-thresed IO cases.
 *	Cases with multiple concurrent IO streams may want to use the buffered version.
 *
 *  Flags that affect behavior:
 *
 *  BLOCKING behaviors
 *	 	- execute blocking or non-blocking read depending on controls
 *		- return character or -1 & XIO_SIG_WOULDBLOCK if non-blocking
 *		- return character or sleep() if blocking
 *
 *  ECHO behaviors
 *		- if ECHO is enabled echo character to stdout
 *		- echo all line termination chars as newlines ('\n')
 *		- Note: putc should expand newlines to <cr><lf>
 *
 *  SPECIAL CHARACTERS 
 *		- special characters such as EOL and control chars are handled by the
 *		  character helper routines. See them for behaviors
 */

int xio_usb_getc_unbuffered(FILE *stream)
{
	while (f.rx_buf_head == f.rx_buf_tail) {	// RX ISR buffer empty
		if (BLOCKING(f.flags)) {
			sleep_mode();
		} else {
			f.signals = XIO_SIG_WOULDBLOCK;
			return(_FDEV_ERR);
		}
	}
	if (--(f.rx_buf_tail) == 0) {				// advance RX tail (RXQ read pointer)
		f.rx_buf_tail = RX_BUFFER_SIZE-1;		// -1 avoids off-by-one error (OBOE)
	}
	c = (f.rx_buf[f.rx_buf_tail] & 0x007F);		// get char from RX buffer & mask MSB
	// 	call action procedure from dispatch table in FLASH (see xio.h for typedef)
	return (((fptr_int)(pgm_read_word(&getcFuncs[c])))());

	//ALTERNATE - call action procedure from character dispatch table in RAM
	//return (getcFuncs[c]());
}

/*
 *  xio_usb_getc_buffered() - unbuffered char reader for USB device
 *
 *  Get next character from RX buffer. 
 *	See "Notes on the circular buffers" at end of xio.h for buffer details.
 *
 *  Flags that affect behavior:
 *
 *  BLOCKING behaviors
 *	 	- execute blocking or non-blocking read depending on controls
 *		- return character or -1 if non-blocking
 *		- return character or sleep() if blocking
 *
 *  LINEMODE and SEMICOLONS behaviors
 *		- consider <cr> and <lf> to be EOL chars (not just <lf>)
 *		- also consider semicolons (';') to be EOL chars if SEMICOLONS enabled
 *		- convert any EOL char to <lf> to signal end-of-string (e.g. to fgets())
 *
 *  ECHO behaviors
 *		- if ECHO is enabled echo character to stdout
 *		- echo all line termination chars as newlines ('\n')
 *		- Note: putc should expand newlines to <cr><lf>
 *
 * Also knows how to trap control characters
 */

int xio_usb_getc_buffered(FILE *stream)
{
	// figure out heads and tails
	while (f.rx_buf_head == f.rx_buf_tail) {	// buffer empty
		if (BLOCKING(f.flags)) {
			sleep_mode();
		} else {
			f.signals = XIO_SIG_WOULDBLOCK;
			return(_FDEV_ERR);
		}
	}
	if (--(f.rx_buf_tail) == 0) {				// decrement and wrap if needed
		f.rx_buf_tail = RX_BUFFER_SIZE-1;		// -1 avoids off-by-one error (OBOE)
	}
	c = (f.rx_buf[f.rx_buf_tail] & 0x007F);
	// 	call action procedure from dispatch table in FLASH (see xio.h for typedef)
	return (((fptr_int)(pgm_read_word(&getcFuncs[c])))());

	//ALTERNATE - call action procedure from character dispatch table in RAM
	//return (getcFuncs[c]());
}

/* 
 *	xio_usb_rx_task() - main loop task for USB device
 *
 *	Non-blocking, run-to-completion task for handling incoming data from USB port
 *  Should be called each time a character is received by the RX ISR
 *  Performs the following functions:
 *		- read character from RX buffer
 *		- strip signals and dispatch them to signal handler
 *		- collect complete line and pass to line handler function
 */

void xio_usb_rx_task() 
{
	
}
