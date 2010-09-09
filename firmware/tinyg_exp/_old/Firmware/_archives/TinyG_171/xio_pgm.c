/*
 *  xio_pgm.h - device driver for program memory "files"
 * 			- works with avr-gcc stdio library
 *
 * Copyright (c) 2010 Alden S. Hart, Jr.
 */

#include <stdio.h>
#include <avr/pgmspace.h>

#include "xio.h"
#include "xio_pgm.h"


// define a (statically allocated) stdio FILE struct
FILE dev_pgm = FDEV_SETUP_STREAM(NULL, xio_pgm_getc, _FDEV_SETUP_RW);

// define and allocate local (static) control struct for the device
struct xioPGM {
	uint16_t flags;						// control flags
	uint8_t sig;						// signal or error value
	uint8_t c;							// line buffer character temp
	uint8_t i;							// line buffer pointer
	uint8_t len;						// line buffer maximum length (zero based)
	int (*sig_func)(uint8_t sig);		// pointer to signal handler function
	int (*line_func)(char * buf);		// pointer to line handler function
	char buf[CHAR_BUFFER_SIZE];			// line buffer for line reader

	char * pgmbase_P;			// base location in memory
	uint16_t idx;				// index into file
};
static struct xioPGM fpgm;		// create local low-level "file" control struct

/* 
 *	xio_pgm_init() - initialize and set controls for program memory device 
 *
 *	Control		   Arg	  Default		Notes
 *	
 *	XIO_RD		  <null>	Y	Enable device for reads
 *	XIO_WR		  <null>  (err)	Enable device for write
 *	XIO_BLOCK	  <null>	Y	Enable blocking reads
 *	XIO_NOBLOCK   <null>  (err)	Disable blocking reads
 *	XIO_ECHO	  <null>		Enable echo
 *	XIO_NOECHO	  <null>	Y	Disable echo
 *	XIO_CRLF	  <null>		Send <cr><lf> if <lf> detected
 *	XIO_NOCRLF	  <null>	Y	Do not convert <lf> to <cr><lf>
 *	XIO_LINEMODE  <null>		Apply special <cr><lf> read handling
 *	XIO_NOLINEMODE <null>	Y	Do not apply special <cr><lf> read handling
 *	XIO_SEMICOLONS <null>		Treat semicolons as line breaks
 *	XIO_NOSEMICOLONS <null>	Y	Don't treat semicolons as line breaks
 *
 *  Control parameters are defaulted and may be set using xio_pgm_control()
 */

void xio_pgm_init(uint16_t control)
{
	fpgm.flags = XIO_FLAG_PGM_DEFS_gm;	// set flags to defaults & initial state
	if (control & XIO_WR) {				// this is actually an error. Ignore for now.
		fpgm.flags |= XIO_FLAG_WR_bm;
	}
	if (control & XIO_NOBLOCK) {		// this is also technically a config error.
		fpgm.flags &= ~XIO_FLAG_BLOCK_bm;
	}
	if (control & XIO_ECHO) {
		fpgm.flags |= XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_NOECHO) {
		fpgm.flags &= ~XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_CRLF) {
		fpgm.flags |= XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_NOCRLF) {
		fpgm.flags &= ~XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_LINEMODE) {
		fpgm.flags |= XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_NOLINEMODE) {
		fpgm.flags &= ~XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_SEMICOLONS) {
		fpgm.flags |= XIO_FLAG_SEMICOLONS_bm;
	}
	if (control & XIO_NOSEMICOLONS) {
		fpgm.flags &= ~XIO_FLAG_SEMICOLONS_bm;
	}

	fpgm.idx = 0;
	fpgm.sig = 0;
	dev_pgm.udata = &(fpgm.sig); 	// bind signals register to pgm FILE struct
	fpgm.len = sizeof(fpgm.buf)-1;				// offset to zero
	fpgm.sig_func = &null_signal;				// bind null signal handler
	fpgm.line_func = &null_line;				// bind null line handler
}

/*	
 *	xio_usb_open() - provide a string address to the program memory device
 *
 *	OK, so this is not really a UNIX open() except for it's moral equivalency
 *  Returns a pointer to the stdio FILE struct or -1 on error
 */

FILE * xio_pgm_open(const prog_char *addr)
{
	fpgm.flags &= XIO_FLAG_RESET_gm;			// reset the signaling bits
	fpgm.pgmbase_P = (PROGMEM char *)addr;		// might want to range check this
	fpgm.idx = 0;
	return(&dev_pgm);
}

/*	
 *	xio_usb_control() - set controls for program memory device 
 *
 *	Control		   Arg	  Default		Notes
 *	
 *	XIO_RD		  <null>	Y	Enable device for reads
 *	XIO_ECHO	  <null>	Y	Enable echo
 *	XIO_NOECHO	  <null>		Disable echo
 *	XIO_LINEMODE  <null>		Apply special <cr><lf> read handling
 *	XIO_NOLINEMODE <null>	Y	Do not apply special <cr><lf> read handling
 *	XIO_SEMICOLONS <null>		Treat semicolons as line breaks
 *	XIO_NOSEMICOLONS <null>	Y	Don't treat semicolons as line breaks
 */

int8_t xio_pgm_control(uint16_t control, int16_t arg)	// arg is not yet used
{
	// transfer control flags to internal flag bits
	fpgm.flags = XIO_FLAG_PGM_DEFS_gm;		// set flags to defaults & initial state
	if (control & XIO_ECHO) {
		fpgm.flags |= XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_NOECHO) {
		fpgm.flags &= ~XIO_FLAG_ECHO_bm;
	}
	if (control & XIO_CRLF) {
		fpgm.flags |= XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_NOCRLF) {
		fpgm.flags &= ~XIO_FLAG_CRLF_bm;
	}
	if (control & XIO_LINEMODE) {
		fpgm.flags |= XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_NOLINEMODE) {
		fpgm.flags &= ~XIO_FLAG_LINEMODE_bm;
	}
	if (control & XIO_SEMICOLONS) {
		fpgm.flags |= XIO_FLAG_SEMICOLONS_bm;
	}
	if (control & XIO_NOSEMICOLONS) {
		fpgm.flags &= ~XIO_FLAG_SEMICOLONS_bm;
	}
	return (0);
}

/* 
 *	xio_pgm_putc() - write character to to program memory device
 *
 *  Always returns error. You cannot write to program memory
 */

int xio_pgm_putc(char c, FILE *stream)
{
	return -1;			// always returns an error. Big surprise.
}


/*
 *  xio_pgm_getc() - read a character from program memory device
 *
 *  Get next character from program memory file.
 *
 *  END OF FILE (EOF)
 *		- the first time you encounter NUL, return ETX
 *		- all subsequent times rreturn NUL
 *	    This allows the higherlevel stdio routines to return a line that 
 *	    terminates with a NUL, but reads from the end of file will return errors.
 *	  	
 *		- return ETX (as returning NUL is indistinguishable from an error)
 *		- return NUL (this is NOT EOF, wich is -1 and signifies and error)
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
 */

int xio_pgm_getc(FILE *stream)
{
	if (fpgm.flags & XIO_FLAG_EOF_bm) {
		fpgm.sig = XIO_SIG_EOF;
		return (_FDEV_EOF);
	}
	if ((fpgm.c = pgm_read_byte(&fpgm.pgmbase_P[fpgm.idx])) == NUL) {
		fpgm.flags |= XIO_FLAG_EOF_bm;
	}
	++fpgm.idx;
	if (!LINEMODE(fpgm.flags)) {		// processing is simple if not LINEMODE
		if (ECHO(fpgm.flags)) {
			putchar(fpgm.c);
		}
		return (fpgm.c);
	}
	// now do the LINEMODE stuff
	if (fpgm.c == NUL) {						// perform newline substitutions
		fpgm.c = '\n';
	} else if (fpgm.c == '\r') {
		fpgm.c = '\n';
	} else if ((SEMICOLONS(fpgm.flags)) && (fpgm.c == ';')) {
		fpgm.c = '\n';
	}
	if (ECHO(fpgm.flags)) {
		putchar(fpgm.c);
	}
	return (fpgm.c);
}

/* 
 *	xio_pgm_readln() - main loop task for program memory device
 *
 *	Non-blocking, run-to-completion return a line from memory
 *	Note: LINEMODE flag is ignored. It's ALWAYS LINEMODE here.
 */

int xio_pgm_readln()
{
	if (!(fpgm.pgmbase_P)) {					// return if no file open
		return (0);
	}
	if (fgets(fpgm.buf, fpgm.len, &dev_pgm) == NULL) {	
		printf_P(PSTR("\r\nEnd of file encountered\r\n"));
		clearerr(&dev_pgm);
		// cut input back to stdin device.
	}
	return (0);
}

