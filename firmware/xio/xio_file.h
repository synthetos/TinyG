/*
 * xio_file.h	- device driver for file-type devices
 *   			- works with avr-gcc stdio library
 *
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

---- How to setup and use program memory "files" ----

  Setup a memory file (OK, it's really just a string)
  should be declared as so:

	const char PROGMEM g0_test1[] = "\
	g0 x10 y20 z30\n\
	g0 x0 y21 z-34.2";

	Line 1 is the initial declaration of the array (string) in program memory
	Line 2 is a continuation line. 
		- Must end with a newline and a continuation backslash
		- (alternately can use a semicolon instead of \n is XIO_SEMICOLONS is set)
		- Each line will be read as a single line of text using fgets()
	Line 3 is the terminating line. Note the closing quote and semicolon.


  Initialize: xio_pgm_init() must be called first. See the routine for options.

  Open the file: xio_pgm_open() is called, like so:
	xio_pgm_open(PGMFILE(&g0_test1));	// simple linear motion test

	PGMFILE does the right cast. If someone more familiar with all this 
	can explain why PSTR doesn't work I'd be grateful.

  Read a line of text. Example from parsers.c
	if (fgets(textbuf, BUF_LEN, srcin) == NULL) {	
		printf_P(PSTR("\r\nEnd of file encountered\r\n"));
		clearerr(srcin);
		srcin = stdin;
		tg_prompt();
		return;
	}
*/

#ifndef xio_file_h
#define xio_file_h

/* 
 * FILE DEVICE CONFIGS 
 */

#define PGM_INIT_bm (XIO_RD | XIO_BLOCK | XIO_ECHO | XIO_CRLF | XIO_LINEMODE)
#define EEP_INIT_bm (XIO_RDWR | XIO_BLOCK | XIO_LINEMODE)
#define TBL_INIT_bm (XIO_RDWR | XIO_BLOCK | XIO_LINEMODE)
#define RAM_INIT_bm (XIO_RDWR | XIO_BLOCK | XIO_LINEMODE)

//#define EEP_ADDR_BASE (0x1000)		// needed to support memory mapped mode
#define EEP_ADDR_BASE (0x0000)		// needed to support memory mapped mode
#define TBL_ADDR_BASE (0x1000)
#define RAM_ADDR_BASE (0x1000)

#define PGM_ADDR_MAX (0x4000)		// 16K
#define EEP_ADDR_MAX (0x1000)		// 4K
#define TBL_ADDR_MAX (0x2000)		// 8K
#define RAM_ADDR_MAX (0x4000)		// 8K

/* 
 * FILE device extended control structure 
 * Note: As defined this struct won't do files larger than 65,535 chars
 * Note: As defined this struct won't do files larger than 4 Gbytes chars
 */

// file-type device control struct
struct xioFILE {
	uint16_t fflags;					// file sub-system flags
	uint32_t rd_offset;					// read index into file
	uint32_t wr_offset;					// write index into file
	uint32_t max_offset;				// max size of file
	char * filebase_P;					// base location in memory
};

/* 
 * FILE DEVICE FUNCTION PROTOTYPES
 */

// PGM functions
void xio_init_pgm(void);
FILE * xio_open_pgm(const char * addr);		// open memory string read only
int xio_cntl_pgm(const uint32_t control);			// validate & set dev flags
int xio_putc_pgm(const char c, struct __file *stream);// always returns ERROR
int xio_getc_pgm(struct __file *stream);			// get a character
int xio_gets_pgm(char *buf, const int size);	// read string from program memory

// EEPROM functions
/*
void xio_init_eep(void);
FILE * xio_open_eep(const prog_char *addr);			// open EEPROM string
int xio_cntl_eep(const uint32_t control);			// validate & set dev flags
int xio_putc_eep(const char c, struct __file *stream);// unoptimized EEPROM write
int xio_getc_eep(struct __file *stream);			// get a character from EEPROM
int xio_gets_eep(char *buf, const int size);	// read string from EEPROM
int xio_seek_eep(uint32_t offset);
int xio_rewind_eep();
//int xio_puts_eep(const char *buf, struct __file *stream);
*/

// RAM Card functions

// SD Card functions


#endif
