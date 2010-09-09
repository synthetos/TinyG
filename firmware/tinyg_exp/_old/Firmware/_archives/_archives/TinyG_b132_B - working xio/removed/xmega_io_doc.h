/*
  xmega_io.c - IO functions for xmega family
  Modeled after UNIX io: open(), close(), read(), write(), ioctl()

  Copyright (c) 2010 Alden S. Hart, Jr.

  IO subsystem features
  	- Looks and works like Unix IO .
	- Syntax, semantics and operation of UNIX IO largely copied
		- xio_open() returns integer (unint8_t) file descriptors
		- xio_read() and write() obey fd, buffer and size conventions (in SIZE_MODE)
	- Macro aliases are defined to expose routines using UNIX names: open(), close()
		as opposed to the module names (xio_open(), xio_close()...)
	- Framework to organize IO drivers for the 41 (by my count) native Xmega devices
	- Extensible to support synthetic devices such as USB ports, RS-485, etc.
	- Can be used to provide the putc / getc needed by AVR-GCC stdio

  Notable differences from UNIX IO
  	- It's Kabuki Theater: everything is pre-allocated (no malloc calls)
	- read() and write() extended to handle lines and strings (in addition to SIZE)
		- LINE_MODE: read/write to defined line delimiter (e.g. \r, \n, ';')
		- STR_MODE:  read/write to end-of-string (aka: zero, ASCII nul, 0, \0)
		- PSTR_MODE: read program memory strings to end-of-string
	- xio_control() is NOT ioctl(). There are enough differences to notice


---- avr-gcc stdio notes ----

  What about non-blocking reads?

  Does udata need to point to the char level control structure (e.g. read ISR buffering)

  List of commands you shouldn;t use for resource reasons:
  	fdevopen()						// malloc
	fdev_setup_stream()				// malloc
	fclose()						// malloc (actually free())
	fprintf() / vfprintf()			// code size issues


  Note: stdio does not perform any bufering

  fgets() looks like readln() - except it only handles NUL and <CR>
  --> need to write a fgets2() (or some other extention) to also look for semicolons

  Does it set errno? or just use the __SERR flag in __file.flags ?


---- Why not just use avr-gcc stdio? ----

  You can. Just link the device putc()/getc() to the streams (fd_setup_stream())
  (this will require integrating the __file struct into these routines)

  Specialized functions for reading to termination chars 
  (e.g. Gcode blocks terminated by <CR>, Arduino commands terminated by ';')

  Abiliy to take over the standard block read and write for specialized functions
  like reading or writing time streams to DACs or other devices

  Very fast dispatch to support high speed serial operations

  Ability to (more) easily create hybrid devices like bridges & networking

  What you lose is all the nice character print stuff.


---- Notes on "The Split" ----

  variables: see xmega_io.h structure
	- top level
	- block level
	- char level

  --- dispatch layer 

  xio_open()
  	- top level needed
	- block level needed
	- char level needed

  xio_close()
  	- all

  xio_control()
	- block level needed (read / write size max)
	- char level needed (device controls)

  xio_read() / xio_readln() / xio_readstr() / xio_readstr_P()  (generics)
	- 

  --- block layer

  xio_open(dev)
  	- setup top layer (dispatch and generic block read/write routines)
	- setup character device
		- needs the ability to override the generic block-level reads and writes

  xio_open_DEVICE()

  --- Device specific 



---- Read/Write Modes ----

  There are four modes for read and write:

	SIZE_MODE	Reads or writes exactly SIZE characters before returning. 
				NULs are not special - i.e. nul chars in strings are passed through
				In non-blocking mode it is possible that the read or write may 
				  complete less than SIZE characters and return with -1, EAGAIN.
				This emulates standard UNIX IO.

	LINE_MODE	Reads until a delimiter is read from the device (ex. \n, \r, ;)
				1st delimiter is written to the rcv string (ex. \r of a \r\n pair)
				The receive buffer string is nul terminated after the first delimiter
				A read that exceeds rx_size_max is an EMSGSIZE error (returns -1)
				The buffer will be full up to that point and terminated at the max.

				Writes until a delimiter is found in the source string
				The first delimiter is written to the device, 
				Terminating nul is not written to the device.
				A write that exceeds tx_size_max is an EMSGSIZE error (returns -1).
				  but will have written all bytes up to that point to the device.

	STR_MODE	Reads until a nul is read from the device.
				The nul is written to the receiving string
				A read that exceeds rx_size_max is an EMSGSIZE error (returns -1)
				The buffer will be full up to that point and terminated at the max.

				Writes until a nul is found in the source string.
				Terminating nul is not written to the device.
				A write that exceeds tx_size_max is an EMSGSIZE error (returns -1).
				  but will have written all bytes up to that point to the device.

	PSTR_MODE	(This mode is not valid for read)
	
				Writes characters from a program memory string to the device
				Writes until a nul is found in the source string (PSTR)
				Terminating nul is not written to the device.
				A write that exceeds tx_size_max is an EMSGSIZE error (returns -1).
				  but will have written all bytes up to that point to the device.

				Typically used to embed PGM string literals in a "print" statement.
				Reading from a PGM memory file is different, and is accomplished by 
				  opening a DEV_PROGMEM device and reading from program memory.

  (Not all devices implement all modes.)

---- Aliases ----

  The following alaises are provided (see xmega_io.h)

	SIZE_MODE	read(f,b,s)		Specify file descriptor, receive buffer and size	
				write(f,b,s)	Specify file descriptor, source buffer and size

	LINE_MODE	readln(f,b)		Specify file descriptor and receive buffer
				writeln(f,b)	Specify file descriptor and source buffer

	STR_MODE	readstr(f,b)	Specify file descriptor and receive buffer
				writestr(f,b)	Specify file descriptor and source buffer

	PSTR_MODE	writepstr(f,b)	Specify file descriptor and source buffer

  Character level functions are also provided:

				char getc(f)	read single character from device
				void putc(f,c)	write single character to device

---- Notes on the circular buffers ----

  An attempt has beeen made to make the circular buffers used by low-level 
  character read/write (getc/putc) as efficient as possible. This opens up 
  higher-speed IO between 100K and 1Mbaud and better supports high-speed 
  parallel operations.

  The circular buffers are unsigned char arrays that count down from the top 
  element and wrap back to the top when index zero is reached. This allows 
  pre-decrement operations, zero tests, and eliminates modulus, mask, substraction 
  and other less efficient array bounds checking. Buffer indexes are all 
  unint_fast8_t which limits these buffers to 254 usable locations. (one location
  is lost to head/tail collision detection and one is lost to the zero position).
  All this enables the compiler to do better optimization.

  Chars are written to the *head* and read from the *tail*. 

  The head is left "pointing to" the character that was previously written - 
  meaning that on write the head is pre-decremented (and wrapped, if necessary), 
  then the new character is written.

  The tail is left "pointing to" the character that was previouly read - 
  meaning that on read the tail is pre-decremented (and wrapped, if necessary),
  then the new character is read.

  The head is only allowed to equal the tail if there are no characters to read.

  On read: If the head = the tail there is nothing to read, so it exits or blocks.

  On write: If the head pre-increment causes the head to equal the tail the buffer
  is full. The head is reset to its previous value and the device should go into 
  flow control (and the byte in the device is not read). Reading a character from 
  a buffer that is in flow control should clear flow control

  (Note: More sophisticated flow control would detect the full condition earlier, 
   say at a high water mark of 95% full, and may go out of flow control at some low
   water mark like 33% full).

---- Coding Conventions ----

  Adopted the following xmega and C variable naming conventions
  (See AVR1000: Getting Started Writing C-code for XMEGA [doc8075.pdf] )

	varname_bm		- single bit mask, e.g. 0x40 aka (1<<4)
	varname_bp		- single bit position, e.g. 4 for the above example
	varname_gm		- group bit mask, e.g. 0x0F
	varname_gc		- group configuration, e.g. 0x0A is 2 bits in the above _gm
	varname_ptr		- indicates a pointer. (but NOT array indexes)
	varname_idx		- indicates an array index (if not simply called i or j)
	varname_vect	- interrupt or other vectors

  These conventions are used for internal variables but may be relaxed for old 
  UNIX vars and DEFINES that don't follow these conventions.

---- Other Stuff ----

  In this code:
  	"NULL" refers to a null (uninitialized) pointer 
   	"NUL" refers to the ASCII string termination character - zero
			See http://home.netcom.com/~tjensen/ptr/  (chapter 3)

---- Adding A New Device ----

  There are native devices - supported directly by the xmegas (like a USART), 
  and derived devices that build on top of one ore more native devices (e.g. USB)
  Derived devices effectively sub-class a native device. Let's discuss native 
  devices first, then how to sub-class a derived device.

  Adding a native device:

	- Create xmega_io_newdevice.c and xmega_io_newdevice.h files 
	  (start with USART.c as an example)
	  	- change the title header to the new name

	- in the xmega_io_newdevice.h file
		- change the module header accordingly		
		- change the #ifndef / #define to xmega_io_newdevice_h
		- add fucntion prototypes for 
			- xio_open_newdevice()
			- xio_close_newdevice()
			- xio_control_newdevice()
			- xio_read_newdevice()
			- xio_write_newdevice()
			- xio_getc_newdevice()
			- xio_putc_newdevice()

	- In the xmega_io_newdevice.c file
		- change the module header accordingly
		- include or comment out <avr/interrupt.h>
		- include your own .h file below xmega_io.h
		- provide a routine for each of
			- xio_open_newdevice()
			- xio_close_newdevice()
			- xio_control_newdevice()
			- xio_read_newdevice()
			- xio_write_newdevice()
			- xio_getc_newdevice()
			- xio_putc_newdevice()

	- in the xmega_io.h file
		- add an fd_ptr for the new device, e.g: 
			#define FD_NDV 10 (or whatever)

		- and increase FD_MAX accordingly

	- in the xmega_io.c file
		- add your header to the includes:
			#include "xmega_io_newdevice.h"

		- add your fd struct to the preallocation list, e.g: 
			struct fdUSART fd_usb, fd_rs485, fs_ndv;

		- edit the fdes[] table with your new FD in the right spot in the array
		  if necessary extend the array and change the DEV_ table, but you shouldn't
		  need to do this for a native device unless it's been inadvertantly left out

		- in xio_init(), extend the pointer table initialization, e.g.
			fd_ptrs[FD_NDV] = &fd_ndv;	

		- include the new device in the dispatch for all the xio_routines(), e.g.
			case DEV_NDV: return xio_open_ndv(dev, control);
			... and similarly for close(), control(), read(), write(), getc(), putc() 



---- To Do ----

	- Flow control for USB low-level read and write
	- Change the _routines (that were helper routines) to xio_ routines
	- Change the FS pointer table to works with void *s so you can do proper polymorphism
	- Add some real flow control to the USARTs
	- Add xio_putc() xio_getc() to every level and cross wire the USB and USARTS accordingly
		Include them in the ailases and in the function popinter bindings.

*/

