/*
  xmega_io.h - "file" and serial functions for xmega family
  Modeled after unix fileio 

  Copyright (c) 2010 Alden S. Hart, Jr.

  Annoying avr20100110 bug: 
  	Browse to this dir for Libs: C:\WinAVR-20100110\avr\lib\avrxmega6

*/

#ifndef xmega_serial_h
#define xmega_serial_h

/*
 * Major IO subsystem configs, constants, and device structures 
 */

#define RX_BUFSIZE 128				// general buffer sizes
#define TX_BUFSIZE 1				// tx buffer is not used
#define SSIZE_MAX RX_BUFSIZE		// maximum bytes for read or write (progmem)

/* USART IO structure
   Note: as defined this struct won't do buffers larger than 255 chars 
*/

struct fdUSART {					// file descriptor struct for serial IO
	uint_fast8_t fd;				// the assigned FD number
	uint_fast8_t baud;				// baud rate index
	uint_fast8_t flags;				// control flags

	uint_fast8_t rx_buf_max;		// RX buffer maximum count
	uint_fast8_t rx_buf_tail;		// RX buffer read index (location from which to read)
	volatile uint8_t rx_buf_head;	// RX buffer write index (changes via ISR)

	uint_fast8_t tx_buf_max;		// TX buffer maximum count
	volatile uint8_t tx_buf_tail;	// TX buffer read index (changes via ISR)
	uint_fast8_t tx_buf_head;		// TX buffer write index

	volatile char rx_buf[RX_BUFSIZE];
	volatile char tx_buf[TX_BUFSIZE];

	struct USART_struct *usart;		// USART structure
	struct PORT_struct *port;		// corresponding port

//	int (*io_read)();		// somehow I'd like to get this to work
//	int (*io_write)();
//	int8_t (*io_close)();
//	int8_t (*io_control)();

};

/* 
 * Function prototypes 
 */

void io_init(void);
int8_t io_open(uint8_t dev, uint32_t control);
int8_t io_close(uint8_t fd);
int8_t io_control(uint8_t fd, uint32_t control);
int16_t io_read(uint8_t fd, char **buf, int count);
int16_t io_write(uint8_t fd, char *buf, int count);


/*************************************
 *
 * IO Subsystem General Assignments
 *
 *************************************/

#define FD_USB 1					// file descriptor for USB port
#define FD_RS485 2					// file descriptor for RS485 port
#define FD_MAX 3					// size of FD pointer array

#define COUNT_MODE 0				// read and write modes
#define LINE_MODE -1
#define NULL_MODE -2

/* Devices recognized by IO system functions */

// Native Xmega devices
#define DEV_NULL	0				// NULL device

#define	DEV_SRAM	1				// string in static RAM
#define DEV_EEPROM	2				// string in EEPROM
#define DEV_PROGMEM 3				// string in main app program memory (FLASH)
#define DEV_TABLEMEM 4				// string in app table program memory (FLASH)
#define DEV_BOOTMEM 5				// string in boot program memory (FLASH)

#define DEV_PORTA	6				// Define ports as IO devices
#define DEV_PORTB	7
#define DEV_PORTC	8
#define DEV_PORTD	9
#define DEV_PORTE	10
#define DEV_PORTF	11
#define DEV_PORTG	12				// not implemented on xmega A3s
#define DEV_PORTH	13				// not implemented on xmega A3s
#define DEV_PORTJ	14				// not implemented on xmega A3s
#define DEV_PORTK	15				// not implemented on xmega A3s
#define DEV_PORTL	16				// not implemented on xmega A3s
#define DEV_PORTM	17				// not implemented on xmega A3s
#define DEV_PORTN	18				// not implemented on xmega A3s
#define DEV_PORTP	19				// not implemented on xmega A3s
#define DEV_PORTQ	20				// not implemented on xmega A3s

#define DEV_PORTR	21				// special purpose port - programming bits only 

#define DEV_USARTC0	22				// USARTS C0 - F1
#define DEV_USARTC1	23
#define DEV_USARTD0	24
#define DEV_USARTD1	25
#define DEV_USARTE0	26
#define DEV_USARTE1	27
#define DEV_USARTF0	28
#define DEV_USARTF1	29

#define DEV_SPIC	30				// SPI interfaces C - F
#define DEV_SPID	31
#define DEV_SPIE	32
#define DEV_SPIF	33

#define DEV_TWIC	34				// Two Wire interfaces C and E
#define DEV_TWIE	35

#define DEV_IRCOM	36				// IR communications module

// Synthetic devices
#define DEV_CONSOLE		37			// mapped to USB, here for convenience
#define DEV_USB			38			// USB comm and controls packaged
#define DEV_RS485		39			// RS485 comm and controls packaged
#define DEV_ENCODERS	40			// Encoder comm and controls packaged
#define DEV_BRIDGE		41			// USB to RS485 bridge

/* Serial Configuration Settings
   Values for common baud rates at 32 Mhz clock
   Enum Baud		BSEL	BSCALE 
	0  	unspec'd	0		0			// use default value
	1	9600		207		0
	2	19200		103		0
	3	38400		51		0
	4	57600		34		0
	5	115200		33 		(-1<<4)
	6	230400		31		(-2<<4)
	7	460800		27		(-3<<4)
	8	921600		19		(-4<<4)
	9	500000		1 		(1<<4)		// 500K
	10	1000000		1		0			// 1 mbps
*/

#define	IO_BAUD_UNSPECIFIED 0
#define IO_BAUD_9600 1
#define IO_BAUD_19200 2
#define IO_BAUD_38400 3
#define IO_BAUD_57600 4
#define IO_BAUD_115200 5
#define IO_BAUD_230400 6
#define IO_BAUD_460800 7
#define IO_BAUD_921600 8
#define IO_BAUD_500000 9
#define IO_BAUD_1000000 10
#define	IO_BAUD_DEFAULT IO_BAUD_115200

/* io_open() io_control() parameters and fs.flags */

#define IO_BAUD_gm		0x0000000F	// baud rate enumeration mask (keep in LSbyte)

#define IO_RDONLY		(1<<8) 		// read enable bit
#define IO_WRONLY		(1<<9)		// write enable only
#define IO_RDWR			(0) 		// read & write

#define IO_ECHO			(1<<10)		// echo reads from device to console (line level)
#define IO_NOECHO		(1<<11)		// disable echo

#define IO_RDBLOCK		(1<<12)		// enable blocking reads
#define IO_WRBLOCK		(1<<13)		// enable blocking writes (not implemented)
#define IO_RDWRBLOCK 	(IO_RDBLOCK | IO_WRBLOCK)  // enable blocking on RD/WR
#define IO_RDNONBLOCK	(1<<14)		// disable blocking reads
#define IO_WRNONBLOCK	(1<<15)		// disable blocking writes (not implemented)
#define IO_RDWRNONBLOCK (IO_RDNONBLOCK | IO_WRNONBLOCK)

#define IO_FLAG_RD_bm		(1<<0)	// read flag in fs.flags
#define IO_FLAG_WR_bm		(1<<1)	// write flag
#define IO_FLAG_RD_BLOCK_bm	(1<<2)	// enable blocking read
#define IO_FLAG_WR_BLOCK_bm	(1<<3)	// enable blocking write
#define IO_FLAG_ECHO_CHAR_bm (1<<4)	// echo read bytes to console at character level
#define IO_FLAG_ECHO_LINE_bm (1<<5)	// echo read bytes to console at line level

#define IO_FLAG_DEFAULT_gm (IO_FLAG_RD_bm | IO_FLAG_WR_bm | IO_FLAG_RD_BLOCK_bm | IO_FLAG_ECHO_LINE_bm)

#define IF_READ(a) (a & IO_FLAG_RD_bm)			// TRUE if read enabled
#define IF_WRITE(a) (a & IO_FLAG_WR_bm)			// TRUE if write enabled
#define IF_BLOCKING(a) (a & IO_FLAG_RD_BLOCK_bm)// TRUE if read blocking enabled
#define IF_ECHO(a) (a & IO_FLAG_ECHO_LINE_bm)	// TRUE if line echo mode enabled



/********************************
 *
 * Device Specific Assignments
 *
 ********************************/

/*
 * USB port assignments
 */

#define USB_USART USARTC0			// USARTC0 is wired to USB chip on the board
#define USB_RX_ISR_vect USARTC0_RXC_vect	// RX ISR
#define USB_TX_ISR_vect USARTC0_TXC_vect	// TX ISR

#define USB_PORT PORTC				// port where the USART is located
#define USB_RX_bm (1<<2)			// RX pin	- these pins are wired on the board
#define USB_TX_bm (1<<3)			// TX pin
#define USB_RTS_bm (1<<1)			// RTS pin
#define USB_CTS_bm (1<<0)			// CTS pin

#define USB_BAUD_RATE 115200		// these 3 must be consistent
#define USB_BSEL 33					// 115200 BAUD
#define USB_BSCALE (-1<<4) 

/*
 * RS485 port assignments
 */

#define RS485_USART USARTC1			// USARTC1 is wired to RS485 circuitry
#define RS485_RX_ISR_vect USARTC1_RXC_vect	// RX ISR
#define RS485_TX_ISR_vect USARTC1_TXC_vect	// TX ISR

#define RS485_PORT PORTC			// port where the USART is located
#define RS485_RX_bm (1<<6)			// RX pin	- these pins are wired on the board
#define RS485_TX_bm (1<<7)			// TX pin
#define RS485_DE_bm (1<<5)			// Data Enable pin (active HI)
#define RS485_RE_bm (1<<4)			//~Recv Enable pin (active LO)

#define RS485_BAUD_RATE 115200	// these 3 must be consistent
#define RS485_BSEL 33
#define RS485USB_BSCALE (-1<<4)


#endif
