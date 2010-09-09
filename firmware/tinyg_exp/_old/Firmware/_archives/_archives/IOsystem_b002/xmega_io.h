/*
  xmega_serial.h - "file" and serial functions for xmega family
  Modeled after unix fileio 

  Copyright (c) 2010 Alden S. Hart, Jr.

*/

#ifndef xmega_serial_h
#define xmega_serial_h

/* IO subsystem configs and constants */

#define TX_BUFSIZE 128
#define RX_BUFSIZE 128				// must be binary multiple
#define RX_BUFMASK (RX_BUFSIZE - 1)
#define SSIZE_MAX 1024				// maximum bytes for read or write (progmem)


/* Function prototypes */

void io_init(void);
int8_t io_open(uint32_t dev, uint32_t control);
int8_t io_close(uint8_t fd);
int8_t io_control(uint8_t fd, uint32_t control);
int io_read(uint8_t fd, char *buf, int count);
int io_write(uint8_t fd, char *buf, int count);


/* Device structures */

// Note: USART control struct won't do a buffer larger than 255 chars */

struct fdUSART {					// file descriptor struct for serial IO
	uint8_t	fd;						// your assigned FD number
	uint32_t control;				// control bits (optimize this)
	unsigned char rx_buf[RX_BUFSIZE]; // RX buffer (obviously)
	uint8_t rx_buf_head;			// index into rxbuffer (not a pointer)
									// ...where to write the next incoming character
	uint8_t rx_buf_tail;			// index of the location from which to read.

//	int *read();		// somehow I'd like to get this to work
//	int *write();
//	int8_t *close();
//	int8_t *control();

	struct USART_struct *usart;		// USART structure
	struct PORT_struct *port;		// corresponding port
};

// static assignments

/* Serial Configuration Settings
   Values for common baud rates at 32 Mhz clock
   Enum Baud	BSEL	BSCALE 
	0 	<off>	0		0
	1	9600	207		0
	2	19200	103		0
	3	38400	
	4	57600	34		0
	5	115200	33 		(-1<<4)
	6	230400	31		(-2<<4)
	7	460800	27		(-3<<4)
	8	921600	19		(-4<<4)
	9	500000	1 		(1<<4)
	10	1000000	1		0			1 mbps
*/

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

#define READ_LINE -1				// read modes
#define READ_TO_NULL -2

#define FD_USB 1					// file descriptor for USB port
#define FD_RS485 2					// file descriptor for RS485 port

// USB port assignments

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


// RS485 port assignments

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


/* Devices recognized by IO system functions */

// Native devices
#define DEV_NULL	0				// NULL device

#define	DEV_SRAM	1				// string in static RAM
#define DEV_EEPROM	2				// string in EEPROM
#define DEV_PROGMEM 3				// string in program memory (FLASH)

#define DEV_PORTA	4				// Define ports as IO devices
#define DEV_PORTB	5
#define DEV_PORTC	6
#define DEV_PORTD	7
#define DEV_PORTE	8
#define DEV_PORTF	9
#define DEV_PORTG	10				// not implemented on xmega A3s
#define DEV_PORTH	11				// not implemented on xmega A3s
#define DEV_PORTJ	12				// not implemented on xmega A3s
#define DEV_PORTK	13				// not implemented on xmega A3s
#define DEV_PORTL	14				// not implemented on xmega A3s
#define DEV_PORTM	15				// not implemented on xmega A3s
#define DEV_PORTN	16				// not implemented on xmega A3s
#define DEV_PORTP	17				// not implemented on xmega A3s
#define DEV_PORTQ	18				// not implemented on xmega A3s

#define DEV_PORTR	19				// special purpose port - programming bits only 

#define DEV_USARTC0	20				// USARTS C0 - F1
#define DEV_USARTC1	21
#define DEV_USARTD0	22
#define DEV_USARTD1	23
#define DEV_USARTE0	24
#define DEV_USARTE1	25
#define DEV_USARTF0	26
#define DEV_USARTF1	27

#define DEV_SPIC	28				// SPI interfaces C - F
#define DEV_SPID	29
#define DEV_SPIE	30
#define DEV_SPIF	31

#define DEV_TWIC	32				// Two Wire interfaces C and E
#define DEV_TWIE	33

#define DEV_IRCOM	34				// IR communications module

// Synthetic devices
#define DEV_CONSOLE		35			// mapped to USB, here for convenience
#define DEV_USB			36			// USB comm and controls packaged
#define DEV_RS485		37			// RS485 comm and controls packaged
#define DEV_ENCODERS	38			// Encoder comm and controls packaged


/* io_open() and io_control() parameters  - fit into ulong */

// lsbyte
#define IO_BAUD_gm		0x0000000F	// baud rate enumeration group mask

// next lsbyte
#define IO_RDONLY		(1<<8) 		// read enable bit
#define IO_WRONLY		(1<<9)		// write enable only
#define IO_RDWR			(IO_RDONLY || IO_WRONLY) // read & write
#define IO_RDNONBLOCK	(1<<10)		// enable non-blocking reads
#define IO_WRNONBLOCK	(1<<11)		// enable non-blocking writes
#define IO_RDWRNONBLOCK (IO_RDNONBLOCK || IO_WRNONBLOCK)
#define IO_WRECHO		(1<<12)		// echo writes to console

#endif
