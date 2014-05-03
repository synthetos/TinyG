#ifndef SIM_SIM_H_ONCE
#define SIM_SIM_H_ONCE

#define NUL (char)0x00  //  ASCII NUL char (0) (not "NULL" which is a pointer)

typedef uint16_t flags_t;
typedef uint8_t buffer_t;

int xio_gets(const uint8_t dev, char *buf, const int size);
FILE *xio_open(const uint8_t dev, const char *addr, const flags_t flags);




#define DEL (char)0x7F

#define RX_BUFFER_SIZE (buffer_t)255                 // buffer_t can be 8 bits
#define TX_BUFFER_SIZE (buffer_t)255                 // buffer_t can be 8 bits
#define XOFF_TX_LO_WATER_MARK (TX_BUFFER_SIZE * 0.05)        // % to issue XON

#define XIO_BLOCK		((uint16_t)1<<0)		// enable blocking reads
#define XIO_NOBLOCK		((uint16_t)1<<1)		// disable blocking reads
#define XIO_XOFF 		((uint16_t)1<<2)		// enable XON/OFF flow control
#define XIO_NOXOFF 		((uint16_t)1<<3)		// disable XON/XOFF flow control
#define XIO_ECHO		((uint16_t)1<<4)		// echo reads from device to stdio
#define XIO_NOECHO		((uint16_t)1<<5)		// disable echo
#define XIO_CRLF		((uint16_t)1<<6)		// convert <LF> to <CR><LF> on writes
#define XIO_NOCRLF		((uint16_t)1<<7)		// do not convert <LF> to <CR><LF> on writes
#define XIO_IGNORECR	((uint16_t)1<<8)		// ignore <CR> on reads
#define XIO_NOIGNORECR	((uint16_t)1<<9)		// don't ignore <CR> on reads
#define XIO_IGNORELF	((uint16_t)1<<10)		// ignore <LF> on reads
#define XIO_NOIGNORELF	((uint16_t)1<<11)		// don't ignore <LF> on reads
#define XIO_LINEMODE	((uint16_t)1<<12)		// special <CR><LF> read handling
#define XIO_NOLINEMODE	((uint16_t)1<<13)		// no special <CR><LF> read handling

#define PGM_FLAGS (XIO_BLOCK | XIO_CRLF | XIO_LINEMODE)


uint8_t xio_test_assertions(void);
void xio_init(void);
void cli(void);
int xio_ctrl(const uint8_t dev, const flags_t flags);
int xio_set_baud(const uint8_t dev, const uint8_t baud_rate);

#endif  // SIM_SIM_H_ONCE
