/*
	Xproto_echo
	test program to echo characters through serial port
	Most of this code was lifted from Boston Android: xmega-serial-echo.c
	Created by: Alden Hart
	Revision: 03/06/10
*/

#include <stdio.h>
#include <avr\io.h>
#define F_CPU 32000000UL
#include <util\delay.h>

void Config32MHzClock(void);
void UsartWriteChar(unsigned char data);
unsigned char UsartReadChar(void);

int main(void)
{
  int data;
  Config32MHzClock();

  CLK.PSCTRL = 0x00; // no division on peripheral clock

  PORTCFG.CLKEVOUT = PORTCFG_CLKOUT_PE7_gc;
  PORTE.DIR = (1<<7); // clkout

  // configure PORTF, USARTF0 (PORTF:3=Tx, PORTF:2=Rx) as asynch serial port
  // This will connect to the USB-Serial chip on EVAL-USB boards
  // For other boards rewrite all occurences of USARTF0 below with USARTE0
  // then you can use PORTE:2,3 as asynch serial port (EVAL-01, EVAL-04 boards)
  PORTF.DIR |= (1<<3) | (1<<0); // set PORTF:3 transmit pin as output
  PORTF.OUT |= (1<<3);          // set PORTF:3 hi 
//  USARTF0.BAUDCTRLA = 207; // 9600b  (BSCALE=207,BSEL=0)
//  USARTF0.BAUDCTRLA = 103; // 19200b  (BSCALE=103,BSEL=0)
//  USARTF0.BAUDCTRLA = 34;  // 57600b  (BSCALE=34,BSEL=0)
//  USARTF0.BAUDCTRLA = 33; USARTF0.BAUDCTRLB = (-1<<4); // 115.2kb (BSCALE=33,BSEL=-1)
//  USARTF0.BAUDCTRLA = 31; USARTF0.BAUDCTRLB = (-2<<4); // 230.4kb (BSCALE=31,BSEL=-2)
//  USARTF0.BAUDCTRLA = 27; USARTF0.BAUDCTRLB = (-3<<4); // 460.8kb (BSCALE=27,BSEL=-3)
  USARTF0.BAUDCTRLA = 19; USARTF0.BAUDCTRLB = (-4<<4); // 921.6kb (BSCALE=19,BSEL=-4)
//  USARTF0.BAUDCTRLA = 1; USARTF0.BAUDCTRLB = (1<<4); // 500kb (BSCALE=19,BSEL=-4)
//  USARTF0.BAUDCTRLA = 1;   // 1Mb (BSCALE=1,BSEL=0)

  USARTF0.CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART

  while(1)
  {
    data=UsartReadChar(); // read char
	UsartWriteChar(data); // write char
	_delay_ms(100);
	PORTF.OUT ^= (1<<0); // toggle LED

  };
};

void UsartWriteChar(unsigned char data)
{
    USARTF0.DATA = data; // transmit ascii 3 over and over
	if(!(USARTF0.STATUS&USART_DREIF_bm))
		while(!(USARTF0.STATUS & USART_TXCIF_bm)); // wait for TX complete
  	USARTF0.STATUS |= USART_TXCIF_bm;  // clear TX interrupt flag
};

unsigned char UsartReadChar(void)
{
	unsigned char ret;
	while(!(USARTF0.STATUS&USART_RXCIF_bm));  // wait for RX complete

  	return USARTF0.DATA;
};

void Config32MHzClock(void)
{
  CCP = CCP_IOREG_gc; 						//Security Signature to modify clock 

  // initialize clock source to be 32MHz internal oscillator (no PLL)
  OSC.CTRL = OSC_RC32MEN_bm; 				// enable internal 32MHz oscillator
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); 	// wait for oscillator ready
  CCP = CCP_IOREG_gc; 						//Security Signature to modify clock 
  CLK.CTRL = 0x01; 							//select sysclock 32MHz osc
};
