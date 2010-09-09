/*
	TinyG_parser.c
	Test program to parser incoming RS-485 characters into packets and echo characters back
	Written by: Alden Hart
	Revision: 03/15/10
*/

#include <stdio.h>
#include <avr\io.h>
#define F_CPU 32000000UL
#include <util\delay.h>

void Config32MHzClock(void);
void ConfigUsartC1(void);
void UsartWriteChar(unsigned char data);
unsigned char UsartReadChar(void);

#define enableDE_bm (1<<5);		// DE line - active HI
#define enableRE_bm (1<<4);		// RE line - active LO
#define charTime (100);			// character time in microseconds


int main(void)
{
  int data;						//. RX/TX byte

  Config32MHzClock();
  CLK.PSCTRL = 0x00; 			// no division on peripheral clock
  PORTCFG.CLKEVOUT = PORTCFG_CLKOUT_PE7_gc;
  PORTE.DIR = (1<<7); 			// clkout on PORTE bit7
 
  ConfigUsartC1();				// configure USART C1 as RS-485 port
  PORTB.DIR |= (1<<0);			// set PORTB:0 output for LED

  while(1)
  {
	data=UsartReadChar(); 		// read char
	UsartWriteChar(data); 		// write char
	PORTB.OUTTGL = (1<<0); 		// toggle LED
  };
};


void UsartWriteChar(unsigned char data)
{
	while(!(USARTC1.STATUS & USART_DREIF_bm)); 		// spin until TX data register is available
	PORTC.OUTSET = enableDE_bm;          			// enable DE
	USARTC1.DATA = data;							// write data register

	while(!(USARTC1.STATUS & USART_TXCIF_bm)); 		// wait for TX complete
	USARTC1.STATUS |= USART_TXCIF_bm;  				// clear TX interrupt flag
	_delay_ms(0.1);						// wait ~1 character time TX to complete
	PORTC.OUTCLR = enableDE_bm;          			// disable DE
};

unsigned char UsartReadChar(void)
{
//	unsigned char ret;
	while(!(USARTC1.STATUS & USART_RXCIF_bm));  	// wait for RX complete
  	return USARTC1.DATA;
};


void ConfigUsartC1(void)
{
// configure PORTC, USARTD1 (PORTC:7=Tx, PORTF:6=Rx) as asynch serial port
// This will connect to the RS-485 port
  PORTC.DIR |= (1<<7); 										// set PORTC:7 transmit pin as output

  PORTC.DIR |= enableDE_bm;									// set portC:5 for DE line as output
  PORTC.OUT &= enableDE_bm;        							// set PORTC:5 lo (disabled)

  PORTC.DIR |= enableRE_bm;									// set portC:4 for ~RE line as output
  PORTC.OUT &= enableRE_bm;          						// set PORTC:4 lo (enabled) 

//  USARTC1.BAUDCTRLA = 207; 									// 9600b  (BSCALE=207,BSEL=0)
//  USARTC1.BAUDCTRLA = 103; 									// 19200b  (BSCALE=103,BSEL=0)
  USARTC1.BAUDCTRLA = 34;  									// 57600b  (BSCALE=34,BSEL=0)
//  USARTC1.BAUDCTRLA = 33; USARTC1.BAUDCTRLB = (-1<<4); 		// 115.2kb (BSCALE=33,BSEL=-1)
//  USARTC1.BAUDCTRLA = 31; USARTC1.BAUDCTRLB = (-2<<4);		// 230.4kb (BSCALE=31,BSEL=-2)
//  USARTC1.BAUDCTRLA = 27; USARTC1.BAUDCTRLB = (-3<<4); 		// 460.8kb (BSCALE=27,BSEL=-3)
//  USARTC1.BAUDCTRLA = 19; USARTC1.BAUDCTRLB = (-4<<4);	 	// 921.6kb (BSCALE=19,BSEL=-4)
//  USARTC1.BAUDCTRLA = 1; USARTC1.BAUDCTRLB = (1<<4); 			// 500kb (BSCALE=19,BSEL=-4)
//  USARTC1.BAUDCTRLA = 1;   									// 1Mb (BSCALE=1,BSEL=0)

  USARTC1.CTRLB = USART_TXEN_bm | USART_RXEN_bm; 			// enable tx and rx on USART
};


void Config32MHzClock(void)
{
  CCP = CCP_IOREG_gc; 								// Security Signature to modify clock 

  // initialize clock source to be 32MHz internal oscillator (no PLL)
  OSC.CTRL = OSC_RC32MEN_bm; 						// enable internal 32MHz oscillator
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); 			// wait for oscillator ready
  CCP = CCP_IOREG_gc; 								// Security Signature to modify clock 
  CLK.CTRL = 0x01; 									// select sysclock 32MHz osc
};


