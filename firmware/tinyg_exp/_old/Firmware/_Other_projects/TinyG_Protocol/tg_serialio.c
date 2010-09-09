/*
	tg_serialio.c
	TinyG Generic Serial Handler (both RS-485 and USB)
	Written by: Alden Hart
	Revision: 03/20/10
	References: Boston Android code used in this module
*/

#include <stdio.h>
#include <avr\io.h>
#define F_CPU 32000000UL
#include <util\delay.h>

#include <tg_serialio.h>

/* serial IO initialization */
void initSerialio(void)
{
  config32MHzClock();
  CLK.PSCTRL = 0x00; 							// no division on peripheral clock
  configUsartC1();								// configure USART C1 as RS-485 port

  /* UNCOMMENT TO TEST CLOCK SPEED ON PORTE.7
  PORTCFG.CLKEVOUT = PORTCFG_CLKOUT_PE7_gc;
  PORTE.DIR = (1<<7); 							// clkout on PORTE bit7
  */ 
}

/* Read character from USART (xmega) */
uint8_t usartReadChar(void) 
{
  PORTB.OUTCLR = 0x07;
   
  if (USARTC1.STATUS & USART_PERR_bm)
  {
 	PORTB.OUTSET = 0x01;
  };
  if (USARTC1.STATUS & USART_BUFOVF_bm)
  {
	PORTB.OUTSET = 0x02;
  };
  if (USARTC1.STATUS & USART_FERR_bm)
  {
	PORTB.OUTSET = 0x04;
  };
  if (USARTC1.STATUS & USART_RXCIF_bm)
  {
	PORTB.OUTSET = 0x08;
  };
//  while(!(USARTC1.STATUS & USART_RXCIF_bm));  	// wait for RX complete
  return USARTC1.DATA;

/*
  while (!(USARTC1.STATUS & USART_RXCIF_bm))
  {
  	if (USARTC1.STATUS & USART_FERR_bm) 
	{
	  return (-1);
	};
  	if (USARTC1.STATUS & USART_BUFOVF_bm) 
	{
      data = USARTC1.DATA;
	  return (-2);
    };
  }; 
  return USARTC1.DATA;
*/
};

/* Write character to USART (xmega) */
void usartWriteChar(uint8_t data) 
{
  while(!(USARTC1.STATUS & USART_DREIF_bm)); 	// spin until TX data register is available
  PORTC.OUTSET = enableDE_bm;          			// enable DE
  PORTC.OUTSET = enableRE_bm;          			// disable DE

  USARTC1.DATA = data;							// write data register

  while(!(USARTC1.STATUS & USART_TXCIF_bm)); 	// wait for TX complete
  USARTC1.STATUS |= USART_TXCIF_bm;  			// clear TX interrupt flag
  _delay_us(100);								// wait ~1 character time TX to complete
  PORTC.OUTCLR = enableDE_bm;          			// disable DE
  PORTC.OUTCLR = enableRE_bm;          			// enable DE

};

/* Write string to USART
   ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=49806&start=0  see tomasch
 */
void usartWriteString(const char *txstring) 
{ 
 uint8_t txbyte; 
 while ((txbyte=*txstring++)) { 
   usartWriteChar(txbyte); 
 } 
}

/* Configure USART C1 (xmega)
   Configure PORTC, USARTD1 (PORTC:7=Tx, PORTF:6=Rx) as asynch serial port
   This will connect to the RS-485 port
 */
void configUsartC1(void) 
{
  PORTC.DIRSET = (1<<7); 						// set PORTC:7 transmit pin as output
  PORTC.DIRCLR = (1<<6); 						// clr PORTC:6 receive pin as input

  PORTC.DIRSET = enableDE_bm;					// set PORTC:5 for DE line as output
  PORTC.OUTCLR = enableDE_bm;        			// clr PORTC:5 (disabled)

  PORTC.DIRSET = enableRE_bm;					// set PORTC:4 for ~RE line as output
  PORTC.OUTCLR = enableRE_bm;          			// clr PORTC:4 (enabled) 

//  USARTC1.BAUDCTRLA = 207; 					// 9600b  (BSEL=207,BSCALE=0)
//  USARTC1.BAUDCTRLA = 103; 					// 19200b  (BSEL=103,BSCALE=0)
  USARTC1.BAUDCTRLA = 34;  						// 57600b  (BSEL=34,BSCALE=0)
//  USARTC1.BAUDCTRLA = 33; USARTC1.BAUDCTRLB = (-1<<4); 	// 115.2kb (BSEL=33,BSCALE=-1)
//  USARTC1.BAUDCTRLA = 31; USARTC1.BAUDCTRLB = (-2<<4);	// 230.4kb (BSEL=31,BSCALE=-2)
//  USARTC1.BAUDCTRLA = 27; USARTC1.BAUDCTRLB = (-3<<4); 	// 460.8kb (BSEL=27,BSCALE=-3)
//  USARTC1.BAUDCTRLA = 19; USARTC1.BAUDCTRLB = (-4<<4);	// 921.6kb (BSEL=19,BSCALE=-4)
//  USARTC1.BAUDCTRLA = 1;  USARTC1.BAUDCTRLB = (1<<4); 	// 500kb (BSEL=19,BSCALE=-4)
//  USARTC1.BAUDCTRLA = 1;   								// 1Mb (BSCALE=1,BSCALE=0)

  USARTC1.CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
};


/* Configure 32 MHz clock (xmega) */
void config32MHzClock(void) 
{
  CCP = CCP_IOREG_gc; 							// Security Signature to modify clk 

  // initialize clock source to be 32MHz internal oscillator (no PLL)
  OSC.CTRL = OSC_RC32MEN_bm; 					// enable internal 32MHz oscillator
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); 		// wait for oscillator ready
  CCP = CCP_IOREG_gc; 							// Security Signature to modify clk
  CLK.CTRL = 0x01; 								// select sysclock 32MHz osc
};
