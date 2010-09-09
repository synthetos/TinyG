/*
	tg_main.c
	TinyG Main File
	Written by: Alden Hart
	Revision: 03/20/10
*/

#include <stdio.h>
#include <avr\io.h>
//#include <avr\pgmspace.h>
//#include <avr\interrupt.h>

#ifndef F_CPU
#define F_CPU 32000000UL
#endif
#include <util\delay.h>

#include <tg_protocol.h>
#include <tg_serialio.h>
#include <tg_motors.h>


/* utility routine function prototypes */
//int testValidNodeID (ubyte);


void initMain(void);


/************************************************************************************

	main

************************************************************************************/


int main(void)
{
//uint8_t data;					//. RX/TX byte

//  initMain();					// general initializations

  while (1) 					// main loop
  {
//	data=usartReadChar(); 		// read char
//	prRunProtocol(data);		// run protocol parser
//	usartWriteChar(data); 		// write char
//	PORTB.OUT = 0x0F; 			// toggle LED

//	data = '!';
//	prRunProtocol(data);
	prRunProtocol();
	
  }
}


/************************************************************************************

	subroutines

************************************************************************************/


//  Main init routine. 
//	Does some stuff on its own and calls the module inits

void initMain()
{
  PORTB.DIR = 0x0F;				// set PORTB:0 output for LED
  PORTA.DIR = 0b00011111; 		// set PORTA data direction - low bits are outputs
  PORTA.OUT	= 0b00000000;

  initSerialio();
  initProtocol();
  initMotors();
};



