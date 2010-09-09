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


void initMain(void);		// prototype for main init routine


/************************************************************************************

	main

************************************************************************************/

int main(void)
{
  uint8_t data;					//.RX/TX byte

  initMain();					// general initializations

  while (1) 					// main loop
  {
	_delay_ms(10);
	data = '3';

//	data=usartReadChar(); 		// read char
//	PORTB.OUTTGL = 0x08; 		// toggle LED
//	prReceiveChar(data);		// run protocol parser
	usartWriteChar(data); 		// write char

/*
	prReceiveChar('!');
	prReceiveChar('1');
	prReceiveChar(' ');
	prReceiveChar('2');
	prReceiveChar('3');
	prReceiveChar(' ');
	prReceiveChar('d');
	prReceiveChar('=');
	prReceiveChar('a');
	prReceiveChar(' ');
	prReceiveChar('(');
	prReceiveChar('a');
	prReceiveChar(')');
	prReceiveChar(';');
*/
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
  PORTA.DIR = 0b00001111; 		// set PORTA data direction - low bits are outputs
  PORTA.OUT	= 0b00000000;

  initSerialio();
  initProtocol();
  initMotors();
};


/* Problem log
 
  Things I can't seem to fix

  -	Moving prStateTable and prCharArray into protocol.h file
  - Running prStateTable from program memory

http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=77845&start=0
   Menu=(menu_t *)pgm_read_word(&SysMenuPtrs[(MenuState.LRValue-1)]); 


   // Get menu handler address from table in flash and call it 
   MenuHandler=(MenuHandler_t)pgm_read_word(&Menu->MenuHandler); 
   MenuHandler(Flags);

http://survey.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=85390&view=next
   page_handler pages[] PROGMEM = { ... 
// Some time later ... 
	page_handler hndlr; 
	hndlr = (page_handler) pgm_read_word(&pages[_status.page]); 




http://winavr.scienceprog.com/example-avr-projects/avr-lcd-menu-routine.html

It is ease to use function pointer, that can be changed during menu item change. 
To make things easier to manage I have created Array of function pointers in Flash

const FuncPtr FuncPtrTable[] PROGMEM=
{ func101, func102, func103,	//functions for submenus of menu 1
func201, func202, //functions for submenus of menu 2
func301, func302, //functions for submenus of menu 3
func401, func402  //functions for submenus of menu 4
//further functions...
};

Using this function array I can easily point to required function with single code line:

FPtr=(FuncPtr)pgm_read_word(&FuncPtrTable[MFIndex(MN.menuNo, MN.subMenuNo)]);

Where MFIndex(MN.menuNo, MN.subMenuNo) function returns array index according to menu and submenu number.

*/



