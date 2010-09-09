/*
	Protocol_001
	Protocol handler - first attempt
	Written by: Alden Hart
	Revision: 03/21/10
	References: "Writing Efficient State Machines in C" 
				 http://johnsantic.com/comp/state.html
*/

#include <stdio.h>
#include <avr\io.h>
#define F_CPU 32000000UL
#include <util\delay.h>

// #include <protocol.h>

void Config32MHzClock(void);
void ConfigUsartC1(void);
void UsartWriteChar(unsigned char data);
unsigned char UsartReadChar(void);

#define st_uStep1 (1<<4);		// stepper controller bits
#define st_uStep0 (1<<3);
#define st_enable (1<<2);
#define st_dir  (1<<1);
#define st_step (1<<0);

#define enableDE_bm (1<<5);		// DE line - active HI
#define enableRE_bm (1<<4);		// RE line - active LO
#define charTime (100);			// character time in microseconds

/* define states and events */

enum states {listen, startPacket, rxFrom, MAX_STATES} current_state;
enum events {gotDigit, gotAlpha, MAX_EVENTS} new_event;

/*
enum states {listen, startPacket, rxFrom, endFrom, rxTo, endTo, rxNext, endNext,
			 rxPacketType, rxTypeValue, endPacketType, rxPayload, finPacket, 
			 execPacket, ackAppPacket, MAX_STATES} current_state;

enum events {gotDigit, gotAlpha, gotBang, gotEqual, gotEOS, gotWS, gotUnsup,
			 gotTO, gotMAX, gotOK, gotERR, MAX_EVENTS} new_event;
*/


/* state action function prototypes */

void acNoop (void);				// no operation. fill spaces in dispatch array
void acErrorExit (void);		// packet reception error exit ("F" on charts)
void acLoopTossChar (void);		// loop in current state, toss input character
void acLoopSaveChar (void);		// loop in current state, save input character to string buffer
void acRestartPacket (void);	// go to beginning of new packet - no error thrown

void actionFinRxFromNode (void);	// final


enum events get_new_event (void);
void ac_listen_Digit (void);
void ac_listen_Alpha (void);
void ac_startPacket_Digit (void);
void ac_startPacket_Alpha (void);
void ac_rxFrom_Digit (void);
void ac_rxFrom_Alpha (void);


/* utility routine function prototypes */
//int testValidNodeID (ubyte);


/* state/event lookup table */

void (*const state_table [MAX_STATES][MAX_EVENTS]) (void) = {
	{ac_listen_Digit, ac_listen_Alpha},					// procedures for listening
	{ac_startPacket_Digit, ac_startPacket_Alpha},		// procedures for startPacket
	{ac_rxFrom_Digit, ac_rxFrom_Alpha}					// procedures for rxFromNode
};

int main(void)
{
//  int data;						//. RX/TX byte

  Config32MHzClock();
  CLK.PSCTRL = 0x00; 			// no division on peripheral clock
  PORTCFG.CLKEVOUT = PORTCFG_CLKOUT_PE7_gc;
  PORTE.DIR = (1<<7); 			// clkout on PORTE bit7
 
  ConfigUsartC1();				// configure USART C1 as RS-485 port
  PORTB.DIR = 0x0F;				// set PORTB:0 output for LED

  PORTA.DIR = 0b00011111; 							// set PORTA data direction - low bits are outputs
  PORTA.OUT	= 0b00000000;


  while (1) 
  {
	new_event = get_new_event();						// get next event to process

	if (((new_event >= 0) && (new_event < MAX_EVENTS))
		&& ((current_state >= 0) && (current_state < MAX_STATES)))
		{
		  state_table [current_state][new_event] ();	// call the action procedure
		} else {
			/* invalid state or event */
		}
  }
}

enum events get_new_event (void)
{
//  return gotDigit;
  return gotAlpha;
};

void ac_listen_Digit (void)
{
	current_state = listen;						// set next state
};

void ac_listen_Alpha (void)
{
	current_state = startPacket;				// set next state
};

void ac_startPacket_Digit (void)
{
	current_state = startPacket;				// set next state
};

void ac_startPacket_Alpha (void)
{
	current_state = rxFrom;						// set next state
};

void ac_rxFrom_Digit (void)
{
	current_state = rxFrom;						// set next state
};

void ac_rxFrom_Alpha (void)
{
	current_state = listen;						// set next state
}

/*
  while(1)
  {
	PORTA.OUT = st_step;
	_delay_us(1);
	PORTA.OUT = 0;
	_delay_us(1000);
*/
/*
	data=UsartReadChar(); 		// read char
	UsartWriteChar(data); 		// write char
	PORTB.OUT = 0x0F; 			// toggle LED
*/


void UsartWriteChar(unsigned char data)
{
	while(!(USARTC1.STATUS & USART_DREIF_bm)); 		// spin until TX data register is available
	PORTC.OUTSET = enableDE_bm;          			// enable DE
	USARTC1.DATA = data;							// write data register

	while(!(USARTC1.STATUS & USART_TXCIF_bm)); 		// wait for TX complete
	USARTC1.STATUS |= USART_TXCIF_bm;  				// clear TX interrupt flag
	_delay_us(100);									// wait ~1 character time TX to complete
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

/*
void (*const state_table [MAX_STATE][MAX_EVENT]) (void) = {
	{acState0, acState1 			//gotDigit
	 acNoop,			//gotAlpha
	 acNoop,			//gotBang
	 acNoop,			//gotEqual
	 acNoop,			//gotEOS
	 acNoop,			//gotWS
	 acNoop,			//gotUnsup
	 acNoop,			//gotTO
	 acNoop,			//gotMAX
	 acNoop,			//gotOK
	 acNoop,			//gotERR
	},
	// startPacket
	{},
	// rxFromNode
	{},
	// endFromNode
	{},
	// rxToNode
	{},
	// endToNode
	{},
	// rxNextTalker
	{},
	// endNextTalker
	{},
	// rxPacketType
	{},
	// rxTypeValue
	{},
	// endPacketType
	{},
	// rxPayload
	{},
	// finishPacket
	{},
	// executePacket
	{},
	// ackAppPacket
	{}
};
*/
