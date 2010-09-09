/*
	tg_protocol.h
	TinyG Protocol Handler include file
	Written by: Alden Hart
	Revision: 03/21/10
*/

/************************************************************************************

	states and events 

************************************************************************************/

// if you change the order or count you must adjust the prStateTable in tg_protocol.c

enum prStates { listen, startPacket, 
				rxFromNode, endFromNode, 
				rxToNode, endToNode, 
				rxNextTalker, endNextTalker,
				rxPacketType, rxTypeValue, endPacketType, rxPayload, 
				finPacket, execPacket, ackAppPacket, 
				MAX_STATES} prState;

enum prEvents { gotDigit, gotAlpha, gotSOH, gotEqual, gotEOS, gotWS, gotJunk,
				gotTO, gotMAX, gotOK, gotERR,
				MAX_EVENTS} prEvent;

uint8_t inChar;

/* general function prototypes */

void initProtocol(void);		// initialization

//void prRunProtocol(unsigned char inChar);	 		// protocol parser entry point
void prRunProtocol(void);	 		// protocol parser entry point

//enum prEvents prDecodeChar(unsigned char inChar); 	// char decoder event handler


/* action function prototypes */


void prNop(void);				// no operation - used to fill spaces in state table
void prUntrappedError(void);	// exit for error that cannot be NAK'd ("L" on chart)
void prTrappedError(void);		// exit for error that can be NAK'd ("F" on chart)
void prLoopTossChar(void);		// loop in current state, toss input character
void prLoopSaveChar(void);		// loop in current state, save char to string buffer
void prRestartPacket(void);		// go to beginning of new packet - no error thrown
void prCharTimeout(void);		// inter-character timeout

void prListenBang(void);		// leave listen with an exclamation point
void prStartPacketDigit(void);	// leave startPacket with a digit
void prStartPacketAlpha(void);	// leave startPacket with an alpha
void prStartPacketError(void);	// error exit back to listen



