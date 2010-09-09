/*
	tg_protocol.h
	TinyG Protocol Handler include file
	Written by: Alden Hart
	Revision: 03/25/10
*/


// main structure for managing packet IO

struct prPacketStruct {
	uint8_t	state;
	uint8_t	event;

	uint8_t toNode;
	uint8_t	fromNode;
	uint8_t	nextTalker;
	uint8_t	packetType;
	uint8_t	typeValue;

//	unit8_t	rxBuffer[128];		// receive buffer for building headers & payloads
} prPacket;


// if you change the order or count you must adjust the prStateTable in tg_protocol.c

enum prStates { listen, startPacket, 
				rxFromNode, endFromNode, 
				rxToNode, endToNode, 
				rxNextTalker, endNextTalker,
				rxPacketType, rxTypeValue, endPacketType, rxPayload, 
				finPacket, execPacket, ackAppPacket, 
				MAX_STATES} prState;

enum prEvents { gotSOH, gotDigit, gotAlpha, gotEqual, gotEOS, gotWS, gotJunk,
				gotTO, gotMAX, gotOK, gotERR,
				MAX_EVENTS} prEvent;





/* general function prototypes */

void initProtocol(void);			// initialization
void prRunProtocol(uint8_t inChar);	// protocol parser entry point


/* action function prototypes */

void prNop(void);				// no operation - used to fill spaces in state table
void prAnyUntrappedError(void);	// exit for error that cannot be NAK'd ("L" on chart)
void prAnyTrappedError(void);	// exit for error that can be NAK'd ("F" on chart)
void prAnyEventListen(void);	// enter listen state from any event
void prAnyEventStart(void);		// styart new packet from any event (no error thrown)
void prAnyLoopToss(void);		// loop in current state, toss input character
void prAnyLoopSave(void);		// loop in current state, save char to rxBuffer
void prAnyCharTimeout(void);	// inter-character timeout

void prStartPacketDigitExit(void);	// leave startPacket with a digit
void prStartPacketAlphaExit(void);	// leave startPacket with an alpha
void prRxFromNodeSpaceExit(void);	// error exit back to listen

