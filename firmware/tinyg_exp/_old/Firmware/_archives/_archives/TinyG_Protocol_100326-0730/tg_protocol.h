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
				rxPacketType, rxTypeValue, rxPayload, endPacket,
				MAX_STATES} prState;

enum prEvents { gotSOH, gotDigit, gotAlpha, gotEqual, gotEOS, gotWS, gotJunk,
				gotTO, gotMAX, gotOK, gotERR,
				MAX_EVENTS} prEvent;





/* general function prototypes */

void initProtocol(void);			// initialization
void prRunProtocol(uint8_t inChar);	// protocol parser entry point


/* action function prototypes */

void prNop(void);					// no operation - used to fill spaces in state table
void prAnyEventListen(void);		// enter listen state from any event
void prAnyEventStart(void);			// start new packet from any event (no error thrown)
void prAnyLoopDiscard(void);		// loop in current state, disard the input character
void prAnyLoopSave(void);			// loop in current state, save input char to rxBuffer
void prAnyCharTimeout(void);		// inter-character timeout

void prAnyUntrappedError(void)	;	// exit for error that cannot be NAK'd
void prAnyTrappedError(void);		// exit for error that can be NAK'd

void prStartPacketDigitExit(void);	// exit startPacket to rxFromNode
void prStartPacketAlphaExit(void);	// exit startPacket to rxPacketType
void prRxFromNodeSpaceExit(void);	// exit rxFromNode to listen
void prEndFromNodeDigitExit(void);	// exit endFromNode to rxToNode
void prRxToNodeSpaceExit(void);		// exit rxFromNode to endFromNode
void prEndToNodeDigitExit(void);	// exit from endFromNode to rxToNode
void prEndToNodeAlphaExit(void);	// exit from endFromNode to rxPacketType
void prRxNextTalkerSpaceExit(void);	// exit rxNextTalker to endNextTalker
void prEndNextTalkerAlphaExit(void);// exit endNextTalker to rxPacketType
void prRxPacketTypeEqualExit(void);	// exit rxPacketType to rxTypeValue
void prRxPacketTypeSpaceExit(void);	// exit rxPacketType to endPacketType
void prRxTypeValueSpaceExit(void);	// exit rxTypeValue to endPacketType
void prRxPayloadEndExit(void);		// exit rxPayload to endPacket
void prEndPacketOK(void);			// exit endPacket to with OK status
void prEndPacketERR(void);			// exit endPacket to with ERROR status
