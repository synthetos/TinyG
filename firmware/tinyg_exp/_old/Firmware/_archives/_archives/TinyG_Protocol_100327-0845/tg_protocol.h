/*
	tg_protocol.h
	TinyG Protocol Handler include file
	Written by: Alden Hart
	Revision: 03/25/10
*/

#define IO_BUFFER_LEN 128

// main structure for managing packet IO

struct prPacketStruct {
	// packet controls
	char	inChar;				// current input
	uint8_t	state;				// current state
	uint8_t	event;				// current event
	uint8_t	ackCode;			// ACK or NAK code
	uint8_t bufferPtr;			// buffer pointer

	// packet data
	uint8_t	fromNode;			// from nodeID
	uint8_t toNode;				// to NodeID
	uint8_t	nextTalker;			// next talker
	uint8_t	packetType;
	uint8_t	typeValue;
	uint8_t	buffer[IO_BUFFER_LEN+1]; // buffer for building headers & payloads
};

struct prPacketStruct rx;		// receiving packet
struct prPacketStruct tx;		// transmitting packet


// if you change the order or count you must adjust the prStateTable in tg_protocol.c

enum prStates { listen, startPacket, rxFromNode, endFromNode, rxToNode, endToNode, 
				rxNextTalker, endNextTalker,rxPacketType, rxTypeValue, rxPayload, 
				MAX_STATES} prState;

enum prEvents { gotSOH, gotDigit, gotAlpha, gotEqual, gotEOS, gotJunk, gotWS, gotTO, 
				MAX_EVENTS} prEvent;


/* general function prototypes */

void initProtocol(void);			// initialization
void prRunProtocol(uint8_t inChar);	// protocol parser entry point


/* action function prototypes */

void prNop(void);					// used to fill spaces in state table
void prAnyEventListen(void);		// enter listen state from any event
void prAnyEventStart(void);			// start new packet from any event (no error throw)
void prAnyLoopDiscard(void);		// loop in current state, disard input character
void prAnyLoopSave(void);			// loop in current state, save input char to rxBuffer

void prThrowGenericError(void);		// throw generic error NAK
void prThrowTimeoutGeneric(void);	// throw generic inter-character timeout NAK

void prThrowTimeoutStartPacket(void);// throw timeout startPacket NAK 
void prThrowMalformedStartPacket(void);// throw malformed startPacket NAK

void prThrowTimeoutFromNode(void);	// throw timeout fromNode NAK 
void prThrowMalformedFromNode(void);// throw malformed fromNode NAK
void prThrowInvalidFromNode(void);	// throw invalid fromNode NAK

void prThrowTimeoutToNode(void);	// throw timeout toNode NAK 
void prThrowMalformedToNode(void);	// throw malformed toNode NAK
void prThrowInvalidToNode(void);	// throw invalid toNode NAK

void prThrowTimeoutNextTalker(void);// throw timeout nextTalker NAK 
void prThrowMalformedNextTalker(void);// throw malformed nextTalker NAK
void prThrowInvalidNextTalker(void);// throw invalid nextTalker NAK

void prThrowTimeoutPacketType(void);// throw timeout PacketType NAK 
void prThrowMalformedPacketType(void);// throw malformed PacketType NAK
void prThrowInvalidPacketType(void);// throw invalid PacketType NAK
void prThrowOverrunPacketType(void);// throw overrun PacketType NAK

void prThrowTimeoutTypeValue(void);// throw timeout TypeValue NAK 
void prThrowMalformedTypeValue(void);// throw malformed TypeValue NAK
void prThrowInvalidTypeValue(void);// throw invalid TypeValue NAK
void prThrowOverrunTypeValue(void);// throw overrun TypeValue NAK

void prThrowTimeoutPayload(void);// throw timeout Payload NAK 
void prThrowOverrunPayload(void);// throw overrun Payload NAK


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

