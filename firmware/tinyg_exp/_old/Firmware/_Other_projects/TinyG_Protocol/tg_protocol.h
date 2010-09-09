/*
	tg_protocol.h
	TinyG Protocol Handler include file
	Written by: Alden Hart
	Revision: 03/27/10
*/

#define IO_BUFFER_LEN 128
#define MAX_NODE 255

// if you change the order or count you must adjust the prStateTable in tg_protocol.c

enum prStates { listen, startPacket, rxFromNode, endFromNode, rxToNode, endToNode, 
				rxNextTalker, endNextTalker,rxPacketType, rxTypeValue, rxPayload, 
				MAX_STATES};

enum prEvents { gotSOH, gotDigit, gotAlpha, gotEqual, gotEOS, gotJunk, gotWS, gotTO, 
				MAX_EVENTS};

enum prPacketTypes { 
				packInvalid,				// use 0 for invalid packet type
				packTalk, packData, packAck, packBack, 
				packQuery, packNodeid, packConfig, packPing,
				packStart, packHush, packResend, packEnd,
				MAX_PACKET_TYPE};

enum prErrors {	prErrOK,					// 0	All packet-level ACKs are OK
				prErrGenericError,			// 1	Generic error
				prErrGenericOverrun,		// 2	Generic "field is too long"
				prErrGenericUnderrun,		// 3	Generic "field is too short"
				prErrGenericTimeout,		// 4	Generic timeout
				prErrGenericInvalid,		// 5	Generic "invalid value"
				prErrGenericMalformed,		// 6	Generic "too garbled"
				prErrGenericMissing,		// 7	Generic "missing data"
				prErrGenericCharError,		// 8	Generic character recv error
				prErrCharFramingError,		// 9	Character reception framing err
				prErrCharParityError,		// 10	Character reception parity error
				prErrCharOverrun,			// 11	Character reception overrun err
				prErrCharUnderrun,			// 12	Character reception underrun err
				prErrCharCollision,			// 13	Collision detected at char level
				prErrPacketReceiveTimeout,	// 14	Too much time between characters
				prErrPacketChecksumError,	// 15	Checksum error on receive
				prErrMalformedStartPacket,	// 16	Start of packet is malformed
				prErrMalformedFromNode,		// 17	Illegal characters in fromNode
				prErrMalformedToNode,		// 18	Illegal characters in toNode
				prErrMalformedNextTalker,	// 19	Illegal characters in nextTalker
				prErrMalformedPacketType,	// 20	Illegal characters in packetType
				prErrMalformedTypeValue,	// 21	Illegal characters in typeValue
				prErrMalformedPayload,		// 22	Illegal characters in payload
				prErrInvalidFromNode,		// 23	Not in range or unassigned
				prErrInvalidToNode,			// 24	Not in range or unassigned
				prErrInvalidNextTalker,		// 25	Not in range or unassigned
				prErrInvalidPacketType,		// 26	Unsupported packetType
				prErrInvalidTypeValue,		// 27	Unsupported typeValue for packetType
				prErrMissingTypeValue,		// 28	typeValue missing or corrupt
				prErrExtraneousTypeValue,	// 29	Unexpected typeValue provided
				prErrPayloadOverrun,		// 30	Payload byte count to too large
				prErrAckTimeout,			// 31	Too much time for ACK/NAK
				prErrTalkTimeout,			// 32	Too much time in talk status
				prErrInterPacketTimeout,	// 33	Too much time between packets
				prErrExtraneousAck,			// 34	ACK/NAK where none was requested
				prErrPayloadParseError,		// 35	Payload failed parsing (applies 
				MAX_PROTOCOL_ERROR};		//		...to network functions only)


/* packet structure */

struct prPacket {
	// packet controls
	uint8_t	inChar;				// current input
	enum prStates state;		// current state
	enum prEvents event;		// current event
	uint8_t	ackCode;			// ACK or NAK code
	uint8_t bufferPtr;			// buffer pointer

	// header and payload data
	uint8_t	fromNode;			// from nodeID
	uint8_t toNode;				// to NodeID
	uint8_t	nextTalker;			// next talker
	uint8_t	packetType;
	uint8_t	typeValue;
	uint8_t	buffer[IO_BUFFER_LEN+1]; // for building header elements & payload
};

struct prPacket rx;				// receiving packet
struct prPacket tx;				// transmitting packet


/* general function prototypes */

void initProtocol(void);					// initialization
void prReceiveChar(uint8_t inChar);			// rx character parser entry point
void prExecutePacket(struct prPacket rx);	// pass rx packet to app layer
void prTransmitPacket(struct prPacket tx);	// transmit a packet

/* action function prototypes */

void prNop(void);					// used to fill spaces in state table
void prAnyEventListen(void);		// enter listen state from any event
void prAnyEventStart(void);			// start new packet from any event (no error)
void prAnyLoopDiscard(void);		// loop in current state, disard input char
void prAnyLoopSave(void);			// loop in current state, save input char

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

void prErrorHandler(struct prPacket rx, uint8_t errno);  // error handler for throws
void prThrowGenericError(void);
void prThrowGenericOverrun(void);
void prThrowPacketReceiveTimeout(void);
void prThrowMalformedStartPacket(void);
void prThrowMalformedFromNode(void);
void prThrowMalformedToNode(void);
void prThrowMalformedNextTalker(void);
void prThrowMalformedPacketType(void);
void prThrowMalformedTypeValue(void);
void prThrowInvalidFromNode(void);
void prThrowInvalidToNode(void);
void prThrowInvalidNextTalker(void);
void prThrowInvalidPacketType(void);
void prThrowInvalidTypeValue(void);
void prThrowPayloadOverrun(void);
