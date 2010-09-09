/*
	tg_protocol.c
	TinyG Protocol Handler
	Written by: Alden Hart
	Revision: 03/27/10

	References: "Writing Efficient State Machines in C" 
				 http://johnsantic.com/comp/state.html
*/

#include <stdio.h>
#include <stdlib.h>
#include <avr\io.h>
#include <avr\pgmspace.h>

#include <tg_protocol.h>
#include <tg_serialio.h>

void initProtocol()
{
  // no inits, for now
}

/************************************************************************************

	lookup tables (must precede the functions that use them)

***********************************************************************************/

/* Event Mapper
   Maps ASCII character to prEvents emnumeration (gotXXXX)
   Runs from program memory (requires pgm_read_byte macro to read) 
 */

//const uint8_t prEventMap[] = {			// version that works from data RAM 
											// must also change accesses to array
const uint8_t prEventMap[] PROGMEM = {

					// dec  hex symbol
		gotEOS, 	//	0	00	NUL	(Null char)
		gotSOH, 	//	1	01	SOH	(Start of Header)
		gotJunk, 	//	2	02	STX	(Start of Text)
		gotJunk, 	//	3	03	ETX	(End of Text)
		gotJunk, 	//	4	04	EOT	(End of Transmission)
		gotJunk, 	//	5	05	ENQ	(Enquiry)
		gotJunk, 	//	6	06	ACK	(Acknowledgment)
		gotJunk, 	//	7	07	BEL	(Bell)
		gotJunk, 	//	8	08	BS	(Backspace)
		gotWS, 		//	9	09	HT	(Horizontal Tab)
		gotEOS, 	//	10	0A	LF	(Line Feed)
		gotJunk, 	//	11	0B	VT	(Vertical Tab)
		gotJunk, 	//	12	0C	FF	(Form Feed)
		gotEOS, 	//	13	0D	CR	(Carriage Return)
		gotJunk, 	//	14	0E	SO	(Shift Out)
		gotJunk, 	//	15	0F	SI	(Shift In)
		gotJunk, 	//	16	10	DLE	(Data Link Escape)
		gotJunk, 	//	17	11	DC1 (XON) (Device Control 1)	
		gotJunk, 	//	18	12	DC2	(Device Control 2)
		gotJunk, 	//	19	13	DC3 (XOFF)(Device Control 3)	
		gotJunk, 	//	20	14	DC4	(Device Control 4)
		gotJunk, 	//	21	15	NAK (Negativ Acknowledgemnt)	
		gotJunk, 	//	22	16	SYN	(Synchronous Idle)
		gotJunk, 	//	23	17	ETB	(End of Trans. Block)
		gotJunk, 	//	24	18	CAN	(Cancel)
		gotJunk, 	//	25	19	EM	(End of Medium)
		gotJunk, 	//	26	1A	SUB	(Substitute)
		gotJunk, 	//	27	1B	ESC	(Escape)
		gotJunk, 	//	28	1C	FS	(File Separator)
		gotJunk, 	//	29	1D	GS	(Group Separator)
		gotJunk, 	//	30	1E	RS  (Reqst to Send)(Record Sep.)	
		gotJunk, 	//	31	1F	US	(Unit Separator)
		gotWS, 		//	32	20	SP	(Space)
		gotSOH, 	//	33	21	!	(exclamation mark)
		gotJunk, 	//	34	22	,	(double quote)	
		gotJunk, 	//	35	23	#	(number sign)
		gotJunk, 	//	36	24	$	(dollar sign)
		gotJunk, 	//	37	25	%	(percent)
		gotJunk, 	//	38	26	&	(ampersand)
		gotJunk, 	//	39	27	'	(single quote)
		gotJunk, 	//	40	28	(	(left/open parenthesis)
		gotJunk, 	//	41	29	)	(right/closing parenth.)
		gotJunk, 	//	42	2A	*	(asterisk)
		gotJunk, 	//	43	2B	+	(plus)
		gotJunk, 	//	44	2C		(comma)
		gotJunk, 	//	45	2D	-	(minus or dash)
		gotJunk, 	//	46	2E	.	(dot)
		gotJunk, 	//	47	2F	/	(forward slash)
		gotDigit, 	//	48	30	0	
		gotDigit, 	//	49	31	1	
		gotDigit, 	//	50	32	2	
		gotDigit, 	//	51	33	3	
		gotDigit, 	//	52	34	4	
		gotDigit, 	//	53	35	5	
		gotDigit, 	//	54	36	6	
		gotDigit, 	//	55	37	7	
		gotDigit, 	//	56	38	8	
		gotDigit, 	//	57	39	9	
		gotJunk, 	//	58	3A	:	(colon)
		gotEOS, 	//	59	3B	;	(semi-colon)
		gotJunk, 	//	60	3C	<	(less than)
		gotEqual, 	//	61	3D	=	(equal sign)
		gotJunk, 	//	62	3E	>	(greater than)
		gotJunk, 	//	63	3F	?	(question mark)
		gotJunk, 	//	64	40	@	(AT symbol)
		gotAlpha,	//	65	41	A	
		gotAlpha,	//	66	42	B	
		gotAlpha,	//	67	43	C	
		gotAlpha,	//	68	44	D	
		gotAlpha,	//	69	45	E	
		gotAlpha,	//	70	46	F	
		gotAlpha,	//	71	47	G	
		gotAlpha,	//	72	48	H	
		gotAlpha,	//	73	49	I	
		gotAlpha,	//	74	4A	J	
		gotAlpha,	//	75	4B	K	
		gotAlpha,	//	76	4C	L	
		gotAlpha,	//	77	4D	M	
		gotAlpha,	//	78	4E	N	
		gotAlpha,	//	79	4F	O	
		gotAlpha,	//	80	50	P	
		gotAlpha,	//	81	51	Q	
		gotAlpha,	//	82	52	R	
		gotAlpha,	//	83	53	S	
		gotAlpha,	//	84	54	T	
		gotAlpha,	//	85	55	U	
		gotAlpha,	//	86	56	V	
		gotAlpha,	//	87	57	W	
		gotAlpha,	//	88	58	X	
		gotAlpha,	//	89	59	Y	
		gotAlpha,	//	90	5A	Z	
		gotJunk,	//	91	5B	[	(left/opening bracket)
		gotJunk,	//	92	5C	\	(back slash)
		gotJunk,	//	93	5D	]	(right/closing bracket)
		gotJunk,	//	94	5E	^	(caret/circumflex)
		gotJunk,	//	95	5F	_	(underscore)
		gotJunk,	//	96	60	`	
		gotAlpha,	//	97	61	a	
		gotAlpha,	//	98	62	b	
		gotAlpha,	//	99	63	c	
		gotAlpha,	//	100	64	d	
		gotAlpha,	//	101	65	e	
		gotAlpha,	//	102	66	f	
		gotAlpha,	//	103	67	g	
		gotAlpha,	//	104	68	h	
		gotAlpha,	//	105	69	i	
		gotAlpha,	//	106	6A	j	
		gotAlpha,	//	107	6B	k	
		gotAlpha,	//	108	6C	l	
		gotAlpha,	//	109	6D	m	
		gotAlpha,	//	110	6E	n	
		gotAlpha,	//	111	6F	o	
		gotAlpha,	//	112	70	p	
		gotAlpha,	//	113	71	q	
		gotAlpha,	//	114	72	r	
		gotAlpha,	//	115	73	s	
		gotAlpha,	//	116	74	t	
		gotAlpha,	//	117	75	u	
		gotAlpha,	//	118	76	v	
		gotAlpha,	//	119	77	w	
		gotAlpha,	//	120	78	x	
		gotAlpha,	//	121	79	y	
		gotAlpha,	//	122	7A	z	
		gotJunk,	//	123	7B	{	(left/opening brace)
		gotJunk,	//	124	7C	|	(vertical bar)
		gotJunk,	//	125	7D	}	(right/closing brace)
		gotJunk,	//	126	7E	~	(tilde)
		gotJunk		//	127	7F	DEL	(delete)
};

/* PacketType Mapper
   Maps ASCII character to packet type enum
   Index is (ASCII character - 0x40) & (0x1F)
   This provides a case insensitive alpha lookup
   Obvoiusly this works for single character packetTypes only
   Runs from program memory (requires pgm_read_byte macro to read) 
 */

const uint8_t prPacketTypeMap[] PROGMEM = {

						// dec  hex symbol
		packInvalid, 	//	64	40	@	(AT symbol)
		packAck,		//	65	41	A	
		packBack,		//	66	42	B	
		packConfig,		//	67	43	C	
		packData,		//	68	44	D	
		packEnd,		//	69	45	E	
		packInvalid,	//	70	46	F	
		packInvalid,	//	71	47	G	
		packHush,		//	72	48	H	
		packInvalid,	//	73	49	I	
		packInvalid,	//	74	4A	J	
		packInvalid,	//	75	4B	K	
		packInvalid,	//	76	4C	L	
		packInvalid,	//	77	4D	M	
		packNodeid,		//	78	4E	N	
		packInvalid,	//	79	4F	O	
		packPing,		//	80	50	P	
		packQuery,		//	81	51	Q	
		packResend,		//	82	52	R	
		packStart,		//	83	53	S	
		packTalk,		//	84	54	T	
		packInvalid,	//	85	55	U	
		packInvalid,	//	86	56	V	
		packInvalid,	//	87	57	W	
		packInvalid,	//	88	58	X	
		packInvalid,	//	89	59	Y	
		packInvalid,	//	90	5A	Z	
		packInvalid,	//	91	5B	[	(left/opening bracket)
		packInvalid,	//	92	5C	\	(back slash)
		packInvalid,	//	93	5D	]	(right/closing bracket)
		packInvalid,	//	94	5E	^	(caret/circumflex)
		packInvalid		//	95	5F	_	(underscore)
};

/* PacketType Strings - for output as ASCII
   This is an example of how to put an entire string table into program memory
   The order of strings in the table must match order of prPacketTypes enum
   Access is by: (PGM_P)pgm_read_word(&(prPacketTypeStrings[i]))
     where i is the prPacketTypes enum, e.g. packData
   ref: http://www.cs.mun.ca/~paul/cs4723/material/atmel/avr-libc-user-manual-1.6.5/pgmspace.html
 */

char prPacketTypeStringInvalid[] PROGMEM = "invalid";
char prPacketTypeStringTalk[] 	 PROGMEM = "talk";
char prPacketTypeStringData[] 	 PROGMEM = "data";
char prPacketTypeStringAck[] 	 PROGMEM = "ack";
char prPacketTypeStringBack[] 	 PROGMEM = "back";
char prPacketTypeStringQuery[]	 PROGMEM = "query";
char prPacketTypeStringNodeid[]	 PROGMEM = "nodeid";
char prPacketTypeStringConfig[]	 PROGMEM = "config";
char prPacketTypeStringPing[]	 PROGMEM = "ping";
char prPacketTypeStringStart[]	 PROGMEM = "start";
char prPacketTypeStringHush[]	 PROGMEM = "hush";
char prPacketTypeStringResend[]	 PROGMEM = "resend";
char prPacketTypeStringEnd[]	 PROGMEM = "end";

PGM_P prPacketTypeStrings[] PROGMEM = 
{
	prPacketTypeStringInvalid,
	prPacketTypeStringTalk,
	prPacketTypeStringData,
	prPacketTypeStringAck,
	prPacketTypeStringBack,
	prPacketTypeStringQuery,
	prPacketTypeStringNodeid,
	prPacketTypeStringConfig,
	prPacketTypeStringPing,
	prPacketTypeStringStart,
	prPacketTypeStringHush,
	prPacketTypeStringResend,
	prPacketTypeStringEnd
};


/* State/Event table
   prNop indicates an unused event: should never be called from that state 
 */

void (*const prStateTable[MAX_STATES][MAX_EVENTS])(void) = {
//void (*const prStateTable[MAX_STATES][MAX_EVENTS])(void) PROGMEM = {
//(PGM_VOID_P) (prStateTable[MAX_STATES][MAX_EVENTS])(void) __ATTR_CONST__ PROGMEM = {

	// INSERT ACTION ROUTINES FOR STATE/EVENT IN THIS TABLE
    
	// listen state
	{	
	 	prAnyEventStart,			// gotSOH
		prAnyLoopDiscard,			// gotDigit
	 	prAnyLoopDiscard,			// gotAlpha
	 	prAnyLoopDiscard,			// gotEqual
	 	prAnyLoopDiscard,			// gotEOS
	 	prAnyLoopDiscard,			// gotJunk
	 	prAnyLoopDiscard,			// gotWS
	 	prNop						// gotTO	- There is no timeout in listen mode
	},
	
	// startPacket
	{	
	 	prAnyEventStart,			// gotSOH
		prStartPacketDigitExit,		// gotDigit	- exit to rxFromNode state
	 	prStartPacketAlphaExit,		// gotAlpha - exit to exPacketType 
	 	prThrowMalformedStartPacket,// gotEqual	- kick back to listen
	 	prThrowMalformedStartPacket,// gotEOS
	 	prThrowMalformedStartPacket,// gotJunk
	 	prAnyLoopDiscard,			// gotWS	- discard the whitespace
	 	prThrowPacketReceiveTimeout	// gotTO
	},

	// rxFromNode
	{	
	 	prAnyEventStart,			// gotSOH
		prAnyLoopSave,				// gotDigit	- save the digit 
	 	prThrowMalformedFromNode,	// gotAlpha	
	 	prThrowMalformedFromNode,	// gotEqual
	 	prThrowMalformedFromNode,	// gotEOS
	 	prThrowMalformedFromNode,	// gotJunk
	 	prRxFromNodeSpaceExit,		// gotWS	- exit to endFromNode
	 	prThrowPacketReceiveTimeout	// gotTO
	},

	// endFromNode
	{	
	 	prAnyEventStart,			// gotSOH
		prEndFromNodeDigitExit,		// gotDigit	- exit to rxToNode 
	 	prThrowMalformedToNode,		// gotAlpha	
	 	prThrowMalformedToNode,		// gotEqual
	 	prThrowMalformedToNode,		// gotEOS
	 	prThrowMalformedToNode,		// gotJunk
	 	prAnyLoopDiscard,			// gotWS
	 	prThrowPacketReceiveTimeout	// gotTO
	},

	// rxToNode
	{	
	 	prAnyEventStart,			// gotSOH
		prAnyLoopSave,				// gotDigit	- save the digit 
	 	prThrowMalformedToNode,		// gotAlpha	
	 	prThrowMalformedToNode,		// gotEqual
	 	prThrowMalformedToNode,		// gotEOS
	 	prThrowMalformedToNode,		// gotJunk
	 	prRxToNodeSpaceExit,		// gotWS	- exit to endToNode
	 	prThrowPacketReceiveTimeout	// gotTO
	},

	// endToNode
	{	
	 	prAnyEventStart,			// gotSOH
		prEndToNodeDigitExit,		// gotDigit	- exit to rxNextTalker 
	 	prEndToNodeAlphaExit,		// gotAlpha	- exit to rxPacketType
	 	prThrowMalformedNextTalker,	// gotEqual
	 	prThrowMalformedNextTalker,	// gotEOS
	 	prThrowMalformedNextTalker,	// gotJunk
	 	prAnyLoopDiscard,			// gotWS
	 	prThrowPacketReceiveTimeout	// gotTO
	},

	// rxNextTalker
	{	
	 	prAnyEventStart,			// gotSOH
		prAnyLoopSave,				// gotDigit	- save the digit 
	 	prThrowMalformedNextTalker,	// gotAlpha	
	 	prThrowMalformedNextTalker,	// gotEqual
	 	prThrowMalformedNextTalker,	// gotEOS
	 	prThrowMalformedNextTalker,	// gotJunk
	 	prRxNextTalkerSpaceExit,	// gotWS	- exit to endNextTalker
	 	prThrowPacketReceiveTimeout	// gotTO
	},

	// endNextTalker
	{	
	 	prAnyEventStart,			// gotSOH
		prThrowMalformedPacketType,	// gotDigit
	 	prEndNextTalkerAlphaExit,	// gotAlpha - exit to rxPacketType
	 	prThrowMalformedPacketType,	// gotEqual
	 	prThrowMalformedPacketType,	// gotEOS
	 	prThrowMalformedPacketType,	// gotJunk
	 	prAnyLoopDiscard,			// gotWS
	 	prThrowPacketReceiveTimeout	// gotTO
	},

	// rxPacketType
	{	
	 	prAnyEventStart,			// gotSOH
		prThrowMalformedPacketType,	// gotDigit 
	 	prAnyLoopSave,				// gotAlpha	
	 	prRxPacketTypeEqualExit,	// gotEqual
	 	prThrowMalformedPacketType,	// gotEOS
	 	prThrowMalformedPacketType,	// gotJunk
	 	prRxPacketTypeSpaceExit,	// gotWS
	 	prThrowPacketReceiveTimeout	// gotTO
	},

	// rxTypeValue
	{	
	 	prAnyEventStart,			// gotSOH
		prAnyLoopSave,				// gotDigit 
	 	prAnyLoopSave,				// gotAlpha	
	 	prThrowMalformedTypeValue,	// gotEqual
	 	prThrowMalformedTypeValue,	// gotEOS
	 	prThrowMalformedTypeValue,	// gotJunk
	 	prRxTypeValueSpaceExit,		// gotWS
	 	prThrowPacketReceiveTimeout	// gotTO
	},

	// rxPayload
	{	
	 	prAnyLoopSave,				// gotSOH
		prAnyLoopSave,				// gotDigit 
	 	prAnyLoopSave,				// gotAlpha	
	 	prAnyLoopSave,				// gotEqual
	 	prRxPayloadEndExit,			// gotEOS
	 	prAnyLoopSave,				// gotJunk
	 	prAnyLoopSave,				// gotWS
	 	prThrowPacketReceiveTimeout	// gotTO
	}
};


/************************************************************************************

	protocol main functions

************************************************************************************/

/****** prReceiveChar(inChar) ******	receive and parse input character
		At 1 Mbps characters arrive every 10 uSec, so it's optimized for speed
		Index correctness flows from a 7 bit input and well constructed tables,
		...no range checking is performed.
*/

void prReceiveChar(uint8_t inChar)
{
  rx.inChar = inChar & 0x7F;							// mask any errant MSBs
  rx.event = pgm_read_byte(&(prEventMap[rx.inChar])); 	// get event
  prStateTable[rx.state][rx.event]();					// call action procedure

// Failed attempt to also try to put the state table in program memory
//  prEvent = prEventMap[inChar];				// get next character event
//  ((prStateTable[prState][prEvent]()));		// call action procedure
};

void prExecutePacket(struct prPacket rx)		// pass rx packet to app layer
{
  prTransmitPacket(rx);
}

/****** prTransmitPacket ******
	Simple transmit a packet. 
	Header data is in the tx struct and payload is in the buffer
 */

void prTransmitPacket(struct prPacket tx)
{
//  uint8_t txbyte;						// transmit byte
  char txstring[4];						// temp string for transmitting numbers as ASCII

  usartWriteChar('!'); 					// send start header

  itoa(tx.fromNode, txstring, 10); 		// send fromNode
  usartWriteString(txstring);

  itoa(tx.toNode, txstring, 10); 		// send fromNode
  usartWriteString(txstring);

  if (tx.nextTalker)					// if non-zero
  {
    itoa(tx.nextTalker, txstring, 10); 	// send fromNode
    usartWriteString(txstring);
  };

  usartWriteString((PGM_P)pgm_read_word(&(prPacketTypeStrings[tx.packetType])));

  usartWriteString((char *)tx.buffer);			// send payload

}


/************************************************************************************

	generic action routines - used by many states
	naming convention is: State - Event - Action or next state

***********************************************************************************/

void prNop(void) {}					// no-op filler routine

void prAnyEventListen(void)			// exit to listen from any event (with no error)
{
  rx.state = listen;				// no inits are necessary - occur at packet start
}

void prAnyEventStart(void)			// start a new packet from any event
{
  rx.bufferPtr = 0;					// reset buffer
  rx.buffer[rx.bufferPtr] = 0;		// write a NULL
  rx.state = startPacket;			// return to start packet
}

void prAnyLoopDiscard(void)			// loop in current state, do not save character
{
  // no operation
}

void prAnyLoopSave(void)			// loop in current state, save char to rx buffer
{
  rx.buffer[rx.bufferPtr++] = rx.inChar; 	// append character to buffer
  rx.buffer[rx.bufferPtr] = 0;				// with a trailing NULL

  if (rx.bufferPtr > IO_BUFFER_LEN)
  {
	prThrowGenericOverrun();		// overrun NAK
  };
}


/************************************************************************************

	state-specific action routines
	naming convention is: State - Event - Action or next state

***********************************************************************************/

void prStartPacketDigitExit(void)	// exit startPacket to rxFromNode
{
  prAnyLoopSave();					// append digit to receive buffer
  rx.state = rxFromNode;			// next state
}

void prStartPacketAlphaExit(void)	// exit startPacket to rxPacketType
{
  rx.state = rxPacketType;
}

void prRxFromNodeSpaceExit(void)	// exit rxFromNode to endFromNode
{
  int	node;

  node = atoi((char *)rx.buffer);	// atoi fromNode

  if (node >= MAX_NODE)
  {
    prThrowInvalidFromNode();
	return;
  }
  rx.fromNode = (uint8_t)node;		// save fromNode value
  rx.bufferPtr = 0;					// reset buffer
  rx.buffer[rx.bufferPtr] = 0;
  rx.state = endFromNode;
}

void prEndFromNodeDigitExit(void)	// exit from endFromNode to rxToNode
{
  prAnyLoopSave();					// append digit to receive buffer
  rx.state = rxToNode;
}

void prRxToNodeSpaceExit(void)		// exit rxFromNode to endFromNode
{
  int	node;

  node = atoi((char *)rx.buffer);	// atoi toNode
  if (node >= MAX_NODE)
  {
	prThrowInvalidToNode();
	return;
  }
  rx.toNode = (uint8_t)node;
  rx.bufferPtr = 0;					// reset buffer
  rx.buffer[rx.bufferPtr] = 0;
  rx.state = endToNode;
}

void prEndToNodeDigitExit(void)		// exit from endFromNode to rxToNode
{
  prAnyLoopSave();					// append digit to receive buffer
  rx.state = rxNextTalker;
}

void prEndToNodeAlphaExit(void)		// exit from endFromNode to rxPacketType
{
  prAnyLoopSave();					// append char to receive buffer
  rx.state = rxPacketType;
}

void prRxNextTalkerSpaceExit(void)	// exit rxNextTalker to endNextTalker
{
  int	node;

  node = atoi((char *)rx.buffer); 	// atoi nextTalker
  if (node >= MAX_NODE)
  {
    prThrowInvalidNextTalker();
	return;
  };
  rx.nextTalker = (uint8_t)node;
  rx.bufferPtr = 0;					// reset buffer
  rx.buffer[rx.bufferPtr] = 0;
  rx.state = endNextTalker;
}

void prEndNextTalkerAlphaExit(void)	// exit endNextTalker to rxPacketType
{
  prAnyLoopSave();					// append char to receive buffer
  rx.state = rxPayload;
}

void prRxPacketTypeSpaceExit(void)	// exit rxPacketType to endPacketType
{
  // get packet type (refer to prPacketTypeMap to see how this line works)
  rx.packetType = pgm_read_byte(&(prPacketTypeMap[(((int)rx.buffer[0]-0x40)&0x1F)]));

  if (rx.packetType == packInvalid)
  {
    prThrowInvalidPacketType();
	return;
  }

  // you could insert tests here to see if any of the packetTypes require values

  rx.bufferPtr = 0;					// reset buffer
  rx.buffer[rx.bufferPtr] = 0;		// with a trailing NULL
  rx.state = rxPayload;
}

void prRxPacketTypeEqualExit(void)	// exit rxPacketType to rxTypeValue
{
  // get packet type (refer to prPacketTypeMap to see how this line works)
  rx.packetType = pgm_read_byte(&(prPacketTypeMap[(((int)rx.buffer[0]-0x40)&0x1F)]));

  if (rx.packetType == packInvalid)
  {
    prThrowInvalidPacketType();
	return;
  }
  rx.bufferPtr = 0;					// reset buffer
  rx.buffer[rx.bufferPtr] = 0;
  rx.state = rxTypeValue;
}

void prRxTypeValueSpaceExit(void)	// exit rxTypeValue to endPacketType
{
  rx.bufferPtr = 0;					// reset buffer
  rx.buffer[rx.bufferPtr] = 0;
  rx.state = rxPayload;
}

void prRxPayloadEndExit(void)		// exit rxPayload to endPacket
{
  rx.state = listen;				// do this first so receiver can restart 
  // don't need to replace EOS with NULL in string - string already ends w/NULL

  // queue ACK
  // exec packet (pass to app layer)
  prExecutePacket(rx);
}

/************************************************************************************

	exception handler action routines

***********************************************************************************/

void prErrorHandler(struct prPacket rx, uint8_t errno)
{
  rx.ackCode = errno;		// for now, just record the error code,
  rx.state = listen;		// ... and return to listen state
}

void prThrowGenericError(void) 			{ prErrorHandler(rx, prErrGenericError); }
void prThrowGenericOverrun(void) 		{ prErrorHandler(rx, prErrGenericOverrun); }
void prThrowPacketReceiveTimeout(void)	{ prErrorHandler(rx, prErrPacketReceiveTimeout); }
void prThrowMalformedStartPacket(void)	{ prErrorHandler(rx, prErrMalformedStartPacket); }
void prThrowMalformedFromNode(void)		{ prErrorHandler(rx, prErrMalformedFromNode); }
void prThrowMalformedToNode(void)		{ prErrorHandler(rx, prErrMalformedToNode); }
void prThrowMalformedNextTalker(void)	{ prErrorHandler(rx, prErrMalformedNextTalker); }
void prThrowMalformedPacketType(void)	{ prErrorHandler(rx, prErrMalformedPacketType); }
void prThrowMalformedTypeValue(void)	{ prErrorHandler(rx, prErrMalformedTypeValue); }
void prThrowInvalidFromNode(void)		{ prErrorHandler(rx, prErrInvalidFromNode); }
void prThrowInvalidToNode(void)			{ prErrorHandler(rx, prErrInvalidToNode); }
void prThrowInvalidNextTalker(void)		{ prErrorHandler(rx, prErrInvalidNextTalker); }
void prThrowInvalidPacketType(void)		{ prErrorHandler(rx, prErrInvalidPacketType); }
void prThrowInvalidTypeValue(void)		{ prErrorHandler(rx, prErrInvalidTypeValue); }
void prThrowPayloadOverrun(void)		{ prErrorHandler(rx, prErrPayloadOverrun); }



