/*
	tg_protocol.c
	TinyG Protocol Handler
	Written by: Alden Hart
	Revision: 03/21/10

	References: "Writing Efficient State Machines in C" 
				 http://johnsantic.com/comp/state.html
*/
#include <tg_protocol.h>
#include <avr\pgmspace.h>

void initProtocol()
{
}

/************************************************************************************

	lookup tables (must precede the funtions that use them)

***********************************************************************************/

/* ASCII Character Mapper
   maps ASCII character to prEvent emnumeration (gotXXXX) 
 */

//const char prCharArray[] = {
//const unsigned char PROGMEM prCharArray[] = {
//const uint8_t PROGMEM prCharArray[] = {

//const uint8_t prCharArray[] = {
const uint8_t prCharArray[] PROGMEM = {

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


/* State/Event table
   prNop indicates an unused event: should never be called from that state 
 */

void (*const prStateTable [MAX_STATES][MAX_EVENTS]) (void) = {
//void (*const prStateTable [MAX_STATES][MAX_EVENTS]) (void) PROGMEM = {

	// INSERT ACTION ROUTINES FOR STATE/EVENT IN THIS TABLE
    
	// listen state
	{	
		prLoopTossChar,			// gotDigit	- prLoopTossChar action for gotDigit event
	 	prLoopTossChar,			// gotAlpha
	 	prListenBang,			// gotBang
	 	prLoopTossChar,			// gotEqual
	 	prLoopTossChar,			// gotEOS
	 	prLoopTossChar,			// gotWS
	 	prLoopTossChar,			// gotJunk
	 	prNop,					// gotTO	- There is no timeout in listen mode
	 	prNop,					// gotMAX
	 	prNop,					// gotOK
	 	prNop					// gotERR
	},
	
	// startPacket
	{	
		prStartPacketDigit,		// gotDigit	- convention for exit states = state+event
	 	prStartPacketAlpha,		// gotAlpha
	 	prRestartPacket,		// gotBang
	 	prUntrappedError,		// gotEqual	- untrapped errors kick back to listen
	 	prUntrappedError,		// gotEOS
	 	prRestartPacket,		// gotWS
	 	prUntrappedError,		// gotJunk
	 	prUntrappedError,		// gotTO
	 	prNop,					// gotMAX
	 	prNop,					// gotOK
	 	prNop					// gotERR
	},

	// rxFromNode
	{	
		prLoopSaveChar,			// gotDigit 
	 	prUntrappedError,		// gotAlpha	
	 	prRestartPacket,		// gotBang
	 	prUntrappedError,		// gotEqual
	 	prUntrappedError,		// gotEOS
	 	prUntrappedError,		// gotEqual
	 	prUntrappedError,		// gotJunk
	 	prCharTimeout,			// gotTO
	 	prNop,					// gotMAX
	 	prNop,					// gotOK
	 	prNop					// gotERR
	},

	// endFromNode
	{	
		prNop,					// gotDigit 
	 	prNop,					// gotAlpha	
	 	prRestartPacket,		// gotBang
	 	prNop,					// gotEqual
	 	prNop,					// gotEOS
	 	prNop,					// gotWS
	 	prNop,					// gotJunk
	 	prCharTimeout,			// gotTO
	 	prNop,					// gotMAX
	 	prNop,					// gotOK
	 	prNop					// gotERR
	},

	// rxToNode
	{	
		prNop,					// gotDigit 
	 	prNop,					// gotAlpha	
	 	prRestartPacket,		// gotBang
	 	prNop,					// gotEqual
	 	prNop,					// gotEOS
	 	prNop,					// gotWS
	 	prNop,					// gotJunk
	 	prCharTimeout,			// gotTO
	 	prNop,					// gotMAX
	 	prNop,					// gotOK
	 	prNop					// gotERR
	},

	// endToNode
	{	
		prNop,					// gotDigit 
	 	prNop,					// gotAlpha	
	 	prRestartPacket,		// gotBang
	 	prNop,					// gotEqual
	 	prNop,					// gotEOS
	 	prNop,					// gotWS
	 	prNop,					// gotJunk
	 	prCharTimeout,			// gotTO
	 	prNop,					// gotMAX
	 	prNop,					// gotOK
	 	prNop					// gotERR
	},

	// rxNextTalker
	{	
		prNop,					// gotDigit 
	 	prNop,					// gotAlpha	
	 	prRestartPacket,		// gotBang
	 	prNop,					// gotEqual
	 	prNop,					// gotEOS
	 	prNop,					// gotWS
	 	prNop,					// gotJunk
	 	prCharTimeout,			// gotTO
	 	prNop,					// gotMAX
	 	prNop,					// gotOK
	 	prNop					// gotERR
	},

	// endNextTalker
	{	
		prNop,					// gotDigit 
	 	prNop,					// gotAlpha	
	 	prRestartPacket,		// gotBang
	 	prNop,					// gotEqual
	 	prNop,					// gotEOS
	 	prNop,					// gotWS
	 	prNop,					// gotJunk
	 	prCharTimeout,			// gotTO
	 	prNop,					// gotMAX
	 	prNop,					// gotOK
	 	prNop					// gotERR
	},

	// rxPacketType
	{	
		prNop,					// gotDigit 
	 	prNop,					// gotAlpha	
	 	prRestartPacket,		// gotBang
	 	prNop,					// gotEqual
	 	prNop,					// gotEOS
	 	prNop,					// gotWS
	 	prNop,					// gotJunk
	 	prCharTimeout,			// gotTO
	 	prNop,					// gotMAX
	 	prNop,					// gotOK
	 	prNop					// gotERR
	},

	// rxTypeValue
	{	
		prNop,					// gotDigit 
	 	prNop,					// gotAlpha	
	 	prRestartPacket,		// gotBang
	 	prNop,					// gotEqual
	 	prNop,					// gotEOS
	 	prNop,					// gotWS
	 	prNop,					// gotJunk
	 	prCharTimeout,			// gotTO
	 	prNop,					// gotMAX
	 	prNop,					// gotOK
	 	prNop					// gotERR
	},

	// endPacketType
	{	
		prNop,					// gotDigit 
	 	prNop,					// gotAlpha	
	 	prRestartPacket,		// gotBang
	 	prNop,					// gotEqual
	 	prNop,					// gotEOS
	 	prNop,					// gotWS
	 	prNop,					// gotJunk
	 	prCharTimeout,			// gotTO
	 	prNop,					// gotMAX
	 	prNop,					// gotOK
	 	prNop					// gotERR
	},

	// rxPayload
	{	
		prNop,					// gotDigit 
	 	prNop,					// gotAlpha	
	 	prNop,					// gotBang
	 	prNop,					// gotEqual
	 	prNop,					// gotEOS
	 	prNop,					// gotWS
	 	prNop,					// gotJunk
	 	prCharTimeout,			// gotTO
	 	prNop,					// gotMAX
	 	prNop,					// gotOK
	 	prNop					// gotERR
	},

	// finPacket
	{	
		prNop,					// gotDigit 
	 	prNop,					// gotAlpha	
	 	prNop,					// gotBang
	 	prNop,					// gotEqual
	 	prNop,					// gotEOS
	 	prNop,					// gotWS
	 	prNop,					// gotJunk
	 	prCharTimeout,			// gotTO
	 	prNop,					// gotMAX
	 	prNop,					// gotOK
	 	prNop					// gotERR
	},

	// execPacket
	{	
		prNop,					// gotDigit 
	 	prNop,					// gotAlpha	
	 	prNop,					// gotBang
	 	prNop,					// gotEqual
	 	prNop,					// gotEOS
	 	prNop,					// gotWS
	 	prNop,					// gotJunk
	 	prCharTimeout,			// gotTO
	 	prNop,					// gotMAX
	 	prNop,					// gotOK
	 	prNop					// gotERR
	},

	// ackAppPacket
	{	
		prNop,					// gotDigit 
	 	prNop,					// gotAlpha	
	 	prNop,					// gotBang
	 	prNop,					// gotEqual
	 	prNop,					// gotEOS
	 	prNop,					// gotWS
	 	prNop,					// gotJunk
	 	prCharTimeout,			// gotTO
	 	prNop,					// gotMAX
	 	prNop,					// gotOK
	 	prNop					// gotERR
	}
};

/************************************************************************************

	protocol main functions

************************************************************************************/

/****** prRunProtocol(inChar)	main entry point for protocol
		normally this would be a layered set of routines, but this is an inner loop
*/

void prRunProtocol(unsigned char inChar)
{
  /* call the action procedure. 
  	 assumes all values are in range (no error checking)
  	 actually the following 2 lines put together:

  prEvent = prCharArray[inChar];					// get next character event
  prStateTable [prState][prEvent] ();				// call the action procedure

	 range checked version is:

  if (((prEvent >= 0) && (prEvent < MAX_EVENTS)) && 
  	  ((prState >= 0) && (prState < MAX_STATES))) 
  {
	prStateTable [prState][prEvent] ();			// call the action procedure
  } else {
	// invalid state or event
  };
  */

//  prEvent = prCharArray[inChar];					// get next character event
  prEvent = pgm_read_byte(&(prCharArray[inChar]));					// get next character event

//byte = pgm_read_byte(&(mydata[i][j]));


  prStateTable[prState][prEvent]();					// call the action procedure

//  pgm_read_word(&(prStateTable[prState][prEvent]()));					// call the action procedure


//  prStateTable[prState][prCharArray[inChar]]();		// call the action procedure

};


/****** prDecodeChar()	returns an enum based on character
		one of: gotDigit, gotAlpha, gotBang, gotEqual, gotEOS, gotWS, gotJunk
*/

//int prDecodeChar(unsigned char inChar)
enum prEvents prDecodeChar(unsigned char inChar)
{
/*
  switch (inChar) {
	case '!':
	  return(gotBang);
	  break;
	case '!':
	  return(gotBang);
	  break;
	default;
	  return(gotJunk);
  }
*/
//  return gotDigit;
  return gotAlpha;
};


/************************************************************************************

	generic action routines - used by many states

***********************************************************************************/


void prNop(void) {}				// no-op


void prReturnToListen(void)		// exit to listen
{
}

void prUntrappedError(void)		// exit for error that cannot be NAK'd ("L" on chart)
{
  prState = listen;
}

void prTrappedError(void)		// exit for error that can be NAK'd ("F" on chart)
{
}

void prLoopTossChar(void)		// loop in current state, toss input character
{
}

void prLoopSaveChar(void)		// loop - save input character to string buffer
{
}

void prRestartPacket(void)		// go to beginning of new packet - no error thrown
{
  prState = startPacket;		// return to start packet
}

void prCharTimeout(void)		// inter-character timeout
{
}


/************************************************************************************

	state-specific action routines - naming convention is State + Action

***********************************************************************************/

void prListenBang(void)			// exit listen via exclamation point
{
  prState = startPacket;
}

void prStartPacketDigit(void)	// exit startPacket via digit
{
  prState = rxFromNode;
}

void prStartPacketAlpha(void)	// exit startPacket via alpha
{
  prState = rxPacketType;
}




