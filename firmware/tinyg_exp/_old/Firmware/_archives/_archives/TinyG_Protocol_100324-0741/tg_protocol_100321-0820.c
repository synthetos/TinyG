/*
	tg_protocol.c
	TinyG Protocol Handler
	Written by: Alden Hart
	Revision: 03/21/10

	References: "Writing Efficient State Machines in C" 
				 http://johnsantic.com/comp/state.html
*/
#include <tg_protocol.h>

void initProtocol()
{
}


/************************************************************************************

	state/event lookup table 

		prNop indicates an unused event: should never be called from that state 

***********************************************************************************/

void (*const prStateTable [MAX_STATES][MAX_EVENTS]) (void) = {

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

	functions

************************************************************************************/

/*
	prRunProtocol(inChar)	main entry point for protocol
		normally this would be a layered set of routines, but this is an inner loop

*/
void prRunProtocol(inChar)		// main entry point for protocol
{

  prDecodeChar(inChar);
}




void parseEvent()
{
  prEvent = prDecodeChar();		// get next character event to process

  if (((prEvent >= 0) && (prEvent < MAX_EVENTS)) && ((prState >= 0) && (prState < MAX_STATES))) {
	prStateTable [prState][prEvent] ();		// call the action procedure
  } else {
	/* invalid state or event */
  };
};


/*
	prDecodeChar()	returns an enum based on character
		one of: gotDigit, gotAlpha, gotBang, gotEqual, gotEOS, gotWS, gotJunk
*/

enum prEvents prDecodeChar (inChar)
{
//  return gotDigit;
  return gotAlpha;
};


/************************************************************************************

	Generic action routines
		used by many states

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

	State-specific action routines
		convention is State + Action

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


