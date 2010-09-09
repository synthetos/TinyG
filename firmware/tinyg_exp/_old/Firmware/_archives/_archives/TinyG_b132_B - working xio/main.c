/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support

  TinyG Notes:
   Modified Grbl to support Xmega family processors
   Modifications Copyright (c) 2010 Alden S. Hart, Jr.

!!!	To compile and link you must use libm.a otherwise the floating point will fail.
	In AVRstudio select Project / Configuration Options
	Select Libraries
	Move libm.a from the left pane to the right pane
	ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=80040&start=0

	When asked to browse for stdlib files, go to: C:\WinAVR-20100110\avr\lib\avrxmega6
	When asked to browse for include files go to: C:\WinAVR-20100110\avr\include

	Configure project - 32000000 Hz processor, and also set 32.0000 Mhz in debug configs

  Another annoying avr20100110 bug: 
  	If you are running WinAVR-20100110 you may be asked to locate libraries or 
	include files that were known to a previous avr-gcc version. If so,
  	browse to this dir for Libs: C:\WinAVR-20100110\avr\lib\avrxmega6

*/

#include "xmega_support.h"	// must precede <util/delay> and app defines

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "xmega_interrupts.h"
#include "xio.h"
#include "xio_usb.h"

#include "tinyg.h"

extern FILE dev_usb;

/*
 * main
 */

int main(void) 
{
//	char c;
	char rd_buf[20];

	/* These inits are order dependent */
	cli();
	xmega_init();				// xmega setup
	xio_init();

	PMIC_SetVectorLocationToApplication();  // as opposed to boot rom
//	PMIC_EnableLowLevel();		// nothing at this level
	PMIC_EnableMediumLevel(); 	// enable serial IO
	PMIC_EnableHighLevel();		// enable stepper timers
	sei();						// enable global interrupts

	// Hello World strings
	printf_P(PSTR("TinyG [TEST MODE] - Version "));
	printf_P(PSTR(TINYG_VERSION));		// Did you know that printf actually...
	printf_P(PSTR("\r\n"));				//...prints to stderr and not stdout?

	for(;;){

		printf_P(PSTR("Enter command: "));
		if (fgets(rd_buf, sizeof(rd_buf)-1, stdin) == 0) {
			printf_P(PSTR("Read Error\r\n"));
		}
		printf(rd_buf);
    }
}



/*

		if ((c = fgetc(stdin)) == EOF) {
			printf_P(PSTR("Got an EOF\r\n"));
		}


		if (tolower(buf[0]) == 'q')
			break;

		switch (tolower(buf[0])) {
			default:
	  			printf("Unknown command: %s\n", buf);
	  			break;

			case 'l':
				if (sscanf(buf, "%*s %s", s) > 0) {
					fprintf(&dev_usb, "Got %s\n", s);
					printf("OK\n");
				} else {
					printf("sscanf() failed\n");
				}
				break;

			case 'u':
				if (sscanf(buf, "%*s %s", s) > 0) {
					fprintf(&dev_usb, "Got %s\n", s);
					printf("OK\n");
				} else {
					printf("sscanf() failed\n");
				}
				break;
		}

	while((c = serialRead()) != 0x04) {				// 0x04 is ASCII ETX
		if((c == '\r') || (c == '\n') || (c == ';')) {  // Line complete. Execute!
			textline[i] = 0;						// terminate and echo the string
			printPgmString(PSTR("\r\n EXEC>> "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			tg_print_gcstatus(gc_execute_line(textline));	// execute cmd & show status
			i = 0;
			textline[i] = 0;						// reset the buffer
			tg_prompt();							
		} else if ((c == 0x08) || (c == 0x7F)) {  	// backspace or delete
			textline[--i] = 0;
		} else if (c <= ' ') { 						// throw away WS & ctrl chars
		} else if (c >= 'a' && c <= 'z') {			// convert lower to upper
			textline[i++] = c-'a'+'A';
		} else {
			textline[i++] = c;
		}
	}


*/

