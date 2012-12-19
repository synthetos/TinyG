//#include "compiler.h"
//#include "ccp.h"
#include <avr/wdt.h>
 
void wdt_reset_mcu(void)
{
/*
	uint8_t temp;

    // WDT enabled (minimum timeout period for max. security) 
    temp = WDT_PER_8CLK_gc | (1 << WDT_ENABLE_bp) | (1 << WDT_CEN_bp);
	ccp_write_io((void *)&WDT.CTRL, temp);
    wdt_wait_while_busy();

     // WDT enabled (maximum window period for max. security)
    temp = WDT_WPER_8KCLK_gc | (1 << WDT_WEN_bp) | (1 << WDT_WCEN_bp);
    ccp_write_io((void *)&WDT.WINCTRL, temp);
    wdt_wait_while_busy();

     // WDT Reset during window => WDT generates an Hard Reset.
    wdt_reset();

    // No exit to prevent the execution of the following instructions.
	while (1) {}
*/

}
