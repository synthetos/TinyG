/************************************************************************/
/* XBoot Extensible AVR Bootloader                                      */
/*                                                                      */
/* tested with ATXMEGA64A3, ATXMEGA128A1, ATXMEGA256A1, ATXMEGA32A4     */
/*                                                                      */
/* xboot.c                                                              */
/*                                                                      */
/* Alex Forencich <alex@alexforencich.com>                              */
/*                                                                      */
/* Copyright (c) 2010 Alex Forencich                                    */
/*                                                                      */
/* Permission is hereby granted, free of charge, to any person          */
/* obtaining a copy of this software and associated documentation       */
/* files(the "Software"), to deal in the Software without restriction,  */
/* including without limitation the rights to use, copy, modify, merge, */
/* publish, distribute, sublicense, and/or sell copies of the Software, */
/* and to permit persons to whom the Software is furnished to do so,    */
/* subject to the following conditions:                                 */
/*                                                                      */
/* The above copyright notice and this permission notice shall be       */
/* included in all copies or substantial portions of the Software.      */
/*                                                                      */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF   */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                */
/* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS  */
/* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   */
/* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN    */
/* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     */
/* SOFTWARE.                                                            */
/*                                                                      */
/************************************************************************/

/*
 * Note: The AVR mega code has been hacked out for readability and 
 *		 because we don't need it.
 */
#include "xboot.h"

void CCPWrite( volatile uint8_t * address, uint8_t value );

#ifdef USE_INTERRUPTS
volatile unsigned char comm_mode;

volatile unsigned char rx_buff0;
volatile unsigned char rx_buff1;
volatile unsigned char rx_char_cnt;

volatile unsigned char tx_buff0;
volatile unsigned char tx_char_cnt;
#else
unsigned char comm_mode;
#endif // USE_INTERRUPTS

unsigned char buffer[SPM_PAGESIZE];

#ifdef NEED_CODE_PROTECTION
unsigned char protected;
#endif // NEED_CODE_PROTECTION

// Main code
int main(void)
{
        ADDR_T address = 0;
        unsigned char in_bootloader = 0;
        unsigned char val = 0;
        int i = 0;
        uint32_t j;
        uint8_t k;
        
        #ifdef NEED_CODE_PROTECTION
        protected = 1;
        #endif // NEED_CODE_PROTECTION
        
        #ifdef USE_I2C_ADDRESS_NEGOTIATION
        unsigned short devid_bit;
        #endif // USE_I2C_ADDRESS_NEGOTIATION
        
        comm_mode = MODE_UNDEF;
        
        #ifdef USE_INTERRUPTS
        rx_char_cnt = 0;
        tx_char_cnt = 0;
        #endif // USE_INTERRUPTS
        
        // Initialization section
        // Entry point and communication methods are initialized here
        // --------------------------------------------------
        
        #ifdef USE_32MHZ_RC
        #if (F_CPU != 32000000L)
        #error F_CPU must match oscillator setting!
        #endif // F_CPU

        // clock setup imported form tinyg/xmega/xmega_init.c
        OSC.XOSCCTRL = 0xCB;					// 12-16 MHz crystal; 0.4-16 MHz XTAL w 16K CLK startup
        OSC.CTRL = 0x08;						// enable external crystal oscillator
        while(!(OSC.STATUS & OSC_XOSCRDY_bm));	// wait for oscillator ready
        OSC.PLLCTRL = 0xC2;						// XOSC is PLL Source; 2x Factor (32 MHz sys clock)
        OSC.CTRL = 0x18;						// Enable PLL & External Oscillator
        while(!(OSC.STATUS & OSC_PLLRDY_bm));	// wait for PLL ready
        CCPWrite(&CLK.CTRL, CLK_SCLKSEL_PLL_gc);// switch to PLL clock
        OSC.CTRL &= ~OSC_RC2MEN_bm;				// disable internal 2 MHz clock

    /* Original code from xboot sources replaced by the above:
        OSC.CTRL |= OSC_RC32MEN_bm; // turn on 32 MHz oscillator
        while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { }; // wait for it to start
        CCP = CCP_IOREG_gc;
        CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
        #ifdef USE_DFLL
        DFLLRC32M.CTRL = DFLL_ENABLE_bm;
        #endif // USE_DFLL
        #else // USE_32MHZ_RC
        #if (F_CPU != 2000000L)
        #error F_CPU must match oscillator setting!
        #endif // F_CPU
        #ifdef USE_DFLL
        DFLLRC2M.CTRL = DFLL_ENABLE_bm;
        #endif // USE_DFLL
     */
        #endif // USE_32MHZ_RC
                
        // interrupts
        #ifdef NEED_INTERRUPTS
        // remap interrupts to boot section
        CCP = CCP_IOREG_gc;
        #ifdef USE_INTERRUPTS
        PMIC.CTRL = PMIC_IVSEL_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
        #else
        PMIC.CTRL = PMIC_IVSEL_bm;
        #endif // USE_INTERRUPTS
        #endif // NEED_INTERRUPTS
        
        // LED
        #ifdef USE_LED
        // Initialize LED pin
        LED_PORT.DIRSET = (1 << LED_PIN);
        #if LED_PIN_INV
        LED_PORT.OUTCLR = (1 << LED_PIN);
        #else
        LED_PORT.OUTSET = (1 << LED_PIN);
        #endif // LED_PIN_INV
        #endif // USE_LED
                
        // I2C Attach LED_PIN
        #ifdef USE_I2C_ADDRESS_NEGOTIATION
        #ifdef USE_ATTACH_LED
        // Initialize ATTACH_LED
        ATTACH_LED_PORT.DIRSET = (1 << ATTACH_LED_PIN);
        #if ATTACH_LED_INV
        ATTACH_LED_PORT.OUTSET = (1 << ATTACH_LED_PIN);
        #else
        ATTACH_LED_PORT.OUTCLR = (1 << ATTACH_LED_PIN);
        #endif // ATTACH_LED_INV
        #endif // USE_ATTACH_LED
        #endif // USE_I2C_ADDRESS_NEGOTIATION
        
        // Enter pin
        #ifdef USE_ENTER_PIN
        // Make sure it's an input
        ENTER_PORT.DIRCLR = (1 << ENTER_PIN);
        #if ENTER_PIN_PUEN
        // Enable bootloader entry pin pullup
        ENTER_PIN_CTRL = 0x18;
        #endif // ENTER_PIN_PUEN
        #endif // USE_ENTER_PIN
        
        #ifdef USE_UART
        // Initialize UART
        uart_init();

        // Initialize RX pin pull-up
        #ifdef UART_RX_PUEN
        // Enable RX pin pullup
        UART_RX_PIN_CTRL = 0x18;
        #endif // UART_RX_PUEN
        
        // Initialize UART EN pin
        #ifdef USE_UART_EN_PIN
        UART_EN_PORT.DIRSET = (1 << UART_EN_PIN);
        #if UART_EN_INV
        UART_EN_PORT.OUTSET = (1 << UART_EN_PIN);
        #else // UART_PIN_INV
        UART_EN_PORT.OUTCLR = (1 << UART_EN_PIN);
        #endif // UART_PIN_INV
        #endif // USE_UART_EN_PIN
        
        #endif // USE_UART
        
        #ifdef USE_I2C
        // Initialize I2C interface
        i2c_init();
        
        #ifdef USE_I2C_ADDRESS_NEGOTIATION
        I2C_AUTONEG_PORT.DIRCLR = (1 << I2C_AUTONEG_PIN);
        I2C_AUTONEG_PORT.OUTCLR = (1 << I2C_AUTONEG_PIN);
        #endif // USE_I2C_ADDRESS_NEGOTIATION
        
		#endif // USE_I2C
        
        #ifdef USE_FIFO
        // Initialize FIFO
        fifo_init();
        #endif // USE_FIFO
        
		//**************************************
		// one-time startup message     
		// THIS CAUSES AVRDUDE TO PUKE. TOO BAD.
		//	unsigned char startup[] = "{\"er\":{\"st\":8,\"msg\":\"Boot loader initialized\",\"val\":0}}\n";
		//	unsigned char startup[] = "{\"er\":{\"st\":8}}\n";
//			unsigned char startup[] = "{\"boot\":2}\n";
//			unsigned char startup[] = "{\"boot\":2}\n";
//			comm_mode = MODE_UART;
//			for (int i = 0; i < sizeof(startup); i++) { send_char(startup[i]);}
		//**************************************

        // --------------------------------------------------
        // End initialization section
        
        // One time trigger section
        // Triggers that are checked once, regardless of
        // whether or not USE_ENTER_DELAY is selected
        // --------------------------------------------------
        
        
        // --------------------------------------------------
        // End one time trigger section
        
#ifdef USE_ENTER_DELAY

	    k = ENTER_BLINK_COUNT*2;

		// ++++ Blink count delay hack
		// This code tests the software reset bit to determine if the bootloader was entered
		// from the application via a $boot=1 or {"boot"1} command. If so, it will remain in
		// the boot loader 20 time longer than usual, which should be about a minute.
		// Thanks to Rob Giseburt for pointing this out.
		if (RST.STATUS & RST_SRF_bm) {	// it came from a software reset
			RST.STATUS = 0xFF;			// reset all status bits (just to be sure)
			k *= 20;					// 20 times the timeout delay, which is typically 3 seconds.
		}
		// ++++ regular code resumes from here

        j = ENTER_BLINK_WAIT;
        while (!in_bootloader && k > 0)
        {
                if (j-- <= 0)
                {
                        #ifdef USE_LED
                        LED_PORT.OUTTGL = (1 << LED_PIN);
                        #endif // USE_LED
                        j = ENTER_BLINK_WAIT;
                        k--;
                }
#else // USE_ENTER_DELAY
                // Need a small delay when not running loop
                // so we don't accidentally enter the bootloader
                // on power-up with USE_ENTER_PIN selected
                asm("nop");
                asm("nop");
                asm("nop");
                asm("nop");
#endif // USE_ENTER_DELAY
                
                // Main trigger section
                // Set in_bootloader here to enter the bootloader
                // Checked when USE_ENTER_DELAY is selected
                // --------------------------------------------------
                
                #ifdef USE_ENTER_PIN
                // Check entry pin state
                if ((ENTER_PORT.IN & (1 << ENTER_PIN)) == (ENTER_PIN_STATE ? (1 << ENTER_PIN) : 0))
                        in_bootloader = 1;
                #endif // USE_ENTER_PIN
                
                #ifdef USE_ENTER_UART
                // Check for received character
                #ifdef ENTER_UART_NEED_SYNC
                if (uart_char_received() && (uart_cur_char() == CMD_SYNC))
                #else // ENTER_UART_NEED_SYNC
                if (uart_char_received())
                #endif // ENTER_UART_NEED_SYNC
                {
                        in_bootloader = 1;
                        comm_mode = MODE_UART;
                }
                
                #endif // USE_ENTER_UART
                
                #ifdef USE_ENTER_I2C
                // Check for address match condition
                if (i2c_address_match())
                {
                        in_bootloader = 1;
                        comm_mode = MODE_I2C;
                }
                #endif // USE_ENTER_I2C
                
                #ifdef USE_ENTER_FIFO
                // Check for received character
                #ifdef ENTER_FIFO_NEED_SYNC
                if (fifo_char_received() && (fifo_cur_char() == CMD_SYNC))
                #else // ENTER_FIFO_NEED_SYNC
                if (fifo_char_received())
                #endif // ENTER_FIFO_NEED_SYNC
                {
                        in_bootloader = 1;
                        comm_mode = MODE_FIFO;
                }
                
                #endif // USE_ENTER_FIFO
                
                // --------------------------------------------------
                // End main trigger section
                
                WDT_Reset();
                
#ifdef USE_ENTER_DELAY
        }
#endif // USE_ENTER_DELAY
        
        #ifdef USE_INTERRUPTS
        // Enable interrupts
        sei();
        #endif // USE_INTERRUPTS
        
        #ifdef USE_WATCHDOG
        WDT_EnableAndSetTimeout();
        #endif // USE_WATCHDOG

        // Main bootloader        
        while (in_bootloader) {
                #ifdef USE_LED
                LED_PORT.OUTTGL = (1 << LED_PIN);
                #endif // USE_LED
                
                val = get_char();
                
                #ifdef USE_WATCHDOG
                WDT_Reset();
                #endif // USE_WATCHDOG
                
                // Main bootloader parser
                // check autoincrement status
                if (val == CMD_CHECK_AUTOINCREMENT)
                {
                        // yes, it is supported
                        send_char(REPLY_YES);
                }
                // Set address
                else if (val == CMD_SET_ADDRESS)
                {
                        // Read address high then low
                        address = get_2bytes();
                        // acknowledge
                        send_char(REPLY_ACK);
                }
                // Extended address
                else if (val == CMD_SET_EXT_ADDRESS)
                {
                        // Read address high then low
                        //address = ((ADDR_T)get_char() << 16) | get_2bytes();
                        asm volatile (
                                "call get_char"    "\n\t"
                                "mov  %C0,r24"     "\n\t"
                                "call get_2bytes"  "\n\t"
                                "clr  %D0"         "\n\t"
                                : "=r" (address)
                                :
                        );
                        
                        // acknowledge
                        send_char(REPLY_ACK);
                }
                // Chip erase
                else if (val == CMD_CHIP_ERASE)
                {
                        // Erase the application section
                        Flash_EraseApplicationSection();
                        // Wait for completion

                        #ifdef USE_WATCHDOG
                        while (NVM_STATUS & NVM_NVMBUSY_bp)
                        {
                                // reset watchdog while waiting for erase completion
                                WDT_Reset();
                        }
                        #else // USE_WATCHDOG
                        SP_WaitForSPM();
                        #endif // USE_WATCHDOG
                        
                        // Erase EEPROM
                        EEPROM_erase_all();
                        
                        // turn off read protection
                        #ifdef NEED_CODE_PROTECTION
                        protected = 0;
                        #endif // NEED_CODE_PROTECTION
                        
                        // acknowledge
                        send_char(REPLY_ACK);
                }
                #ifdef ENABLE_BLOCK_SUPPORT
                // Check block load support
                else if (val == CMD_CHECK_BLOCK_SUPPORT )
                {
                        // yes, it is supported
                        send_char(REPLY_YES);
                        // Send block size (page size)
                        send_char((SPM_PAGESIZE >> 8) & 0xFF);
                        send_char(SPM_PAGESIZE & 0xFF);
                }
                // Block load
                else if (val == CMD_BLOCK_LOAD)
                {
                        // Block size
                        i = get_2bytes();
                        // Memory type
                        val = get_char();
                        // Load it
                        send_char(BlockLoad(i, val, &address));
                }
                // Block read
                else if (val == CMD_BLOCK_READ)
                {
                        // Block size
                        i = get_2bytes();
                        // Memory type
                        val = get_char();
                        // Read it
                        BlockRead(i, val, &address);
                }
                #endif // ENABLE_BLOCK_SUPPORT
                #ifdef ENABLE_FLASH_BYTE_SUPPORT
                // Read program memory byte
                else if (val == CMD_READ_BYTE)
                {
                        unsigned int w = Flash_ReadWord((address << 1));
                        
                        #ifdef ENABLE_CODE_PROTECTION
                        if (protected)
                                w = 0xffff;
                        #endif // ENABLE_CODE_PROTECTION
                        
                        send_char(w >> 8);
                        send_char(w);
                        
                        address++;
                }
                // Write program memory low byte
                else if (val == CMD_WRITE_LOW_BYTE)
                {
                        // get low byte
                        i = get_char();
                        send_char(REPLY_ACK);
                }
                // Write program memory high byte
                else if (val == CMD_WRITE_HIGH_BYTE)
                {
                        // get high byte; combine
                        i |= (get_char() << 8);
                        Flash_LoadFlashWord((address << 1), i);
                        address++;
                        send_char(REPLY_ACK);
                }
                // Write page
                else if (val == CMD_WRITE_PAGE)
                {
                        if (address >= (APP_SECTION_SIZE>>1))
                        {
                                // don't allow bootloader overwrite
                                send_char(REPLY_ERROR);
                        }
                        else
                        {
                                Flash_WriteApplicationPage( address << 1);
                                send_char(REPLY_ACK);
                        }
                }
                #endif // ENABLE_FLASH_BYTE_SUPPORT
                #ifdef ENABLE_EEPROM_BYTE_SUPPORT
                // Write EEPROM memory
                else if (val == CMD_WRITE_EEPROM_BYTE)
                {
                        EEPROM_write_byte(address, get_char());
                        address++;
                }
                // Read EEPROM memory
                else if (val == CMD_READ_EEPROM_BYTE)
                {
                        char c = EEPROM_read_byte(address);
                        
                        #ifdef ENABLE_EEPROM_PROTECTION
                        if (protected)
                                c = 0xff;
                        #endif // ENABLE_EEPROM_PROTECTION
                        
                        send_char(c);
                        address++;
                }
                #endif // ENABLE_EEPROM_BYTE_SUPPORT
                #ifdef ENABLE_LOCK_BITS

                // Write lockbits
                else if (val == CMD_WRITE_LOCK_BITS)
                {
                        SP_WriteLockBits( get_char() );
                        send_char(REPLY_ACK);
                }
                // Read lockbits
                else if (val == CMD_READ_LOCK_BITS)
                {
                        send_char(SP_ReadLockBits());
                }
                #endif // ENABLE_LOCK_BITS

                #ifdef ENABLE_FUSE_BITS
                // Read low fuse bits
                else if (val == CMD_READ_LOW_FUSE_BITS)
                {
                        send_char(SP_ReadFuseByte(0));
                }
                // Read high fuse bits
                else if (val == CMD_READ_HIGH_FUSE_BITS)
                {
                        send_char(SP_ReadFuseByte(1));
                }
                // Read extended fuse bits
                else if (val == CMD_READ_EXT_FUSE_BITS)
                {
                        send_char(SP_ReadFuseByte(2));
                }
                #endif // ENABLE_FUSE_BITS

                // Enter and leave programming mode
                else if ((val == CMD_ENTER_PROG_MODE) || (val == CMD_LEAVE_PROG_MODE))
                {
                        // just acknowledge
                        send_char(REPLY_ACK);
                }
                // Exit bootloader
                else if (val == CMD_EXIT_BOOTLOADER)
                {
                        in_bootloader = 0;
                        send_char(REPLY_ACK);
                }
                // Get programmer type
                else if (val == CMD_PROGRAMMER_TYPE)
                {
                        // serial
                        send_char('S');
                }
                // Return supported device codes
                else if (val == CMD_DEVICE_CODE)
                {
                        // send only this device
                        send_char(123); // TODO
                        // terminator
                        send_char(0);
                }
                // Set LED, clear LED, and set device type
                else if ((val == CMD_SET_LED) || (val == CMD_CLEAR_LED) || (val == CMD_SET_TYPE))
                {
                        // discard parameter
                        get_char();
                        send_char(REPLY_ACK);
                }
                // Return program identifier
                else if (val == CMD_PROGRAM_ID)
                {
                        send_char('X');
                        send_char('B');
                        send_char('o');
                        send_char('o');
                        send_char('t');
                        send_char('+');
                        send_char('+');
                }
                // Read software version
                else if (val == CMD_VERSION)
                {
                        send_char('0' + XBOOT_VERSION_MAJOR);
                        send_char('0' + XBOOT_VERSION_MINOR);
                }
                // Read signature bytes
                else if (val == CMD_READ_SIGNATURE)
                {
                        send_char(SIGNATURE_2);
                        send_char(SIGNATURE_1);
                        send_char(SIGNATURE_0);
                }
                #ifdef ENABLE_CRC_SUPPORT
                else if (val == CMD_CRC)
                {
                        uint32_t start = 0;
                        uint32_t length = 0;
                        uint16_t crc;
                        
                        val = get_char();
                        
                        switch (val)
                        {
                                case SECTION_FLASH:
                                        length = PROGMEM_SIZE;
                                        break;
                                case SECTION_APPLICATION:
                                        length = APP_SECTION_SIZE;
                                        break;
                                case SECTION_BOOT:
                                        start = BOOT_SECTION_START;
                                        length = BOOT_SECTION_SIZE;
                                        break;
                                #ifdef ENABLE_API
                                case SECTION_APP:
                                        length = XB_APP_SIZE;
                                        break;
                                case SECTION_APP_TEMP:
                                        start = XB_APP_TEMP_START;
                                        length = XB_APP_TEMP_SIZE;
                                        break;
                                #endif // ENABLE_API
                                default:
                                        send_char(REPLY_ERROR);
                                        continue;
                        }
                        
                        crc = crc16_block(start, length);
                        
                        send_char((crc >> 8) & 0xff);
                        send_char(crc & 0xff);
                }
                #endif // ENABLE_CRC_SUPPORT
                #ifdef USE_I2C
                #ifdef USE_I2C_ADDRESS_NEGOTIATION
                // Enter autonegotiate mode
                else if (val == CMD_AUTONEG_START)
                {
                        // The address autonegotiation protocol is borrowed from the
                        // OneWire address detection method.  The algorthim Utilizes
                        // one extra shared wire, pulled up by resistors just like the
                        // main I2C bus, a OneWire bus, or a wired-AND IRQ line.
                        // The protocol involves intelligently guessing all of the
                        // connected devices' 88 bit unique hardware ID numbers, stored
                        // permanently in the production signature row during manufacture
                        // (see XMega series datasheet for details)
                        #ifdef __AVR_XMEGA__
                        // k is temp
                        // devid is pointer to current bit, init to first bit
                        // of the hardware ID in the production signature row
                        devid_bit = 0x08 << 3;
                        // read first byte of hardware ID into temporary location
                        k = SP_ReadCalibrationByte(0x08);
                        
                        // main negotiation loop
                        while (1)
                        {
                                // wait for incoming data
                                while (1)
                                {
                                        // check for bit read command
                                        if (!(I2C_AUTONEG_PORT.IN & (1 << I2C_AUTONEG_PIN)))
                                        {
                                                // write current bit of hardware ID
                                                ow_slave_write_bit(k & 1);  // write bit
                                                break;
                                        }
                                        // check for I2C bus activity
                                        else if (I2C_DEVICE.SLAVE.STATUS & (TWI_SLAVE_APIF_bm | TWI_SLAVE_DIF_bm))
                                        {
                                                // grab a byte
                                                // (there will be no I2C bus activity while
                                                // the autonegotiation is taking place,
                                                // so it's OK to block)
                                                val = get_char();
                                                // Is this an address byte for me?
                                                if (val == CMD_AUTONEG_DONE)
                                                {
                                                        // If so, we're now attached, so light
                                                        // the LED and update the I2C bus
                                                        // controller accordingly
                                                        
                                                        // turn on attach LED
                                                        #ifdef USE_ATTACH_LED
                                                        #if ATTACH_LED_INV
                                                        ATTACH_LED_PORT.OUTCLR = (1 << ATTACH_LED_PIN);
                                                        #else
                                                        ATTACH_LED_PORT.OUTSET = (1 << ATTACH_LED_PIN);
                                                        #endif // ATTACH_LED_INV
                                                        #endif // USE_ATTACH_LED
                                                        
                                                        // get new address
                                                        #if I2C_AUTONEG_DIS_GC
                                                        I2C_DEVICE.SLAVE.ADDR = get_char() << 1;
                                                        #else
                                                        I2C_DEVICE.SLAVE.ADDR = (get_char() << 1) | 1;
                                                        #endif // I2C_AUTONEG_DIS_GC
                                                        
                                                        #if I2C_AUTONEG_DIS_PROMISC
                                                        // turn off promiscuous mode
                                                        I2C_DEVICE.SLAVE.CTRLA = TWI_SLAVE_ENABLE_bm;
                                                        #endif // I2C_AUTONEG_DIS_PROMISC
                                                        
                                                        // we're done here
                                                        goto autoneg_done;
                                                }
                                                // Check for sync command
                                                else if (val == CMD_SYNC)
                                                {
                                                        // break out to main bootloader on sync
                                                        goto autoneg_done;
                                                }
                                        }
                                }
                                // Already wrote normal bit, so write the inverted one
                                ow_slave_write_bit(~k & 1); // write inverted bit
                                // Now read master's guess
                                i = ow_slave_read_bit();
                                // Does the guess agree with the current bit?
                                if ((k & 1 && i) || (~k & 1 && !i))
                                {
                                        // look at next bit
                                        devid_bit++;
                                        k >>= 1;
                                        
                                        // time for next byte?
                                        if (!(devid_bit & 7))
                                        {
                                                // Out of bits?
                                                if (devid_bit > (0x15 << 3))
                                                {
                                                        // Can't break here (need to wait
                                                        // to see if the master sends along
                                                        // an address) so wrap around instead
                                                        devid_bit = 0x08 << 3;
                                                }
                                                // there are some holes in the signature row,
                                                // so skip over them
                                                if (devid_bit == (0x0E << 3))
                                                        devid_bit += 0x02 << 3;
                                                if (devid_bit == (0x11 << 3))
                                                        devid_bit += 0x01 << 3;
                                                // Read next byte
                                                k = SP_ReadCalibrationByte(devid_bit >> 3);
                                        }
                                }
                                else
                                {
                                        // No match, we're done here
                                        break;
                                }
                        }
                        
autoneg_done:
                        // dummy to avoid error message
                        // this actually produces code 4 bytes smaller than either
                        // an asm nop, a continue, or a bare semicolon
                        i = 0;
                        
                        #endif // __AVR_XMEGA__
                }
                // out-of-order autonegotiate address message
                else if (val == CMD_AUTONEG_DONE)
                {
                        // ignore it
                        // (blocking to send a ? will cause trouble)
                }
                #endif // USE_I2C_ADDRESS_NEGOTIATION
                #endif // USE_I2C
                // ESC (0x1b) to sync
                // otherwise, error
                else if (val != CMD_SYNC)
                {
                        send_char(REPLY_ERROR);
                }
                
                // Wait for any lingering SPM instructions to finish
                Flash_WaitForSPM();
                
                // End of bootloader main loop
        }
        
        #ifdef NEED_INTERRUPTS
        // Disable interrupts
        cli();
        #endif // NEED_INTERRUPTS
        
        // Bootloader exit section
        // Code here runs after the bootloader has exited,
        // but before the application code has started
        // --------------------------------------------------
        
        #ifdef ENABLE_API
        #ifdef ENABLE_API_FIRMWARE_UPDATE
        // Update firmware if needed
        install_firmware();
        #endif // ENABLE_API_FIRMWARE_UPDATE
        #endif // ENABLE_API
        
        #ifdef USE_FIFO
        // Shut down FIFO
        fifo_deinit();
        #endif // USE_FIFO
        
        #ifdef USE_I2C
        // Shut down I2C interface
        i2c_deinit();
        #endif // USE_I2C
        
        #ifdef USE_UART
        // Shut down UART
        uart_deinit();
        
        // Disable RX pin pull-up
        #ifdef UART_RX_PUEN
        // Disable RX pin pullup
        UART_RX_PIN_CTRL = 0;
        #endif // UART_RX_PUEN
        
        // Shut down UART EN pin
        #ifdef USE_UART_EN_PIN
        UART_EN_PORT.DIRCLR = (1 << UART_EN_PIN);
        UART_EN_PORT.OUTCLR = (1 << UART_EN_PIN);
        #endif // USE_UART_EN_PIN
        #endif // USE_UART
        
        #ifdef LOCK_SPM_ON_EXIT
        // Lock SPM writes
        SP_LockSPM();
        #endif // LOCK_SPM_ON_EXIT
        
        // Disable bootloader entry pin
        #ifdef USE_ENTER_PIN
        #if ENTER_PIN_PUEN
        // Disable bootloader entry pin pullup
        ENTER_PIN_CTRL = 0;
        #endif // ENTER_PIN_PUEN
        #endif // USE_ENTER_PIN
        
        // LED
        #ifdef USE_LED
        // Turn off LED on exit
        LED_PORT.DIRCLR = (1 << LED_PIN);
        LED_PORT.OUTCLR = (1 << LED_PIN);
        #endif // USE_LED
        
        // Attach LED
        #ifdef USE_I2C_ADDRESS_NEGOTIATION
        #ifdef USE_ATTACH_LED
        // Disable ATTACH_LED
        ATTACH_LED_PORT.DIRCLR = (1 << ATTACH_LED_PIN);
        ATTACH_LED_PORT.OUTCLR = (1 << ATTACH_LED_PIN);
        #endif // USE_ATTACH_LED
        #endif // USE_I2C_ADDRESS_NEGOTIATION
        
        #ifdef NEED_INTERRUPTS
        // remap interrupts back to application section
        CCP = CCP_IOREG_gc;
        PMIC.CTRL = 0;
        #endif // NEED_INTERRUPTS
        
        #ifdef USE_WATCHDOG
        WDT_Disable();
        #endif // USE_WATCHDOG
        
        // --------------------------------------------------
        // End bootloader exit section
        
        // Jump into main code
        asm("jmp 0");
        
        #ifdef __builtin_unreachable
        // Size optimization as the asm jmp will not return
        // However, it seems it is not available on older versions of gcc
        __builtin_unreachable();
        #endif
}

#ifdef USE_I2C_ADDRESS_NEGOTIATION

#define ow_assert()             I2C_AUTONEG_PORT.DIRSET = (1 << I2C_AUTONEG_PIN)
#define ow_deassert()           I2C_AUTONEG_PORT.DIRCLR = (1 << I2C_AUTONEG_PIN)
#define ow_read()               (I2C_AUTONEG_PORT.IN & (1 << I2C_AUTONEG_PIN))
#define ow_is_asserted()        (!ow_read())

unsigned char __attribute__ ((noinline)) ow_slave_read_bit(void)
{
        unsigned char ret;
        ow_slave_wait_bit();
        _delay_us(12);
        ret = ow_read();
        _delay_us(8);
        return ret;
}

void __attribute__ ((noinline)) ow_slave_write_bit(unsigned char b)
{
        ow_slave_wait_bit();
        if (!b)
        {
                ow_assert();
        }
        _delay_us(20);
        ow_deassert();
}

void ow_slave_wait_bit(void)
{
        while (ow_read()) { };
}

#endif // USE_I2C_ADDRESS_NEGOTIATION

#ifdef USE_INTERRUPTS
unsigned char __attribute__ ((noinline)) get_char(void)
{
        unsigned char ret;
        
        while (rx_char_cnt == 0) { };
        
        cli();
        
        ret = rx_buff0;
        rx_buff0 = rx_buff1;
        rx_char_cnt--;
        
        sei();
        
        return ret;
}

void __attribute__ ((noinline)) send_char(unsigned char c)
{
        while (1)
        {
                cli();
                
                if (tx_char_cnt == 0)
                {
                        tx_buff0 = c;
                        tx_char_cnt = 1;
                        
                        #ifdef USE_UART
                        if (comm_mode == MODE_UART)
                        {
                                uart_send_char(c);
                        }
                        #endif // USE_UART
                        
                        #ifdef USE_I2C
                        #error I2C interrupts are not yet implemented
                        #endif
                        
                        #ifdef USE_FIFO
                        if (comm_mode == MODE_FIFO)
                        {
                                fifo_send_char(c);
                        }
                        #endif // USE_FIFO
                        
                        sei();
                        return;
                }
                
                sei();
        }
}

#else

unsigned char __attribute__ ((noinline)) get_char(void)
{
        unsigned char ret;
        
        while (1)
        {
                #ifdef USE_UART
                // Get next character
                if (comm_mode == MODE_UNDEF || comm_mode == MODE_UART)
                {
                        if (uart_char_received())
                        {
                                comm_mode = MODE_UART;
                                return uart_cur_char();
                        }
                }
                #endif // USE_UART
                
                #ifdef USE_I2C
                // Get next character
                if (comm_mode == MODE_UNDEF || comm_mode == MODE_I2C)
                {
                        if (i2c_address_match())
                        {
                                // Address match, send ACK
                                i2c_send_ack();
                                comm_mode = MODE_I2C;
                                first_byte = 1;
                        }
                        if (i2c_char_received())
                        {
                                // Data has arrived
                                ret = i2c_cur_char();
                                i2c_send_ack();
                                return ret;
                        }
                        if (i2c_ready_data())
                        {
                                if (!first_byte && i2c_got_ack())
                                {
                                        i2c_end_transmission(); // end transaction
                                }
                                else
                                {
                                        first_byte = 0;
                                        // Wants data, but there is no data to send...
                                        // also include NAK
                                        i2c_send_char(REPLY_ERROR);
                                        i2c_send_nak();
                                }
                        }
                }
                #endif // USE_I2C

                #ifdef USE_FIFO
                // Get next character
                if (comm_mode == MODE_UNDEF || comm_mode == MODE_FIFO)
                {
                        if (fifo_char_received())
                        {
                                comm_mode = MODE_FIFO;
                                return fifo_cur_char();
                        }
                }
                #endif // USE_FIFO
                
        }
        
        return ret;
}

void __attribute__ ((noinline)) send_char(unsigned char c)
{
        #ifdef USE_I2C
        unsigned char tmp;
        #endif
        
        #ifdef USE_UART
        // Send character
        if (comm_mode == MODE_UNDEF || comm_mode == MODE_UART)
        {
                #ifdef USE_UART_EN_PIN
                #if UART_EN_INV
                UART_EN_PORT.OUTCLR = (1 << UART_EN_PIN);
                #else // UART_PIN_INV
                UART_EN_PORT.OUTSET = (1 << UART_EN_PIN);
                #endif // UART_PIN_INV
                #endif // USE_UART_EN_PIN
                uart_send_char_blocking(c);
                #ifdef USE_UART_EN_PIN
                #if UART_EN_INV
                UART_EN_PORT.OUTSET = (1 << UART_EN_PIN);
                #else // UART_PIN_INV
                UART_EN_PORT.OUTCLR = (1 << UART_EN_PIN);
                #endif // UART_PIN_INV
                #endif // USE_UART_EN_PIN
        }
        #endif // USE_UART
        
        #ifdef USE_I2C
        // Send character
        if (comm_mode == MODE_UNDEF || comm_mode == MODE_I2C)
        {
                while (1)
                {
                        if (i2c_address_match())
                        {
                                // Address match, send ACK
                                i2c_send_ack();
                                first_byte = 1;
                        }
                        if (i2c_char_received())
                        {
                                // Data has arrived, ignore it
                                tmp = i2c_cur_char();
                                i2c_send_ack();
                        }
                        if (i2c_ready_data())
                        {
                                if (!first_byte && i2c_got_ack())
                                {
                                        i2c_end_transmission(); // end transaction
                                }
                                else
                                {
                                        first_byte = 0;
                                        // Send data along
                                        i2c_send_char(c);
                                        i2c_send_ack();
                                }
                                return;
                        }
                }
        }
        #endif // USE_I2C

        #ifdef USE_FIFO
        // Send character
        if (comm_mode == MODE_UNDEF || comm_mode == MODE_FIFO)
        {
                fifo_send_char_blocking(c);
                
        }
        #endif // USE_FIFO
        
}

#endif // USE_INTERRUPTS

unsigned int __attribute__ ((noinline)) get_2bytes()
{
        // return (get_char() << 8) | get_char();
        unsigned int result;
        asm volatile (
                "call get_char"    "\n\t"
                "mov  %B0,r24"     "\n\t"
                "call get_char"    "\n\t"
                "mov  %A0,r24"     "\n\t"
                : "=r" (result)
                :
        );
        return result;
}

void clear_buffer(void)
{
        unsigned char *ptr = buffer;
        for (long i = 0; i < SPM_PAGESIZE; i++)
        {
                *(ptr++) = 0xff;
        }
}

unsigned char BlockLoad(unsigned int size, unsigned char mem, ADDR_T *address)
{
        ADDR_T tempaddress;
        
        #ifdef USE_WATCHDOG
        WDT_Reset();
        #endif // USE_WATCHDOG
        
        // fill up buffer
        for (int i = 0; i < SPM_PAGESIZE; i++)
        {
                char c = 0xff;
                
                if (i < size)
                        c = get_char();
                
                buffer[i] = c;
        }
        
        // EEPROM memory type.
        if(mem == MEM_EEPROM)
        {
                EEPROM_write_block(*address, buffer, size);
                (*address) += size;
                
                return REPLY_ACK; // Report programming OK
        } 
        
        // Flash memory type
        else if (mem == MEM_FLASH || mem == MEM_USERSIG)
        {
                // NOTE: For flash programming, 'address' is given in words.
                tempaddress = (*address) << 1;  // Store address in page.
                
                (*address) += size >> 1;
                                
                if (mem == MEM_FLASH)
                {
                        #ifdef ENABLE_FLASH_ERASE_WRITE
                        Flash_ProgramPage(tempaddress, buffer, 1);
                        #else
                        Flash_ProgramPage(tempaddress, buffer, 0);
                        #endif
                }
                else if (mem == MEM_USERSIG)
                {
                        Flash_LoadFlashPage(buffer);
                        Flash_EraseUserSignatureRow();
                        Flash_WaitForSPM();
                        Flash_WriteUserSignatureRow();
                        Flash_WaitForSPM();
                }                
                return REPLY_ACK; // Report programming OK
        }
        
        // Invalid memory type?
        else
        {
                return REPLY_ERROR;
        }
}



void BlockRead(unsigned int size, unsigned char mem, ADDR_T *address)
{
        int offset = 0;
        int size2 = size;
        
        // EEPROM memory type.
        
        if (mem == MEM_EEPROM) // Read EEPROM
        {
                EEPROM_read_block(*address, buffer, size);
                (*address) += size;
        }
        
        // Flash memory type.
        else if (mem == MEM_FLASH || mem == MEM_USERSIG || mem == MEM_PRODSIG)
        {
                (*address) <<= 1; // Convert address to bytes temporarily.
                
                do
                {
                        if (mem == MEM_FLASH)
                        {
                                buffer[offset++] = Flash_ReadByte(*address);
                        }
                        else if (mem == MEM_USERSIG)
                        {
                                buffer[offset++] = SP_ReadUserSignatureByte(*address);
                        }
                        else if (mem == MEM_PRODSIG)
                        {
                                buffer[offset++] = SP_ReadCalibrationByte(*address);
                        }
                        Flash_WaitForSPM();
                        
                        (*address)++;    // Select next word in memory.
                        size--;          // Subtract two bytes from number of bytes to read
                } while (size);         // Repeat until all block has been read
                
                (*address) >>= 1;       // Convert address back to Flash words again.
        }
        else
        {
                // bad memory type
                return;
        }
        
        // code protection
        if (
        #ifdef ENABLE_CODE_PROTECTION
                (protected && mem == MEM_FLASH) ||
        #endif // ENABLE_CODE_PROTECTION
        #ifdef ENABLE_EEPROM_PROTECTION
                (protected && mem == MEM_EEPROM) ||
        #endif // ENABLE_EEPROM_PROTECTION
        #ifdef ENABLE_BOOTLOADER_PROTECTION
                (*address >= (BOOT_SECTION_START >> 1) && mem == MEM_FLASH) ||
        #endif // ENABLE_BOOTLOADER_PROTECTION
                0
        )
                clear_buffer();
        
        // send bytes
        for (int i = 0; i < size2; i++)
        {
                send_char(buffer[i]);
        }
        
}

uint16_t crc16_block(uint32_t start, uint32_t length)
{
        uint16_t crc = 0;
        
        int bc = SPM_PAGESIZE;
        
        for ( ; length > 0; length--)
        {
                if (bc == SPM_PAGESIZE)
                {
                        Flash_ReadFlashPage(buffer, start);
                        start += SPM_PAGESIZE;
                        bc = 0;
                }
                
                crc = _crc16_update(crc, buffer[bc]);
                
                bc++;
        }
        
        return crc;
}

void install_firmware()
{
        uint16_t crc;
        uint16_t crc2;
        
        // read last block
        Flash_ReadFlashPage(buffer, XB_APP_TEMP_START + XB_APP_TEMP_SIZE - SPM_PAGESIZE);
        
        // check for install command
        if (buffer[SPM_PAGESIZE-6] == 'X' && buffer[SPM_PAGESIZE-5] == 'B' &&
                buffer[SPM_PAGESIZE-4] == 'I' && buffer[SPM_PAGESIZE-3] == 'F')
        {
                crc = (buffer[SPM_PAGESIZE-2] << 8) | buffer[SPM_PAGESIZE-1];
                
                // skip last 6 bytes as they are the install command
                crc2 = crc16_block(XB_APP_TEMP_START, XB_APP_TEMP_SIZE - 6);
                
                // crc last 6 bytes as empty
                for (int i = 0; i < 6; i++)
                        crc2 = _crc16_update(crc2, 0xff);
                
                if (crc == crc2)
                {
                        for (uint32_t ptr = 0; ptr < XB_APP_SIZE; ptr += SPM_PAGESIZE)
                        {
                                #ifdef USE_LED
                                LED_PORT.OUTTGL = (1 << LED_PIN);
                                #endif // USE_LED
                                
                                Flash_ReadFlashPage(buffer, ptr + XB_APP_TEMP_START);
                                // if it's the last page, clear out the last 6 bytes
                                if (ptr >= XB_APP_SIZE - SPM_PAGESIZE)
                                {
                                        for (int i = SPM_PAGESIZE-6; i < SPM_PAGESIZE; i++)
                                                buffer[i] = 0xff;
                                }
                                Flash_ProgramPage(ptr, buffer, 1);
                        }
                }
                
                xboot_app_temp_erase();
        }
}


/****************************************************************************/
/* Macros to protect the code from interrupts */

#define AVR_ENTER_CRITICAL_REGION() uint8_t volatile saved_sreg = SREG; cli();
#define AVR_LEAVE_CRITICAL_REGION() SREG = saved_sreg;


/* CCP write helper function written in assembly.
 *
 *  This function is written in assembly because of the time critial
 *  operation of writing to the registers.
 *
 *  - address A pointer to the address to write to.
 *  - value   The value to put in to the register.
 */

void CCPWrite( volatile uint8_t * address, uint8_t value )
{
#ifdef __ICCAVR__

	// Store global interrupt setting in scratch register and disable interrupts.
        asm("in  R1, 0x3F \n"
	    "cli"
	    );

	// Move destination address pointer to Z pointer registers.
	asm("movw r30, r16");
#ifdef RAMPZ
	asm("ldi  R16, 0 \n"
            "out  0x3B, R16"
	    );

#endif
	asm("ldi  r16,  0xD8 \n"
	    "out  0x34, r16  \n"
#if (__MEMORY_MODEL__ == 1)
	    "st     Z,  r17  \n");
#elif (__MEMORY_MODEL__ == 2)
	    "st     Z,  r18  \n");
#else /* (__MEMORY_MODEL__ == 3) || (__MEMORY_MODEL__ == 5) */
	    "st     Z,  r19  \n");
#endif /* __MEMORY_MODEL__ */

	// Restore global interrupt setting from scratch register.
        asm("out  0x3F, R1");

#elif defined __GNUC__
	AVR_ENTER_CRITICAL_REGION();
	volatile uint8_t * tmpAddr = address;
#ifdef RAMPZ
	RAMPZ = 0;
#endif
	asm volatile(
		"movw r30,  %0"	      "\n\t"
		"ldi  r16,  %2"	      "\n\t"
		"out   %3, r16"	      "\n\t"
		"st     Z,  %1"       "\n\t"
		:
		: "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)
		: "r16", "r30", "r31"
		);

	AVR_LEAVE_CRITICAL_REGION();
#endif
}

