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

#include "xboot.h"

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

// Main code
int main(void)
{
        ADDR_T address = 0;
        unsigned char in_bootloader = 0;
        unsigned char val = 0;
        int i = 0;
		int j, k;
        void (*reset_vect)( void ) = 0x000000;
        
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
        #endif
        #ifdef __AVR_XMEGA__
        OSC.CTRL |= OSC_RC32MEN_bm; // turn on 32 MHz oscillator
        while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { }; // wait for it to start
        CCP = CCP_IOREG_gc;
        CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
        #ifdef USE_DFLL
        DFLLRC32M.CTRL = DFLL_ENABLE_bm;
        #endif // USE_DFLL
        #endif // __AVR_XMEGA__
        #else
        #if (F_CPU != 2000000L)
        #error F_CPU must match oscillator setting!
        #endif
        #ifdef __AVR_XMEGA__
        #ifdef USE_DFLL
        DFLLRC2M.CTRL = DFLL_ENABLE_bm;
        #endif // USE_DFLL
        #endif // __AVR_XMEGA__
        #endif
        
        #ifdef NEED_INTERRUPTS
        // remap interrupts to boot section
        CCP = CCP_IOREG_gc;
        #ifdef USE_INTERRUPTS
        PMIC.CTRL = PMIC_IVSEL_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
        #else
        PMIC.CTRL = PMIC_IVSEL_bm;
        #endif // USE_INTERRUPTS
        #endif // NEED_INTERRUPTS
        
        #ifdef USE_LED
        // Initialize LED pin
        LED_PORT.DIRSET = (1 << LED_PIN);
        #if LED_PIN_INV
        LED_PORT.OUTCLR = (1 << LED_PIN);
        #else
        LED_PORT.OUTSET = (1 << LED_PIN);
        #endif // LED_PIN_INV
        #endif // USE_LED
        
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
                #ifdef __AVR_XMEGA__
                #ifdef ENTER_UART_NEED_SYNC
                if (uart_char_received() && (uart_cur_char() == CMD_SYNC))
                #else // ENTER_UART_NEED_SYNC
                if (uart_char_received())
                #endif // ENTER_UART_NEED_SYNC
                {
                        in_bootloader = 1;
                        comm_mode = MODE_UART;
                }
                #endif // __AVR_XMEGA__
                
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
                #ifdef __AVR_XMEGA__
                #ifdef ENTER_FIFO_NEED_SYNC
                if (fifo_char_received() && (fifo_cur_char() == CMD_SYNC))
                #else // ENTER_FIFO_NEED_SYNC
                if (fifo_char_received())
                #endif // ENTER_FIFO_NEED_SYNC
                {
                        in_bootloader = 1;
                        comm_mode = MODE_FIFO;
                }
                #endif // __AVR_XMEGA__
                
                #endif // USE_ENTER_FIFO
                 
                // --------------------------------------------------
                // End main trigger section
                
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
                        SP_EraseApplicationSection();
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
                        
                        // Randomize page buffer
                        EEPROM_LoadPage(&val);
                        // Erase EEPROM
                        EEPROM_EraseAll();
                        
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
                        send_char((APP_SECTION_PAGE_SIZE >> 8) & 0xFF);
                        send_char(APP_SECTION_PAGE_SIZE & 0xFF);
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
                        send_char(SP_ReadByte((address << 1)+1));
                        send_char(SP_ReadByte((address << 1)+0));
                        
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
                        SP_LoadFlashWord((address << 1), i);
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
                                SP_WriteApplicationPage( address << 1);
                                send_char(REPLY_ACK);
                        }
                }
                #endif // ENABLE_FLASH_BYTE_SUPPORT
                #ifdef ENABLE_EEPROM_BYTE_SUPPORT
                // Write EEPROM memory
                else if (val == CMD_WRITE_EEPROM_BYTE)
                {
                        EEPROM_WriteByte( (unsigned char)(address / EEPROM_PAGE_SIZE), (unsigned char) (address & EEPROM_BYTE_ADDRESS_MASK), get_char() );
                        address++;
                }
                // Read EEPROM memory
                else if (val == CMD_READ_EEPROM_BYTE)
                {
                        send_char( EEPROM_ReadByte( (unsigned char)(address / EEPROM_PAGE_SIZE), (unsigned char) (address & EEPROM_BYTE_ADDRESS_MASK) ) );
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
                        send_char('1');
                        send_char('6');
                }
                // Read signature bytes
                else if (val == CMD_READ_SIGNATURE)
                {
                        send_char(SIGNATURE_2);
                        send_char(SIGNATURE_1);
                        send_char(SIGNATURE_0);
                }
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
                                                k = SP_ReadCalibrationByte(k >> 3);
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
                SP_WaitForSPM();
                
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
        #endif // USE_UART
        
        #ifdef LOCK_SPM_ON_EXIT
        // Lock SPM writes
        SP_LockSPM();
        #endif // LOCK_SPM_ON_EXIT
        
        #ifdef USE_LED
        // Turn off LED on exit
        LED_PORT.DIRCLR = (1 << LED_PIN);
        #endif // USE_LED
        
        #ifdef USE_I2C_ADDRESS_NEGOTIATION
        #ifdef USE_ATTACH_LED
        // Disable ATTACH_LED
        ATTACH_LED_PORT.DIRSET = (1 << ATTACH_LED_PIN);
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
        EIND = 0x00;
        reset_vect();
}

#ifdef USE_I2C_ADDRESS_NEGOTIATION

#ifdef __AVR_XMEGA__
#define ow_assert()             I2C_AUTONEG_PORT.DIRSET = (1 << I2C_AUTONEG_PIN)
#define ow_deassert()           I2C_AUTONEG_PORT.DIRCLR = (1 << I2C_AUTONEG_PIN)
#define ow_read()               (I2C_AUTONEG_PORT.IN & (1 << I2C_AUTONEG_PIN))
#define ow_is_asserted()        (!ow_read())
#else
#define ow_assert()             DDRC |= (1 << 0)
#define ow_deassert()           DDRC &= ~(1 << 0)
#define ow_read()               (PINC & (1 << 0))
#define ow_is_asserted()        (!ow_read())
#endif // __AVR_XMEGA__

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
                        #ifdef __AVR_XMEGA__
                        if (uart_char_received())
                        {
                                comm_mode = MODE_UART;
                                return uart_cur_char();
                        }
                        #endif // __AVR_XMEGA__
                }
                #endif // USE_UART
                
                #ifdef USE_I2C
                // Get next character
                if (comm_mode == MODE_UNDEF || comm_mode == MODE_I2C)
                {
                        #ifdef __AVR_XMEGA__
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
                        #endif // __AVR_XMEGA__
                }
                #endif // USE_I2C

                #ifdef USE_FIFO
                // Get next character
                if (comm_mode == MODE_UNDEF || comm_mode == MODE_FIFO)
                {
                        #ifdef __AVR_XMEGA__
                        if (fifo_char_received())
                        {
                                comm_mode = MODE_FIFO;
                                return fifo_cur_char();
                        }
                        #endif // __AVR_XMEGA__
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
                #ifdef __AVR_XMEGA__
                uart_send_char_blocking(c);
                #endif // __AVR_XMEGA__
                
        }
        #endif // USE_UART
        
        #ifdef USE_I2C
        // Send character
        if (comm_mode == MODE_UNDEF || comm_mode == MODE_I2C)
        {
                while (1)
                {
                        #ifdef __AVR_XMEGA__
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
                        #endif // __AVR_XMEGA__
                }
        }
        #endif // USE_I2C

        #ifdef USE_FIFO
        // Send character
        if (comm_mode == MODE_UNDEF || comm_mode == MODE_FIFO)
        {
                #ifdef __AVR_XMEGA__
                fifo_send_char_blocking(c);
                #endif // __AVR_XMEGA__
                
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

unsigned char BlockLoad(unsigned int size, unsigned char mem, ADDR_T *address)
{
        unsigned int data;
        ADDR_T tempaddress;
        
	#ifdef USE_WATCHDOG
	WDT_Reset();
	#endif // USE_WATCHDOG

        // EEPROM memory type.
        if(mem == MEM_EEPROM)
        {
                unsigned char pageAddr, byteAddr, value;
                unsigned char buffer[APP_SECTION_PAGE_SIZE];
                
                EEPROM_FlushBuffer();
                // disable mapping of EEPROM into data space (enable IO mapped access)
                EEPROM_DisableMapping();
                
                // Fill buffer first, as EEPROM is too slow to copy with UART speed 
                for(tempaddress=0;tempaddress<size;tempaddress++){
                        buffer[tempaddress] = get_char();
                }
                
                // Then program the EEPROM
                for( tempaddress=0; tempaddress < size; tempaddress++)
                {
                        // void EEPROM_WriteByte( uint8_t pageAddr, uint8_t byteAddr, uint8_t value )
                        pageAddr = (unsigned char)( (*address) / EEPROM_PAGE_SIZE);
                        byteAddr = (unsigned char)( (*address) & EEPROM_BYTE_ADDRESS_MASK);
                        value = buffer[tempaddress];
                        
                        EEPROM_WriteByte(pageAddr, byteAddr, value);
                        
                        (*address)++; // Select next EEPROM byte
                }
                
                return REPLY_ACK; // Report programming OK
        } 
        
        // Flash memory type
        else if (mem == MEM_FLASH || mem == MEM_USERSIG)
        {
                // NOTE: For flash programming, 'address' is given in words.
                (*address) <<= 1; // Convert address to bytes temporarily.
                tempaddress = (*address);  // Store address in page.
                
                do
                {
                        data = get_char();
                        data |= (get_char() << 8);
                        SP_LoadFlashWord(*address, data);
                        (*address)+=2; // Select next word in memory.
                        size -= 2; // Reduce number of bytes to write by two.
                } while(size); // Loop until all bytes written.
                
                if (mem == MEM_FLASH)
                {
                        #ifdef ENABLE_FLASH_ERASE_WRITE
                        SP_EraseWriteApplicationPage(tempaddress);
                        #else
                        SP_WriteApplicationPage(tempaddress);
                        #endif
                }
                else if (mem == MEM_USERSIG)
                {
                        SP_EraseUserSignatureRow();
                        SP_WaitForSPM();
                        SP_WriteUserSignatureRow();
                }
                
                SP_WaitForSPM();
                
                (*address) >>= 1; // Convert address back to Flash words again.
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
        // EEPROM memory type.
        
        if (mem == MEM_EEPROM) // Read EEPROM
        {
                unsigned char byteAddr, pageAddr;
                
                EEPROM_DisableMapping();
                EEPROM_FlushBuffer();
                
                do
                {
                        pageAddr = (unsigned char)(*address / EEPROM_PAGE_SIZE);
                        byteAddr = (unsigned char)(*address & EEPROM_BYTE_ADDRESS_MASK);
                        
                        send_char( EEPROM_ReadByte( pageAddr, byteAddr ) );
                        // Select next EEPROM byte
                        (*address)++;
                        size--; // Decrease number of bytes to read
                } while (size); // Repeat until all block has been read
        }
        
        // Flash memory type.
        else if (mem == MEM_FLASH || mem == MEM_USERSIG || mem == MEM_PRODSIG)
        {
                (*address) <<= 1; // Convert address to bytes temporarily.
                
                do
                {
                        if (mem == MEM_FLASH)
                        {
                                send_char( SP_ReadByte( *address) );
                                send_char( SP_ReadByte( (*address)+1) );
                        }
                        else if (mem == MEM_USERSIG)
                        {
                                send_char( SP_ReadUserSignatureByte( *address) );
                                send_char( SP_ReadUserSignatureByte( (*address)+1) );
                        }
                        else if (mem == MEM_PRODSIG)
                        {
                                send_char( SP_ReadCalibrationByte( *address) );
                                send_char( SP_ReadCalibrationByte( (*address)+1) );
                        }
                        
                        SP_WaitForSPM();
                        
                        (*address) += 2;    // Select next word in memory.
                        size -= 2;          // Subtract two bytes from number of bytes to read
                } while (size);         // Repeat until all block has been read
                
                (*address) >>= 1;       // Convert address back to Flash words again.
        }
}


