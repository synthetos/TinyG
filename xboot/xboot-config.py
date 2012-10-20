import os.path
import shutil
import datetime
import string
import sys

# This function writes a parameter to file, but comments it out if the variable passed says it should be undefined.
def write_param(f, var, param):
	if var == "yes":
		f.write(param + "\r\n")
	else:
		f.write("//" + param + "\r\n")

# Initialize all variables with some default values.
watchdog_timeout = "WDT_PER_1KCLK_gc"
use_uart = "PORTC"
use_enter_delay = ""
enter_blink_count = "3"
enter_blink_wait = "30000"
enter_port = "PORTC"
enter_pin = "0"
enter_pin_state = "1"
enter_pin_puen = "0"
use_enter_i2c = ""
use_enter_fifo = ""
use_fifo = ""
use_i2c = ""
use_i2c_address_negotiation = ""
use_attach_led = ""
attach_led_port = "PORTC"
attach_led_pin = "0"
attach_led_inv = "0"
use_enter_fifo = ""
use_fifo = ""
led_port = "PORTC"
led_pin = "0"
led_inv = "0"
uart_baud_rate = "38400"
uart_port_name = "C"
uart_number = "0"
i2c_device_port = "E"
i2c_match_any = "0"
i2c_address = "10"
i2c_gc_enable = "0"
i2c_autoneg_dis_promisc = "1"
i2c_autoneg_dis_gc = "0"
i2c_autoneg_port = "PORTC"
i2c_autoneg_pin = "0"




print "\r\n\r\n*** XBoot Configuration Script ***\r\n"
print "Thanks for using the XBoot configuration script.  You will now be guided through the configuration process."
print "For yes/no questions, please answer by typing either 'yes' or 'no'.  For all other questions, answer with "
print "one of the prescribed options.\r\n"

print "\r\nSection 1: MCU Config"

print "\r\nWhich AVR would you like to configure XBoot for?  List provides known good controllers - use avr-gcc standard MCU names if not on list."
print "[atxmega16a4 atxmega32a4 atxmega64a1 atxmega64a3 atxmega64a4 atxmega128a1 atxmega128a3 atxmega128a4 atxmega192a1 atxmega192a3 atxmega256a1 atxmega256a3 atxmega256a3b]"
mcu = raw_input(">:")

print "\r\nClock Speed:"
print "1) 2MHz"
print "2) 32MHz Ring Oscillator"
f_cpu = raw_input(">:")
if f_cpu == "1":
	f_cpu = "2000000L"
else:
	f_cpu = "32000000L"

use_interrupts = raw_input("\r\nUse interrupts? (Usually not necessary.)  [yes/no]\r\n>:")
use_watchdog = raw_input("Enable watchdog? [yes/no]\r\n>:")
if use_watchdog == "yes":
	print "Please select a watchdog prescaler from the following options:"
	print "[WDT_PER_8CLK_gc WDT_PER_16CLK_gc WDT_PER_32CLK_gc WDT_PER_64CLK_gc WDT_PER_128CLK_gc WDT_PER_256CLK_gc"
	print " WDT_PER_512CLK_gc WDT_PER_1KCLK_gc WDT_PER_2KCLK_gc WDT_PER_4KCLK_gc WDT_PER_8KCLK_gc]"
	watchdog_timeout = raw_input(">:")

print "\r\nSection 2: Bootloader Entrance\r\n"

use_enter_uart = raw_input("Enter bootloader if a character is received via UART? [yes/no]\r\n>:")

if use_enter_uart == "yes":
	use_uart = "yes"
	use_enter_delay = "yes"
else:
	use_enter_delay = raw_input("Should there be a delay on MCU reset?  This acts as a window to detect bootloader entrance conditions. [yes/no]\r\n>:")

if use_enter_delay == "yes":
	enter_blink_wait = raw_input("   There will be a delay used when the AVR is reset.  How long should the loop be? (0-65535, a good answer is 30000)\r\n   >:")
	enter_blink_count = raw_input("   How many times should this loop be called? (The LED, if enabled, blinks once per loop, a good answer is 3)\r\n   >:")

use_enter_pin = raw_input("\r\nEnter bootloader if a pin is held in a high/low state on MCU reset? [yes/no]\r\n>:")

if use_enter_pin == "yes":
	enter_port = raw_input("   What port will be used? (Ex: PORTA, PORTC, etc.)\r\n   >:")
	enter_pin = raw_input("   What pin on " + enter_port + " will be used? (Enter number only)\r\n   >:")
	enter_pin_state = raw_input("   What state should the pin be in to trigger bootloader entry? (Enter '1' for High, or '0' for Low)\r\n   >:")
	enter_pin_puen = raw_input("   Should this pin have the internal pullup enabled? ('0' = No, '1' = Yes)\r\n   >:")

if use_enter_uart != "yes":
	use_enter_i2c = raw_input("\r\nEnter bootloader if a character is received via I2C? [yes/no]\r\n>:")

	if use_enter_i2c == "yes":
		use_i2c = "yes"
		use_i2c_address_negotiation = raw_input("   Should I2C address negotiation be used? [yes/no]\r\n   >:")
		use_attach_led = raw_input("   Should an LED be used to show when MCU is attached, i.e. receives a new I2C address? [yes/no]\r\n   >:")
		if use_attach_led == "yes":
			use_attach_led = raw_input("      Enter the attach LED port name. (Full port name, e.g. 'PORTA')\r\n      >:")
			attach_led_pin = raw_input("      Enter the attach LED pin number. (Single number, e.g. '0' or '5')\r\n      >:")
			attach_led_inv = raw_input("      Should the attach LED logic be inverted? [yes/no]      >:")

print "\r\nSection 3: Communication Configuration"

use_led = raw_input("\r\nShould an LED be used by the bootloader to show progress? [yes/no]\r\n>:")

if use_led == "yes":
	print "   You have opted to use an LED."
	led_port = raw_input("   Enter LED port name. (Full port name, e.g. 'PORTA' or 'PORTC')\r\n   >:")
	led_pin = raw_input("   Enter LED pin number. (Single digit, e.g. '0' or '5', etc.)\r\n   >:")
	led_inv = raw_input("   Should the LED logic be inverted? [yes/no]\r\n   >:")

if use_enter_uart == "yes":
	print "\r\nYou have opted to use a UART for communications."
	uart_baud_rate = raw_input("   Enter baud rate. (9600, 19200, 38400, 57600, 115200 are standard - others might not work)\r\n   >:")
	uart_port_name = raw_input("   Enter UART port name. (e.g. 'PORTC' or 'PORTD')\r\n   >:")
	uart_number = raw_input("   Enter UART number. (Should be '0' or '1')\r\n   >:")
	
if use_enter_i2c == "yes":
	print "\r\nYou have opted to use I2C for communications."
	i2c_device_port = raw_input("   Enter the I2C port name. (e.g. 'PORTC' or 'PORTE')\r\n   >:")
	i2c_match_any = raw_input("   Should the I2C controller use promiscuous mode (match any address)? [yes/no]\r\n   >:")
	i2c_address = raw_input("   Enter the I2C address. (In HEX, but without leading '0x')\r\n   >:")
	i2c_gc_enable = raw_input("   Enable I2C bus general call capability? [yes/no]\r\n   >:")
	i2c_autoneg_dis_promisc = raw_input("   Disable I2C promiscuous mode after completion of autonegotiation routine? [yes/no]\r\n   >:")
	i2c_autoneg_dis_gc = raw_input("   Disable I2C general call detection after completion of autonegotiation routine? [yes/no]\r\n   >:")
	i2c_autoneg_port = raw_input("   Which port is used for autonegotiation? (e.g., 'PORTA', etc.)\r\n   >:")
	i2c_autoneg_pin = raw_input("   Which pin on " + i2c_autoneg_port + " is the autonegotiation pin?\r\n   >:")

print "\r\nSection 4: Special Fixes"

use_avr1008_eeprom = raw_input("\r\nShould the bootloader use Flash/EEPROM fixes noted in AVR1008? (See documentation if your chip is affected) [yes/no]\r\n>:")

print "\r\nSection 5: Bootloader Exit"

lock_spm_on_exit = raw_input("\r\nShould SPM be locked upon bootloader exit? (If you're not sure, the answer is definitely 'no'.) [yes/no]\r\n>:")

print "\r\n\r\n***********************************************************************************************************************************\r\n"
print "I think I have all the information I need to write a new xboot.h file for you.  Are you sure you've entered all the info correctly?"
if raw_input("Enter yes to continue.  [yes/no]\r\n>:") != "yes":
	sys.exit()


# Check to see if xboot.h exists.  If it does, move it and timestamp it so we don't overwrite it.
if os.path.isfile("xboot.h"):
	shutil.move("xboot.h", "xboot_" + string.replace(string.replace(str(datetime.datetime.now())[:19], ":", "_"), " ", "_") + ".h")

f = open("xboot.h", 'w')

f.write('/************************************************************************/\r\n')
f.write('/* XBoot Extensible AVR Bootloader                                      */\r\n')
f.write('/*                                                                      */\r\n')
f.write('/* tested with ATXMEGA64A3, ATXMEGA128A1, ATXMEGA256A1, ATXMEGA32A4     */\r\n')
f.write('/*                                                                      */\r\n')
f.write('/* xboot.h                                                              */\r\n')
f.write('/*                                                                      */\r\n')
f.write('/* Alex Forencich <alex@alexforencich.com>                              */\r\n')
f.write('/*                                                                      */\r\n')
f.write('/* Copyright (c) 2010 Alex Forencich                                    */\r\n')
f.write('/*                                                                      */\r\n')
f.write('/* Permission is hereby granted, free of charge, to any person          */\r\n')
f.write('/* obtaining a copy of this software and associated documentation       */\r\n')
f.write('/* files(the "Software"), to deal in the Software without restriction,  */\r\n')
f.write('/* including without limitation the rights to use, copy, modify, merge, */\r\n')
f.write('/* publish, distribute, sublicense, and/or sell copies of the Software, */\r\n')
f.write('/* and to permit persons to whom the Software is furnished to do so,    */\r\n')
f.write('/* subject to the following conditions:                                 */\r\n')
f.write('/*                                                                      */\r\n')
f.write('/* The above copyright notice and this permission notice shall be       */\r\n')
f.write('/* included in all copies or substantial portions of the Software.      */\r\n')
f.write('/*                                                                      */\r\n')
f.write('/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      */\r\n')
f.write('/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF   */\r\n')
f.write('/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                */\r\n')
f.write('/* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS  */\r\n')
f.write('/* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   */\r\n')
f.write('/* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN    */\r\n')
f.write('/* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     */\r\n')
f.write('/* SOFTWARE.                                                            */\r\n')
f.write('/*                                                                      */\r\n')
f.write('/************************************************************************/\r\n')

f.write('\r\n#ifndef __XBOOT_H\r\n')
f.write('#define __XBOOT_H\r\n')
          
f.write('\r\n#include <avr/io.h>\r\n') 
f.write('#include <util/delay.h>\r\n')
f.write('#include <avr/interrupt.h> \r\n')
        
f.write('\r\n// token pasting\r\n')
f.write('#define token_paste2_int(x, y) x ## y\r\n')
f.write('#define token_paste2(x, y) token_paste2_int(x, y)\r\n')
f.write('#define token_paste3_int(x, y, z) x ## y ## z\r\n')
f.write('#define token_paste3(x, y, z) token_paste3_int(x, y, z)\r\n')
    
f.write('\r\n// Configuration\r\n')

f.write('\r\n// clock config\r\n')
f.write('#define USE_DFLL\r\n')
f.write('// use 32MHz osc if makefile calls for it\r\n')
f.write('#if (F_CPU == 32000000L)\r\n')
f.write('// defaults to 2MHz RC oscillator\r\n')
f.write('// define USE_32MHZ_RC to override\r\n')
f.write('#define USE_32MHZ_RC\r\n')
f.write('#endif\r\n')

f.write('\r\n// AVR1008 fixes\r\n')
f.write('// Really only applicable to 256a3 rev A and B devices\r\n')
write_param(f, use_avr1008_eeprom, "#define USE_AVR1008_EEPROM")		# USE_AVR1008_EEPROM

f.write('\r\n// bootloader entrance\r\n')
write_param(f, use_enter_delay, "#define USE_ENTER_DELAY")			# USE_ENTER_DELAY
write_param(f, use_enter_pin, '#define USE_ENTER_PIN')				# USE_ENTER_PIN
write_param(f, use_enter_uart, '#define USE_ENTER_UART')			# USE_ENTER_UART
write_param(f, use_enter_i2c, '#define USE_ENTER_I2C')				# USE_ENTER_I2C
write_param(f, use_enter_fifo, '#define USE_ENTER_FIFO')			# USE_ENTER_FIFO

f.write('\r\n// bootloader exit\r\n')
write_param(f, lock_spm_on_exit, '#define LOCK_SPM_ON_EXIT')			# LOCK_SPM_ON_EXIT

f.write('\r\n// bootloader communication\r\n')
write_param(f, use_led, '#define USE_LED')					# USE_LED
write_param(f, use_uart, '#define USE_UART')					# USE_UART
write_param(f, use_i2c, '#define USE_I2C')					# USE_I2C
write_param(f, use_fifo, '#define USE_FIFO')					# USE_FIFO
write_param(f, use_i2c_address_negotiation, '#define USE_I2C_ADDRESS_NEGOTIATION')	# USE_I2C_ADDRESS_NEGOTIATION
write_param(f, use_attach_led, '#define USE_ATTACH_LED')			# USE_ATTACH_LED

f.write('\r\n// General Options\r\n')
write_param(f, use_interrupts, '#define USE_INTERRUPTS')			# USE_INTERRUPTS
write_param(f, use_watchdog, '#define USE_WATCHDOG')				# USE_WATCHDOG

f.write('\r\n// bootloader features\r\n')
f.write('#define ENABLE_BLOCK_SUPPORT\r\n')
f.write('#define ENABLE_FLASH_BYTE_SUPPORT\r\n')
f.write('#define ENABLE_EEPROM_BYTE_SUPPORT\r\n')
f.write('#define ENABLE_LOCK_BITS\r\n')
f.write('#define ENABLE_FUSE_BITS\r\n')

f.write('\r\n// ENTER_PIN\r\n')
f.write('#define ENTER_PORT              ' + enter_port + '\r\n')		# ENTER_PORT
f.write('#define ENTER_PIN               ' + enter_pin + '\r\n')		# ENTER_PIN
f.write('#define ENTER_PIN_CTRL          token_paste3(ENTER_PORT.PIN, ENTER_PIN, CTRL)\r\n')
f.write('#define ENTER_PIN_STATE         ' + enter_pin_state + '\r\n')		# ENTER_PIN_STATE
f.write('#define ENTER_PIN_PUEN          ' + enter_pin_puen + '\r\n')		# ENTER_PIN_PUEN

f.write('\r\n// ENTER_DELAY\r\n')
f.write('#define ENTER_BLINK_COUNT       ' + enter_blink_count + '\r\n')	# ENTER_BLINK_COUNT
f.write('#define ENTER_BLINK_WAIT        ' + enter_blink_wait + '\r\n')		# ENTER_BLINK_WAIT

f.write('\r\n// ENTER_UART\r\n')
f.write('//#define ENTER_UART_NEED_SYNC')

f.write('\r\n// ENTER_FIFO\r\n')
f.write('//#define ENTER_FIFO_NEED_SYNC')

f.write('\r\n// WATCHDOG\r\n')
f.write('// Select only one\r\n')
f.write('//#define WATCHDOG_TIMEOUT        WDT_PER_8CLK_gc\r\n')
f.write('//#define WATCHDOG_TIMEOUT        WDT_PER_16CLK_gc\r\n')
f.write('//#define WATCHDOG_TIMEOUT        WDT_PER_32CLK_gc\r\n')
f.write('//#define WATCHDOG_TIMEOUT        WDT_PER_64CLK_gc\r\n')
f.write('//#define WATCHDOG_TIMEOUT        WDT_PER_128CLK_gc\r\n')
f.write('//#define WATCHDOG_TIMEOUT        WDT_PER_256CLK_gc\r\n')
f.write('//#define WATCHDOG_TIMEOUT        WDT_PER_512CLK_gc\r\n')
f.write('//#define WATCHDOG_TIMEOUT        WDT_PER_1KCLK_gc\r\n')
f.write('//#define WATCHDOG_TIMEOUT        WDT_PER_2KCLK_gc\r\n')
f.write('//#define WATCHDOG_TIMEOUT        WDT_PER_4KCLK_gc\r\n')
f.write('//#define WATCHDOG_TIMEOUT        WDT_PER_8KCLK_gc\r\n')
f.write('#define WATCHDOG_TIMEOUT        ' + watchdog_timeout + '\r\n')		# WATCHDOG_TIMEOUT

f.write('\r\n// LED\r\n')
f.write('#define LED_PORT                ' + led_port + '\r\n')			# LED_PORT
f.write('#define LED_PIN                 ' + led_pin + '\r\n')			# LED_PIN
if led_inv == "yes":
	f.write('#define LED_INV                 1\r\n')			# LED_INV
else:
	f.write('#define LED_INV                 0\r\n')

f.write('\r\n// UART\r\n')
f.write('#define UART_BAUD_RATE                  ' + uart_baud_rate + '\r\n')	# UART_BAUD_RATE
f.write('#define UART_PORT_NAME                  ' + uart_port_name[-1:] + '\r\n')	# UART_PORT_NAME (just take the letter of the port)
f.write('#define UART_NUMBER                     ' + uart_number + '\r\n')	# UART_NUMBER
f.write('#if (UART_NUMBER == 0)\r\n')
f.write('#define UART_TX_PIN                     3\r\n')
f.write('#else\r\n')
f.write('#define UART_TX_PIN                     7\r\n')
f.write('#endif\r\n')
f.write('#define UART_PORT                       token_paste2(PORT, UART_PORT_NAME)\r\n')
f.write('#define UART_DEVICE_PORT                token_paste2(UART_PORT_NAME, UART_NUMBER)\r\n')
f.write('#define UART_DEVICE                     token_paste2(USART, UART_DEVICE_PORT)\r\n')
f.write('#define UART_DEVICE_RXC_ISR             token_paste3(USART, UART_DEVICE_PORT, _RXC_vect)\r\n')
f.write('#define UART_DEVICE_DRE_ISR             token_paste3(USART, UART_DEVICE_PORT, _DRE_vect)\r\n')
f.write('#define UART_DEVICE_TXC_ISR             token_paste3(USART, UART_DEVICE_PORT, _TXC_vect)\r\n')

f.write('\r\n// FIFO\r\n')
f.write('#define FIFO_DATA_PORT  PORTC\r\n');
f.write('#define FIFO_CTL_PORT   PORTD\r\n');
f.write('#define FIFO_RXF_N_bm   1<<3\r\n');
f.write('#define FIFO_TXE_N_bm   1<<2\r\n');
f.write('#define FIFO_RD_N_bm    1<<1\r\n');
f.write('#define FIFO_WR_N_bm    1<<0\r\n');
f.write('#define FIFO_BIT_REVERSE\r\n');

f.write('\r\n#ifdef __AVR_XMEGA__\r\n')

f.write('\r\n// BAUD Rate Values\r\n')
f.write('// Known good at 2MHz\r\n')
f.write('#if (F_CPU == 2000000L) && (UART_BAUD_RATE == 19200)\r\n')
f.write('#define UART_BSEL_VALUE         12\r\n')
f.write('#define UART_BSCALE_VALUE       0\r\n')
f.write('#define UART_CLK2X              1\r\n')
f.write('#elif (F_CPU == 2000000L) && (UART_BAUD_RATE == 38400)\r\n')
f.write('#define UART_BSEL_VALUE         22\r\n')
f.write('#define UART_BSCALE_VALUE       -2\r\n')
f.write('#define UART_CLK2X              1\r\n')
f.write('#elif (F_CPU == 2000000L) && (UART_BAUD_RATE == 57600)\r\n')
f.write('#define UART_BSEL_VALUE         26\r\n')
f.write('#define UART_BSCALE_VALUE       -3\r\n')
f.write('#define UART_CLK2X              1\r\n')
f.write('#elif (F_CPU == 2000000L) && (UART_BAUD_RATE == 115200)\r\n')
f.write('#define UART_BSEL_VALUE         19\r\n')
f.write('#define UART_BSCALE_VALUE       -4\r\n')
f.write('#define UART_CLK2X              1\r\n')
f.write('// Known good at 32MHz\r\n')
f.write('#elif (F_CPU == 32000000L) && (UART_BAUD_RATE == 19200)\r\n')
f.write('#define UART_BSEL_VALUE         103\r\n')
f.write('#define UART_BSCALE_VALUE       0\r\n')
f.write('#define UART_CLK2X              0\r\n')
f.write('#elif (F_CPU == 32000000L) && (UART_BAUD_RATE == 38400)\r\n')
f.write('#define UART_BSEL_VALUE         51\r\n')
f.write('#define UART_BSCALE_VALUE       0\r\n')
f.write('#define UART_CLK2X              0\r\n')
f.write('#elif (F_CPU == 32000000L) && (UART_BAUD_RATE == 57600)\r\n')
f.write('#define UART_BSEL_VALUE         34\r\n')
f.write('#define UART_BSCALE_VALUE       0\r\n')
f.write('#define UART_CLK2X              0\r\n')
f.write('#elif (F_CPU == 32000000L) && (UART_BAUD_RATE == 115200)\r\n')
f.write('#define UART_BSEL_VALUE         16\r\n')
f.write('#define UART_BSCALE_VALUE       0\r\n')
f.write('#define UART_CLK2X              0\r\n')
f.write('// None of the above, so calculate something\r\n')
f.write('#else\r\n')
f.write('#warning Not using predefined BAUD rate, possible BAUD rate error!\r\n')
f.write('#if (F_CPU == 2000000L)\r\n')
f.write('#define UART_BSEL_VALUE         ((F_CPU) / ((uint32_t)UART_BAUD_RATE * 8) - 1)\r\n')
f.write('#define UART_BSCALE_VALUE       0\r\n')
f.write('#define UART_CLK2X              1\r\n')
f.write('#else\r\n')
f.write('#define UART_BSEL_VALUE         ((F_CPU) / ((uint32_t)UART_BAUD_RATE * 16) - 1)\r\n')
f.write('#define UART_BSCALE_VALUE       0\r\n')
f.write('#define UART_CLK2X              0\r\n')
f.write('#endif\r\n')
f.write('#endif\r\n')

f.write('\r\n#endif // __AVR_XMEGA__\r\n')

f.write('\r\n// I2C\r\n')
f.write('#ifdef __AVR_XMEGA__\r\n')

f.write('\r\n#define I2C_DEVICE_PORT                 ' + i2c_device_port[-1:] + '\r\n')	# I2C_DEVICE_PORT
f.write('#define I2C_DEVICE                      token_paste2(TWI, I2C_DEVICE_PORT)\r\n')
f.write('#define I2C_DEVICE_ISR                  token_paste3(TWI, I2C_DEVICE_PORT, _TWIS_vect)\r\n')

f.write('\r\n#define I2C_MATCH_ANY                   ' + i2c_match_any + '\r\n')	# I2C_MATCH_ANY
f.write('#define I2C_ADDRESS                     0x' + i2c_address + '\r\n')		# I2C_ADDRESS
f.write('#define I2C_GC_ENABLE                   ' + i2c_gc_enable + '\r\n')		# I2C_GC_ENABLE

f.write('\r\n#endif // __AVR_XMEGA__\r\n')

f.write('\r\n// I2C Address Autonegotiation\r\n')
f.write('// Note: only works on XMega chips for the time being\r\n')
f.write('// There is no easy way to get this to work on regular\r\n')
f.write('// ATMega chips as they have no unique part ID number\r\n')
if i2c_autoneg_dis_promisc == "yes":
	f.write('#define I2C_AUTONEG_DIS_PROMISC         1\r\n')		# I2C_AUTONEG_DIS_PROMISC
else:
	f.write('#define I2C_AUTONEG_DIS_PROMISC         0\r\n')
if i2c_autoneg_dis_gc == "yes":
	f.write('#define I2C_AUTONEG_DIS_GC              1\r\n')		# I2C_AUTONEG_DIS_GC
else:
	f.write('#define I2C_AUTONEG_DIS_GC              0\r\n')
f.write('#define I2C_AUTONEG_PORT                ' + i2c_autoneg_port + '\r\n')	# I2C_AUTONEG_PORT
f.write('#define I2C_AUTONEG_PIN                 ' + i2c_autoneg_pin + '\r\n')	# I2C_AUTONEG_PIN

f.write('\r\n// Attach LED\r\n')
f.write('#define ATTACH_LED_PORT                 ' + attach_led_port + '\r\n')	# ATTACH_LED_PORT
f.write('#define ATTACH_LED_PIN                  ' + attach_led_pin + '\r\n')	# ATTACH_LED_PIN
if attach_led_inv == "yes":
	f.write('#define ATTACH_LED_INV                  1\r\n')		# ATTACH_LED_INV
else:
	f.write('#define ATTACH_LED_INV                  0\r\n')

f.write('\r\n#ifndef EEPROM_BYTE_ADDRESS_MASK\r\n')
f.write('#if EEPROM_PAGE_SIZE == 32\r\n')
f.write('#define EEPROM_BYTE_ADDRESS_MASK 0x1f\r\n')
f.write('#else\r\n')
f.write('#error Unknown EEPROM page size!  Please add new byte address value!\r\n')
f.write('#endif\r\n')
f.write('#endif\r\n')

f.write('\r\n#ifdef USE_INTERRUPTS\r\n')
f.write('#ifndef NEED_INTERRUPTS\r\n')
f.write('#define NEED_INTERRUPTS\r\n')
f.write('#endif // NEED_INTERRUPTS\r\n')
f.write('#endif // USE_INTERRUPTS\r\n')

f.write('\r\n#ifdef USE_AVR1008_EEPROM\r\n')
f.write('#ifndef NEED_INTERRUPTS\r\n')
f.write('#define NEED_INTERRUPTS\r\n')
f.write('#endif // NEED_INTERRUPTS\r\n')
f.write('#endif // USE_AVR1008_EEPROM\r\n')

f.write('\r\n// communication modes\r\n')
f.write('#define MODE_UNDEF              0\r\n')
f.write('#define MODE_UART               1\r\n')
f.write('#define MODE_I2C                2\r\n')
f.write('#define MODE_FIFO               3\r\n')

f.write('\r\n// types\r\n')
f.write('typedef uint32_t ADDR_T;\r\n')

f.write('\r\n// Includes\r\n')
f.write('#include "protocol.h"\r\n')
f.write('#include "sp_driver.h"\r\n')
f.write('#include "eeprom_driver.h"\r\n')
f.write('#include "uart.h"\r\n')
f.write('#include "i2c.h"\r\n')
f.write('#include "watchdog.h"\r\n')

f.write('\r\n// globals\r\n')
f.write('#ifdef USE_INTERRUPTS\r\n')
f.write('extern volatile unsigned char comm_mode;\r\n')

f.write('\r\nextern volatile unsigned char rx_buff0;\r\n')
f.write('extern volatile unsigned char rx_buff1;\r\n')
f.write('extern volatile unsigned char rx_char_cnt;\r\n')

f.write('\r\nextern volatile unsigned char tx_buff0;\r\n')
f.write('extern volatile unsigned char tx_char_cnt;\r\n')
f.write('#else\r\n')
f.write('extern unsigned char comm_mode;\r\n')
f.write('#endif // USE_INTERRUPTS\r\n')

f.write('\r\n// Functions\r\n')
f.write('unsigned char __attribute__ ((noinline)) ow_slave_read_bit(void);\r\n')
f.write('void __attribute__ ((noinline)) ow_slave_write_bit(unsigned char b);\r\n')
f.write('void ow_slave_wait_bit(void);\r\n')

f.write('\r\nunsigned char __attribute__ ((noinline)) get_char(void);\r\n')
f.write('void __attribute__ ((noinline)) send_char(unsigned char c);\r\n')
f.write('unsigned int __attribute__ ((noinline)) get_2bytes(void);\r\n')

f.write('\r\nunsigned char BlockLoad(unsigned int size, unsigned char mem, ADDR_T *address);\r\n')
f.write('void BlockRead(unsigned int size, unsigned char mem, ADDR_T *address);\r\n')


f.write('\r\n\r\n#endif // __XBOOT_H\r\n\r\n')

f.close()

# Check to see if xboot.h exists.  If it does, move it and timestamp it so we don't overwrite it.
if os.path.isfile("xboot-config.mk"):
	shutil.move("xboot-config.mk", "xboot-config_" + string.replace(string.replace(str(datetime.datetime.now())[:19], ":", "_"), " ", "_") + ".mk")

f = open("xboot-config.mk", 'w')
f.write('# xboot-config.mk makefile part\r\n\r\n')
f.write('# automatically generated by xboot-config.py\n\r\n')
f.write('MCU = ' + mcu + '\r\n\r\n')
f.write('F_CPU = ' + f_cpu.replace('L','') + '\r\n')
f.close()

print "A new xboot.h file has been generated for you.  The next steps are:"
print ">: make clean"
print ">: make"
print ">: make program"
print "\r\nYou may still need to make some changes to the Makefile.  TODO: Add functionality to ask questions and generate Makefile also."
print "\r\nEnjoy XBoot!"

