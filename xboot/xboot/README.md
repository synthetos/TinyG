# XBoot Readme

See the XBoot wiki for the latest version of this information:
http://alexforencich.com/wiki/en/xboot/

Also, please check the git repository on github:
https://github.com/alexforencich/xboot

### Table of Contents

    1 Introduction
        1.1 Compatibility List
    2 Using XBoot
        2.1 Configure
        2.2 Build XBoot and Program to Chip
        2.3 Write Main Application Program
        2.4 Notes for Main Application
    3 Configuring XBoot
        3.1 Bootloader clock options
        3.2 AVR 1008 fixes
        3.3 Bootloader entrance options
        3.4 Bootloader exit options
        3.5 Bootloader communication
        3.6 General Options
        3.7 Bootloader features
        3.8 API Support
        3.9 Code Protection
    4 XBoot API
        4.1 Using the API
        4.2 Return Values
        4.3 General API Functions
        4.4 Low-level Flash Programming API
        4.5 Firmware Update API
        4.6 Offsets defined in xbootapi.h
        4.7 Using the Firmware Update API

## 1 Introduction

XBoot is an extensible, modular bootloader for the ATMEL AVR processor series,
compatible with both AVR ATMEGA and AVR XMEGA devices with sufficient memory.
It is compatible with the AVR109 (butterfly) bootloader protocol with a few
XMEGA specific extensions for access to the user and production signature
rows. XBoot includes several advanced features including multiple serial
busses and an API providing the ability for the running application to update
itself without the need for external programming hardware. Many bootloaders
only support RS232 for programming from a PC, but XBoot's modularity allows it
to support the exact same set of commands over any hardware serial port.
Currently, I²C, RS485, and parallel FIFO support have been incorporated. This
allows for easy in-system reconfiguration of XBoot equipped chips with little
additional time investment. Also, XBoot includes support for I²C address
autonegotiation for when multiple, identically configured processors sit on
the same I²C bus.

Thanks for using XBoot!

Alex Forencich

### 1.1 Compatibility List

Currently, XBoot should work on any XMEGA processor and any ATMEGA with
sufficient memory (4K boot NRWW block). The following list of processors are
currently supported. An asterisk denotes the MCU has been tested and confirmed
XBoot compatible.

  * XMEGA 
    * atxmega16a4 *
    * atxmega32a4 *
    * atxmega64a1
    * atxmega64a3 *
    * atxmega64a4
    * atxmega128a1 *
    * atxmega128a3 *
    * atxmega128a4
    * atxmega192a1
    * atxmega192a3
    * atxmega256a1
    * atxmega256a3b
    * atxmega256a3 *
    * atxmega16d4
    * atxmega32d4
    * atxmega64d3
    * atxmega64d4
    * atxmega128d3
    * atxmega128d4
    * atxmega192d3
    * atxmega256d3
    * atxmega16a4u
    * atxmega32a4u *
    * atxmega64a3u
    * atxmega64a4u
    * atxmega128a3u
    * atxmega128a4u
    * atxmega192a3u
    * atxmega256a3u
    * atxmega256a3bu
    * atxmega64b1
    * atxmega64b3
    * atxmega128b1
    * atxmega128b3
  * ATMEGA
    * atmega324
    * atmega324pa
    * atmega325
    * atmega3250
    * atmega328p *
    * atmega329
    * atmega3290
    * atmega64
    * atmega640
    * atmega644
    * atmega644pa
    * atmega645
    * atmega6450
    * atmega649
    * atmega649p
    * atmega6490
    * atmega128
    * atmega1280
    * atmega1281
    * atmega1284p *
    * atmega2560
    * atmega2561

## 2 Using XBoot

### 2.1 Configure

Before building XBoot, please configure it so it will interface properly with
your system. This will involve selecting a .conf.mk file in the conf directory
and then editing some parameters. The main parameters that need to be set in
the .conf.mkfile are the target chip (MCU) and the frequency (F_CPU). For
ATMEGA chips, the boot size (BOOTSZ) must also be set, generally to the largest
setting. All you need to do is make sure the only line that's not commented
out is the one for your chip and the proper frequency. For the simplest
bootloader configuration on the XMEGA, you may only choose 2000000 and
32000000 for the clock speed, corresponding to the two internal RC oscillator
frequencies. For ATMEGA chips, the specific start up clock speed (based on the
fuse settings and/or external crystal or other clock source) must be
specifically entered manually. For the rest of the configuration, see the
section 3, “Configuring XBoot”.

### 2.2 Build XBoot and Program to Chip

To build XBoot, select a .conf.mk file from the conf directory and make sure
the settings are appropriate for your chip, programmer, and configuration.
Then type “make [file].conf.mk”. This will copy [file].conf.mk to config.mk,
generate config.h, compile the whole package, and generate xboot.hex, which
can be downloaded with any programming cable capable of programming AVR chips.
If you have an XMEGA and want to save some time and just program the boot
section, type “make xboot-boot.hex” and then write the new file xboot-boot.hex
to the boot section directly with avrdude. The makefile includes built-in
support for avrdude, so all you need to do is modify the avrdude parameters
in the .conf.mk file for the programmer you have and type “make program”.

Note that after typing “make [file].conf.mk”, the settings from [file].conf.mk
will remain active until either a new .conf.mk file is selected or “make
clean” is run.


### 2.3 Write Main Application Program

To write a program to a device with XBoot installed, use a command like this:

    avrdude -p atxmega64a3 -P /dev/ttyUSB0 -c avr109 -b 115200 -U flash:w:main.hex

Or for windows:

    avrdude -p atxmega64a3 -P com1 -c avr109 -b 115200 -U flash:w:main.hex

Also, feel free to re-use XBoot's makefile for your own code. Like XBoot, it
is reconfigurable and can be used to compile most projects, just make sure to
turn off the MAKE_BOOTLOADER flag for regular programs. It also has the
configuration options for XBoot as a target built in, all you need to do is
switch a couple of comments around.

**NOTE:** At this time, avrdude (currently 5.10) does NOT support programming 
the XMEGA flash boot section (see https://savannah.nongnu.org/bugs/?28744)
due to the fact that a different programming command must be sent to the chip
to write flash pages in the boot section. If you want to use avrdude, you
will need to compile it from source with one of the patches listed on the
bug report. 

### 2.4 Notes for Main Application

Here are a few tips for your main application that will make using XBoot a
much more pleasant experience.

#### 2.4.1 Catch the "Enter Bootloader" command

When AVRDude starts programming the chip, the first character sent out is the
the sync command; the “Escape” character, 0x1B. If your program transmits
ASCII, or only transmits Binary during certain program states, you can monitor
the UART for the escape character and cause a software reset to enter the
bootloader, as shown in the following snippet on an XMEGA:

    if (rx_byte == 0x1B) {
       CCPWrite( &RST.CTRL, RST_SWRST_bm );
    }

Or, if CCPWrite is not available, this should also work:

    if (rx_byte == 0x1B) {
       CCP = CCP_IOREG_gc;
       RST.CTRL = RST_SWRST_bm;
    }

In many cases, this allows you to use the AVRDude program command without
having to manually reset the AVR. Alternatively, the API call
`xboot_reset()` will have the same effect.

#### 2.4.2 Combine hex files

To streamline programming multiple chips for a production run, the tool
`srec_cat` can be used to combine the hex files.  Use the command as follows:

    srec_cat xboot.hex -intel main.hex -intel -o mergedfile.hex -intel

This will concatenate the two hex files together with the proper offsets.  
Please make sure to use xboot.hex and not xboot-boot.hex to ensure tha the
correct offset is used (xboot.hex is relative to address 0 while
xboot-boot.hex is relative to the start of the boot section).  Note that the
combined hex file cannot be programmed with xboot; it is intended to be
written with an ISP programmer so both xboot and the application are written
in one step.  

## 3 Configuring XBoot

XBoot is designed to be reconfigured to suit specific needs. Out of the box,
all of the standard features are turned on. Turning off features and
reassigning pins is easy, just pick a .conf.mk file in the .conf folder, make
a copy of it, and edit the appropriate parameters. Then build xboot with your
parameters by running `make [file].conf.mk`.

Recommended configuration:

    # Entry
    USE_ENTER_DELAY = yes
    USE_ENTER_UART = yes
     
    # Communication
    USE_LED = yes
    USE_UART = yes
     
    # Bootloader Features
    ENABLE_BLOCK_SUPPORT = yes
    ENABLE_FLASH_BYTE_SUPPORT = yes
    ENABLE_EEPROM_BYTE_SUPPORT = yes
    ENABLE_LOCK_BITS = yes
    ENABLE_FUSE_BITS = yes

This configuration will make the bootloader work similarly to an Arduino. It
will blink its light a few times, polling for a character. If none is
received, it starts the application. If one shows up, it enters the bootloader
and processes it.

In fact, the file arduino328p.conf.mk can be used to build XBoot for use on an
atmega328p based Arduino. 

### 3.1 Bootloader clock options

#### 3.1.1 USE_DFLL

This will turn on the DFLL for the selected oscillator, improving its
accuracy. Recommended for high serial baud rates. This option only applies to
XMEGA processors.

#### 3.1.2 USE_32MHZ_RC

This will switch to the 32MHz RC oscillator on start. In the default
configuration of xboot.h, this will be defined automatically when `F_CPU` is
set to 32000000. This option only applies to XMEGA processors.

### 3.2 AVR 1008 fixes

If you're using a device affected by AVR1008, then you may need to enable
these for the bootloader to successfully program the chip. Affected chips are
the ATXMEGA256A3 rev A, ATXMEGA256A3B rev B, ATXMEGA256A3 rev B, and possibly
the ATXMEGA192A3.

#### 3.2.1 USE_AVR1008_EEPROM

This enables the AVR1008 fix for the EEPROM controller. This option only
applies to XMEGA processors.

### 3.3 Bootloader entrance options

#### 3.3.1 USE_ENTER_DELAY

If this is defined, XBoot will run a loop, specified with the `ENTER_BLINK_*`
variables, and check for an entry condition. If none is found, it jumps into
the main code. (BTW, they're called `ENTER_BLINK_*` because they assume
`USE_LED` is defined. If it isn't, it will still work, but the variable names
don't make a whole lot of sense…)

Options

  * `ENTER_BLINK_COUNT` defines the number of times to blink the LED, e.g. 3 
  * `ENTER_BLINK_WAIT` defines the number of loops to make between blinks, e.g. 30000 

#### 3.3.2 USE_ENTER_PIN

If this is defined, XBoot will check the state of a pin, specified with the
`ENTRY_PORT` and `ENTRY_PIN_*` variables, when it starts (and possibly
throughout the startup delay loop) to determine if it should start or just
jump into the main program.

Options

  * `ENTER_PORT` defines the port that the in is in, e.g. `PORTC`
  * `ENTER_PIN` defines the pin in the port, an integer from 0 to 7 
  * `ENTER_PIN_CTRL` defines the `PINnCTRL` register for the pin, e.g. `ENTER_PORT.PIN0CTRL`
  * `ENTER_PIN_STATE` defines the “asserted” state of the pin, 0 or 1 
  * `ENTER_PIN_PUEN` enables a pull-up resistor on the pin if nonzero 

#### 3.3.3 USE_ENTER_UART

If this is defined, XBoot will poll for received characters over the UART. If
one is received, it will enter the bootloader code. `USE_UART` must be
defined. `ENTER_UART_NEED_SYNC` can also be defined to require the sync
command (0x1B) in order to enter the bootloader. This can help prevent the
bootloader from being started accidentally.

#### 3.3.4 USE_ENTER_I2C

If this is defined, XBoot will poll for received characters over the I2C
interface. If one is received, it will enter the bootloader code. `USE_I2C`
must be defined.

#### 3.3.5 USE_ENTER_FIFO

If this is defined, XBoot will poll for received characters over the FIFO. If
one is received, it will enter the bootloader code. `USE_FIFO` must be
defined. `ENTER_FIFO_NEED_SYNC` can also be defined to require the sync
command (0x1B) in order to enter the bootloader. This can help prevent the
bootloader from being started accidentally.

### 3.4 Bootloader exit options

#### 3.4.1 LOCK_SPM_ON_EXIT

If this is defined, SPM instructions will be locked on bootloader exit.

### 3.5 Bootloader communication

#### 3.5.1 USE_LED

If this is defined, XBoot will use an LED for feedback, specified by the
`LED_*` variables.

Options

  * `LED_PORT_NAME` defines the port, e.g. `A` for `PORTA`
  * `LED_PIN` defines the pin, e.g. 0 
  * `LED_INV` inverts the LED state if nonzero 

#### 3.5.2 USE_UART

If this is defined, XBoot will configure and use a UART for communication.

Options

  * `UART_BAUD_RATE` defines the baud rate of the UART, e.g. 19200 
  * `UART_PORT_NAME` defines the port that the UART is connected to, e.g. `D`
    * Note: this only applies to XMEGA devices 
  * `UART_NUMBER` defines number of the UART device on the port, e.g. 1 for USARTD1 (or USART1 for ATMEGA) 
  * `UART_U2X` turns on the double-rate BRG in ATMEGA parts 
    * Note: this only applies to ATMEGA devices 
  * `UART_RX_PUEN` enables a pull-up on the UART RX pin

#### 3.5.3 USE_UART_EN_PIN

If this is defined along with `USE_UART`, XBoot will configure and use a
transmit enable pin. This allows configuration over a half-duplex RS485
connection.

Options

  * `UART_EN_PORT_NAME` defines the port, e.g. `C` for `PORTC`
  * `UART_EN_PIN` defines the pin, e.g. 4 
  * `UART_EN_INV` inverts the EN pin output state if nonzero 

#### 3.5.4 USE_I2C

If this is defined, XBoot will configure and use an I2C/TWI controller in
slave mode for communication.

Note: Currently only implemented on XMEGA.

Options

  * `I2C_DEVICE_PORT` defines the port the I2C interface is on, e.g. `E` for TWIE 
  * `I2C_MATCH_ANY` will enable the I2C controller promiscuous mode (match any address) if nonzero 
  * `I2C_ADDRESS` defines the default I2C address 0x10 
  * `I2C_GC_ENABLE` enables the I2C bus general call capability (address 0) if nonzero 

#### 3.5.5 USE_I2C_ADDRESS_NEGOTIATION

Enables I2C address autonegotiation if defined. Requires `USE_I2C`.

Note: Currently only implemented on XMEGA due to the presence of the
Production Signature Row with a signature unique to each chip produced. A
suitable workaround for ATMEGA has not yet been implemented.

Options

  * `I2C_AUTONEG_DIS_PROMISC` will disable I2C promiscuous mode after completion of autonegotiation routine if nonzero 
  * `I2C_AUTONEG_DIS_GC` will disable I2C general call detection after completion of autonegotiation routine if nonzero 
  * `I2C_AUTONEG_PORT_NAME` defines the port in which the autonegotiation pin is located, e.g. `A`
  * `I2C_AUTONEG_PIN` defines the pin, e.g. 2 

#### 3.5.6 USE_ATTACH_LED

Enables the autonegotiation code to turn on a light when a new I2C address is
received.

Options

  * `ATTACH_LED_PORT_NAME` defines the port, e.g. `A` for `PORTA`
  * `ATTACH_LED_PIN` defines the pin, e.g. 1 
  * `ATTACH_LED_INV` inverts the LED state if nonzero 

#### 3.5.7 USE_FIFO

If this is defined, XBoot will talk to an external parallel FIFO for
communication.

Options

  * `FIFO_DATA_PORT_NAME` defines the FIFO data port, e.g. `C` for `PORTC`
  * `FIFO_CTL_PORT_NAME` defines the FIFO control port, e.g. `D` for `PORTD`
  * `FIFO_RXF_N_bm` defines the receive flag pin mask on the control port, e.g. `(1 << 3)` for pin 3 
  * `FIFO_TXE_N_bm` defines the transmit enable pin mask on the control port 
  * `FIFO_RD_N_bm` defines the read strobe pin mask on the control port 
  * `FIFO_WR_N_bm` defines the write strobe pin mask on the control port 
  * `FIFO_BIT_REVERSE` will reverse the data bits 

### 3.6 General Options

#### 3.6.1 USE_INTERRUPTS

Defining this will configure XBoot to use interrupts instead of polled I/O for
serial communications. This will increase code size and won't offer much
advantage at the time being, so only use if you know what you're doing.

#### 3.6.2 USE_WATCHDOG

Defining this will enable the watchdog timer during operation of the
bootloader. This can reduce the overhead caused by failed programming attempts
by resetting the chip if the bootloader and host get out of sync.

Options

  * `WATCHDOG_TIMEOUT` determines the watchdog timeout period; leave only one of the listed lines uncommented (see XMEGA A series datasheet for details) 

### 3.7 Bootloader features

Generally, these are all enabled, but they can be disabled to save code space.

#### 3.7.1 ENABLE_BLOCK_SUPPORT

Enables flash block access support

#### 3.7.2 ENABLE_FLASH_BYTE_SUPPORT

Enables flash byte access support

#### 3.7.3 ENABLE_EEPROM_BYTE_SUPPORT

Enables EEPROM byte access support

#### 3.7.4 ENABLE_LOCK_BITS

Enables lock bit read and write support (note: cannot clear lock bits to 1,
complete chip erase from external programmer needed to do that)

Note: only supported on XMEGA

#### 3.7.5 ENABLE_FUSE_BITS

Enables fuse bit read support (cannot write fuse bits outside of hardware
programming)

Note: only supported on XMEGA

#### 3.7.6 ENABLE_FLASH_ERASE_WRITE

Erase each page before writing. This allows the device to be reprogrammed
without a complete erase sequence.

#### 3.7.7 ENABLE_CRC_SUPPORT

Enables commands for computing the CRC of various sections of Flash memory.

### 3.8 API Support

#### 3.8.1 ENABLE_API

Enable API functionality. This functionality can be completely disabled to
save space.

#### 3.8.2 USE_API_VERSION

Select API version to implement. Currently the only legal value is 1.

#### 3.8.3 ENABLE_API_LOW_LEVEL_FLASH

Enable low level flash access APIs. Turns on the following API calls:

  * `xboot_spm_wrapper` (can be separately disabled) 
  * `xboot_erase_application_page`
  * `xboot_write_application_page`
  * `xboot_write_user_signature_row` (XMEGA specific) 

#### 3.8.4 ENABLE_API_SPM_WRAPPER

Enable SPM wrapper API. Requires `ENABLE_API_LOW_LEVEL_FLASH` to be defined.

#### 3.8.5 ENABLE_API_FIRMWARE_UPDATE

Enable firmware update APIs. Turns on the following API calls in addition to
enabling the firmware upgrade code in xboot:

  * `xboot_app_temp_erase`
  * `xboot_app_temp_write_page`

### 3.9 Code Protection

The code protection features built into xboot keep your code safe from reverse
engineering. Protected areas will read the same as unprogrammed flash or
EEPROM (0xff).

#### 3.9.1 ENABLE_CODE_PROTECTION

Enable basic code protection. Code protection prevents reading of the flash
memory via xboot's interface. Code protection is temporarily disabled by an
erase command, allowing verification of newly written firmware. Code 
protection does not disable CRC commands.

#### 3.9.2 ENABLE_EEPROM_PROTECTION

Enable EEPROM protection. EEPROM protection prevents reading of the EEPROM
memory via xboot's interface. Like code protection, EEPROM protection is
temporarily disabled by an erase command, allowing verification of newly
written firmware.

#### 3.9.3 ENABLE_BOOTLOADER_PROTECTION

Enable bootloader protection. Bootloader protection prevents reading of the
boot block via xboot's interface. Unlike code protection, bootloader
protection is not disabled by an erase command.

## 4 XBoot API

XBoot provides several hooks to enable flash reprogramming by the application.
Since the `SPM` instruction is disabled when executing in the application
section (Read-While-Write, RWW), it is not possible for an application to
write to any location in flash memory without being able to use code located
in the boot (Non-Read-While-Write, NRWW) section.

XBoot provides three different types of API calls. The first type are
informational and are required for operation of the API. The second type are
for low-level Flash programming access. The third are for controlled firmware
updating. The low-level Flash programming calls and the firmware update calls
can be selectively disabled separately.

**Note:** XBoot automatically disables interrupts during Flash programming
operations. This is necessary because, despite the name, RWW flash cannot be
read and written at the same time (NRWW can be read while RWW is written,
though). The interrupt bit will be automatically restored on return from an
API call. 

### 4.1 Using the API

The API requires some loader code to find and execute the APIs. This code is
fully contained within `xbootapi.c` and `xbootapi.h`. Include these files in
your application to use all of the xboot API calls.

The loader must also know the size of the boot section on the chip in order to
calculate all of its offsets and find the jump table. This is defined in the
header files for XMEGA chips as `BOOT_SECTION_SIZE` since it is constant.
Since ATMEGA chips are generally reconfigurable, it is not constant and
therefore must be defined manually in xbootapi.h or passed to avr-gcc with
`-DBOOT_SECTION_SIZE=0x…`. The xboot makefile does this automatically when it
builds xboot for many ATMEGA chips and the makefile can be freely reused for
your application, simplifying this process.

**Note:** All the page-based flash access commands work on Flash pages
`SPM_PAGESIZE` bytes in size, located at addresses of multiples of
`SPM_PAGESIZE`. 

### 4.2 Return Values

All of the API calls except for `xboot_reset` return a value indicating
success or an error code, defined in `xbootapi.h`.

    #define XB_SUCCESS 0
    #define XB_ERR_NO_API 1
    #define XB_ERR_NOT_FOUND 2
    #define XB_INVALID_ADDRESS 3

  * `XB_SUCCESS` is returned when the call succeeds 
  * `XB_ERR_NO_API` is returned when the loader cannot find the API calls in xboot (either the APIs are disabled, xboot is not installed, or the loader is not looking at the right address) 
  * `XB_ERR_NOT_FOUND` is returned when the particular API call is not found because it is disabled in xboot 
  * `XB_INVALID_ADDRESS` is returned when an invalid address is passed to an API call 

### 4.3 General API Functions

The general API functions are informational only.

    uint8_t xboot_get_version(uint16_t *ver);
    uint8_t xboot_get_api_version(uint8_t *ver);

#### 4.3.1 xboot_get_version

    uint8_t xboot_get_version(uint16_t *ver);

Returns the version of xboot in `ver`, MSB is major version and LSB is minor
version.

#### 4.3.2 xboot_get_api_version

    uint8_t xboot_get_api_version(uint8_t *ver);

Returns the API version in `ver`. Currently the only legal value is 1.

### 4.4 Low-level Flash Programming API

The low-level Flash programming API provides low-level access to the Flash
memory. Can be disabled in xboot via `ENABLE_API_LOW_LEVEL_FLASH`

    uint8_t xboot_spm_wrapper(void);
    uint8_t xboot_erase_application_page(uint32_t address);
    uint8_t xboot_write_application_page(uint32_t address, uint8_t *data, uint8_t erase);
    uint8_t xboot_write_user_signature_row(uint8_t *data);

#### 4.4.1 xboot_spm_wrapper

    uint8_t xboot_spm_wrapper(void);

Not currently implemented. Will eventually be used to execute any valid SPM
command. Use with caution. Can be independently disabled in xboot via
`ENABLE_API_SPM_WRAPPER`

#### 4.4.2 xboot_erase_application_page

    uint8_t xboot_erase_application_page(uint32_t address);

Erase the page in application memory (`SPM_PAGESIZE` bytes) pointed to by
`address`.

#### 4.4.3 xboot_write_application_page

    uint8_t xboot_write_application_page(uint32_t address, uint8_t *data, uint8_t erase);

Write `SPM_PAGESIZE` bytes of `data` to the page in application memory pointed
to by `address`, erasing before hand if `erase` is nonzero.

#### 4.4.4 xboot_write_user_signature_row

    uint8_t xboot_write_user_signature_row(uint8_t *data);

XMEGA only. Write `data` to the user signature row, automatically erasing it
beforehand.

### 4.5 Firmware Update API

The firmware update API allows client firmware to update itself atomically
while making it difficult for the application to accidentally overwrite
itself. The loader can call the underlying low level calls if these higher
level calls are disabled, however, the firmware may not be updated after a
reset if the actual firmware update capability in the bootloader is disabled.

Out of all the API calls listed here, the only actual API calls are
`xboot_app_temp_erase` and `xboot_app_temp_write_page`. The rest do not
require code running in the bootloader space, aside from
`xboot_install_firmware` which calls `xboot_app_temp_write_page` internally,
and so are implemented directly in `xbootapi.c`.

Note that for all the `app_temp` calls, `addr = 0` is not the beggining of
application flash but the beginning of the application temporary section,
generally about the halfway point of the chip's memory. Hence, when using
these calls, it is impossible to overwrite the application section directly.

    uint8_t xboot_app_temp_erase(void);
    uint8_t xboot_app_temp_write_page(uint32_t addr, uint8_t *data, uint8_t erase);
    uint8_t xboot_app_temp_crc16_block(uint32_t start, uint32_t length, uint16_t *crc);
    uint8_t xboot_app_temp_crc16(uint16_t *crc);
    uint8_t xboot_app_crc16_block(uint32_t start, uint32_t length, uint16_t *crc);
    uint8_t xboot_app_crc16(uint16_t *crc);
    uint8_t xboot_install_firmware(uint16_t crc);
    void __attribute__ ((noreturn)) xboot_reset(void);

#### 4.5.1 xboot_app_temp_erase

    uint8_t xboot_app_temp_erase(void);

Erase the application temporary section

#### 4.5.2 xboot_app_temp_write_page

    uint8_t xboot_app_temp_write_page(uint32_t addr, uint8_t *data, uint8_t erase);

Write `SPM_PAGESIZE` bytes of `data` to page in temporary section pointed to
by `addr`, erasing beforehand if `erase` is nonzero. Note that `addr = 0` is
not the beggining of application flash but the beginning of the application
temporary section.

Equivalent to:

    xboot_write_application_page(address + XB_APP_TEMP_START, data, erase);

#### 4.5.3 xboot_app_temp_crc16_block

    uint8_t xboot_app_temp_crc16_block(uint32_t start, uint32_t length, uint16_t *crc);

Compute the crc hash of `length` bytes, starting at `start` in the application
temporary section and return in `crc`. Note that `start = 0` is not the
beggining of application flash but the beginning of the application temporary
section.

Equivalent to:

    xboot_app_crc16_block(start + XB_APP_TEMP_START, length, crc);

#### 4.5.4 xboot_app_temp_crc16

    uint8_t xboot_app_temp_crc16(uint16_t *crc);

Computer the crc hash of the complete application temporary section and return
in `crc`.

Equivalent to:

    xboot_app_temp_crc16_block(0, XB_APP_TEMP_SIZE, crc);

#### 4.5.5 xboot_app_crc16_block

    uint8_t xboot_app_crc16_block(uint32_t start, uint32_t length, uint16_t *crc);

Compute the crc hash of `length` bytes, starting at `start` in the application
section and return in `crc`. Note that `start = 0` is actually address 0 in
flash, the beginning of the application section.

`xboot_app_crc16_block` uses `_crc16_update` from [avr-libc](http://www.nongnu.org/avr-libc/)
<[util/crc16.h](http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html)>
with an initial value of 0 internally to compute the crc.

#### 4.5.6 xboot_app_crc16

    uint8_t xboot_app_crc16(uint16_t *crc);

Computer the crc hash of the complete application section and return in `crc`.

Equivalent to:

    xboot_app_crc16_block(0, XB_APP_SIZE, crc);

#### 4.5.7 xboot_install_firmware

    uint8_t xboot_install_firmware(uint16_t crc);

Write the install firmware command to the end of application temporary flash,
along with the crc of the section passed in `crc`. This crc must be calculated
when the firmware update is built to ensure its correctness, not calculated
with `xboot_app_temp_crc16` on the fly.

This command does not actually install the firmware. It simply goes to the
highest location in application flash and writes “XBIF” followed by the CRC.
After XBoot starts (and if firmware update is enabled), it will look for this
sequence, calculate the crc, install the firmware (if the crc matches), and
erase the application temporary section. Since the command is stored in the
flash memory, if XBoot is interrupted by a reset during the copy operation, it
will simply restart the copy operation from the beginning when it starts up
again, making the update operation atomic.

As this command will return, it is possible to perform clean-up and perhaps
post a message that the device is going down for a firmware update before the
actual update occurs. To reset the chip for the update, call `xboot_reset`.
Note that the firmware install process can be cancelled by erasing the last
page of the application temporary section (or the whole section) before
resetting the chip.

The crc functions all use `_crc16_update` from [avr-libc](http://www.nongnu.org/avr-libc/)
<[util/crc16.h](http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html)>
with an initial crc of 0. The equivalent C code is:

    uint16_t crc16_update(uint16_t crc, uint8_t a)
    {
        int i;
     
        crc ^= a;
        for (i = 0; i < 8; ++i)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc = (crc >> 1);
        }
     
        return crc;
    }

Calculating the crc for a new firmware must be done before it is sent to the
chip for an update. As the crc passed to `xboot_install_firmware` must match
the crc that XBoot calculates over the entire application temporary section,
the firmware must be padded to the size of the application temporary section
(`XB_APP_TEMP_SIZE`) with 0xff when the crc is calculated beforehand.

This checksum will be the same as the one calculated by
`xboot_app_temp_crc16`. The temptation to simply pass the output of this
function back to `xboot_install_firmware` is great. However, this is where a
device can be bricked with ease if the firmware is copied into the application
section incorrectly. Therefore, make sure the precalculated crc is the one
passed to `xboot_install_firmware`.

#### 4.5.8 xboot_reset

    void __attribute__ ((noreturn)) xboot_reset(void);

This call will trigger a device reset and will not return. In XMEGA devices,
it is done via the `RST.CTRL` register. In ATMEGA devices, it uses the
watchdog timer, which XBoot will disable automatically after the reset.

### 4.6 Offsets defined in xbootapi.h

Several offsets and addresses are defined in `xbootapi.h`. They are detailed
in the following table, along with example values (XMEGA with 64K application
flash and 8K boot flash, 72K total flash):

    Name                    Value           Description
    
    PROGMEM_SIZE            0x012000        Size of entire program memory
    
    BOOT_SECTION_START      0x010000        Offset of boot section
    
    APP_SECTION_START       0x000000        Offset of entire application section
    
    APP_SECTION_SIZE        0x010000        Size of entire application section
    
    APP_SECTION_END         0x00FFFF        End address of entire application
                                            section
    
    JUMP_TABLE_LOCATION     0x0101E8        Location of jump table in bootloader
    
    XB_APP_START            0x000000        Offset of application section for
                                            firmware updates
    
    XB_APP_SIZE             0x008000        Size of application section for
                                            firmware updates
    
    XB_APP_END              0x007FFF        End address of application section for
                                            firmware updates
    
    XB_APP_TEMP_START       0x008000        Offset of application temporary
                                            section for firmware updates
    
    XB_APP_TEMP_SIZE        0x008000        Size of application temporary section
                                            for firmware updates
    
    XB_APP_TEMP_END         0x00FFFF        End address of application temporary
                                            section for firmware updates

### 4.7 Using the Firmware Update API

The firmware update API allows client firmware to update itself. The process
is not entirely foolproof, but with sufficient testing it is very difficult to
'brick' a device with the firmware upgrade API.

The memory map of a device using the firmware update API looks something like
the following (XMEGA with 64K application flash and 8K boot flash, 72K total
flash):

    Section                 Address
    --------------------------------------------
                            0x000000
                            XB_APP_START
    
    Application
    32K
      
                            XB_APP_END
                            0x007FFF
    --------------------------------------------
                            0x008000
                            XB_APP_TEMP_START
    
    Temporary Storage
    32K
    
                            XB_APP_TEMP_END
                            0x00FFFF
    --------------------------------------------
                            0x010000
    XBoot
    8K
                            0x011FFF
    --------------------------------------------

To successfully use the firmware update API, the application must fit
completely inside the application section.

Updating is performed by writing the firmware one page of `SPM_PAGESIZE` bytes
at a time to the temporary storage section with `xboot_app_temp_write_page`,
then calling `xboot_install_firmware` to schedule the firmware update, and
finally calling `xboot_reset` to enter XBoot to reset the chip and actually
install the firmware.

When the firmware is not being updated, the temporary storage section can be
used for storing application data.

#### 4.7.1 Firmware update example

The follwing example assumes `get_new_firmware_crc`, `read_new_firmware`, and
`cleanup` are functions with appropriate parameters defined elsewhere in the
client firmware. This routine calls `get_new_firmware_crc` to get the crc of
the firmware update followed by a call to `xboot_app_temp_erase` to erase the
temporary section. Then it calls `read_new_firmware` to read bytes into
read_data, writing accumulated pages to the temporary section until it has no
more bytes to read. The last page is padded with 0xff so as to not affect the
crc calculation. Then it computes the crc of the application temporary
section. If it matches the crc it fetched earlier, it initializes the firmware
install process, cleans up, and resets the chip.

    
    void upgrade_firmware()
    {
        uint8_t page_buffer[SPM_PAGESIZE];
        uint8_t read_data[1024];
        uint8_t *read_ptr;
     
        uint32_t addr = 0;
        uint16_t page_addr = 0;
        uint16_t read_bytes;
     
        uint16_t target_crc = get_new_firmware_crc();
        uint16_t crc;
     
        if (xboot_app_temp_erase() != XB_SUCCESS)
            return;
     
        while (read_bytes = read_new_firmware(read_data, 1024))
        {
            read_ptr = read_data;
     
            while (read_bytes > 0)
            {
                page_buffer[page_addr++] = *read_ptr++;
                read_bytes--;
                if (page_addr >= SPM_PAGESIZE)
                {
                    if (xboot_app_temp_write_page(addr, page_buffer, 0) != XB_SUCCESS)
                        return;
                    addr += SPM_PAGESIZE;
                    page_addr = 0;
                }
            }
        }
     
        if (page_addr > 0)
        {
            while (page_addr < SPM_PAGESIZE)
            {
                page_buffer[page_addr++] = 0xff;
            }
            if (xboot_app_temp_write_page(addr, page_buffer, 0) != XB_SUCCESS)
                return;
        }
     
        xboot_app_temp_crc16(&crc);
     
        if (crc != target_crc)
            return;
     
        if (xboot_install_firmware(target_crc) != XB_SUCCESS)
            return;
    
        cleanup();
     
        xboot_reset();
    }
