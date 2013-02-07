# xmega64a3 configuration

# use config.h
USE_CONFIG_H = yes

# MCU
MCU = atxmega64a3

# Clock Speed
# Use 2 MHz internal RC oscillator
F_CPU = 2000000

# Programmer settings
OVERRIDE_AVRDUDE_PROGRAMMER = yes
AVRDUDE_PROGRAMMER = jtag2pdi
AVRDUDE_PORT = usb

# Fuse settings
AVRDUDE_FUSES =
# If you wish to override the default fuse settings
# determined in main Makefile, change them here
# and then uncomment OVERRIDE_AVRDUDE_FUSES
# See XMega A series datasheet (Atmel doc8077) section 4.16

# Fuse byte 0: JTAG User ID
# If a custom JTAG User ID is required, uncomment
# and set it here
#AVRDUDE_FUSES += -U fuse0:w:0x00:m

# Fuse byte 1: Watchdog
# Set WDPER and WDWPER
# See datasheet sections 4.16.2, 11.7.1, and 11.7.2
# for more information
#AVRDUDE_FUSES += -U fuse1:w:0x00:m

# Fuse byte 2: Reset configuration
# Spike detector, reset vector location, and BOD
# in power down configuration
# See datasheet section 4.16.3 for more information
#AVRDUDE_FUSES += -U fuse2:w:0xBF:m

# There is no fuse byte 3.....

# Fuse byte 4: Start-up configuration
# See datasheet section 4.16.4
# Configures external reset disable, start-up time,
# watchdog timer lock, and jtag enable
#AVRDUDE_FUSES += -U fuse4:w:0xFE:m

# Fuse byte 5
# See datasheet section 4.16.5
# Configures BOD operation in active mode,
# EEPROM preserved through chip erase, and
# BOD detection leven
#AVRDUDE_FUSES += -U fuse5:w:0xFF:m

# Lock byte
# See datasheet section 4.16.6
# Lock bits for boot loader, application,
# and application table sections via internal
# SPM commands and external programming interface
#AVRDUDE_FUSES += -U lock:w:0xFF:m

# Write user sig row (256 bytes max)
# Uncomment to initialize user sig row with custom data
##AVRDUDE_USERSIG = -U usersig:w:0x01,0x02,0x03:m
##AVRDUDE_USERSIG = -U usersig:w:filename
#AVRDUDE_USERSIG = -U usersig:w:...:m

# Uncomment to override default fuse configurations
# from main Makefile
#OVERRIDE_AVRDUDE_FUSES = yes

# XBoot settings

# AVR1008 fixes
# Really only applicable to XMEGA 256a3 rev A and B devices
USE_AVR1008_EEPROM = no

# Entry
USE_ENTER_DELAY = yes
USE_ENTER_PIN = no
USE_ENTER_UART = yes
USE_ENTER_I2C = no
USE_ENTER_FIFO = no

# Exit
LOCK_SPM_ON_EXIT = no

# Communication
USE_LED = yes
USE_UART = yes
USE_UART_EN_PIN = no
USE_I2C = no
USE_I2C_ADDRESS_NEGOTIATION = no
USE_ATTACH_LED = no
USE_FIFO = no

# General Options
USE_INTERRUPTS = no
USE_WATCHDOG = no

# Bootloader Features
ENABLE_BLOCK_SUPPORT = yes
ENABLE_FLASH_BYTE_SUPPORT = yes
ENABLE_EEPROM_BYTE_SUPPORT = yes
ENABLE_LOCK_BITS = yes
ENABLE_FUSE_BITS = yes
ENABLE_FLASH_ERASE_WRITE = yes
ENABLE_CRC_SUPPORT = yes

# API
ENABLE_API = yes
USE_API_VERSION = 1
ENABLE_API_LOW_LEVEL_FLASH = yes
ENABLE_API_SPM_WRAPPER = yes
ENABLE_API_FIRMWARE_UPDATE = yes

# Code Protection
ENABLE_CODE_PROTECTION = no
ENABLE_EEPROM_PROTECTION = no
ENABLE_BOOTLOADER_PROTECTION = no

# ENTER_PIN
ENTER_PORT_NAME       = C
ENTER_PIN             = 0
ENTER_PIN_STATE       = 0
ENTER_PIN_PUEN        = 1

# ENTER_DELAY
ENTER_BLINK_COUNT     = 3
ENTER_BLINK_WAIT      = 30000

# ENTER_UART
#ENTER_UART_NEED_SYNC = yes

# ENTER_FIFO
#ENTER_FIFO_NEED_SYNC = yes

# USE_WATCHDOG
# Select only one
#WATCHDOG_TIMEOUT      = WDT_PER_8CLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_16CLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_32CLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_64CLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_128CLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_256CLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_512CLK_gc
WATCHDOG_TIMEOUT      = WDT_PER_1KCLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_2KCLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_4KCLK_gc
#WATCHDOG_TIMEOUT      = WDT_PER_8KCLK_gc

# LED
LED_PORT_NAME         = A
LED_PIN               = 0
LED_INV               = 1

# UART
# Select BAUD rate, port name, and UART number
UART_BAUD_RATE        = 115200
UART_PORT_NAME        = D
UART_NUMBER           = 1
UART_RX_PUEN          = yes

# UART RS485 Enable Output
UART_EN_PORT_NAME     = C
UART_EN_PIN           = 4
UART_EN_PIN_INV       = 0

# FIFO
FIFO_DATA_PORT_NAME   = C
FIFO_CTL_PORT_NAME    = D
FIFO_RXF_N            = 3
FIFO_TXE_N            = 2
FIFO_RD_N             = 1
FIFO_WR_N             = 0
FIFO_BIT_REVERSE      = yes

# I2C
I2C_DEVICE_PORT       = C

I2C_MATCH_ANY         = 1
I2C_ADDRESS           = 0x10
I2C_GC_ENABLE         = 1

# I2C Address Autonegotiation
# Note: only works on XMega chips for the time being
# There is no easy way to get this to work on regular
# ATMega chips as they have no unique part ID number
I2C_AUTONEG_DIS_PROMISC       = 1
I2C_AUTONEG_DIS_GC            = 0
I2C_AUTONEG_PORT_NAME         = A
I2C_AUTONEG_PIN               = 2

# Attach LED
ATTACH_LED_PORT_NAME          = A
ATTACH_LED_PIN                = 1
ATTACH_LED_INV                = 1



