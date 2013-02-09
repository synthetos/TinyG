# XBoot Configuration for Arduino (atmega328p)
# http://www.arduino.cc/

# use config.h
USE_CONFIG_H = yes

# MCU
MCU = atmega328p

# Select boot size
# Note: if boot size is too small, XBoot may not fit.
# Generally, it should be left on largest
# See part datasheet for specific values
# Largest
BOOTSZ=0
# Large
#BOOTSZ=1
# Medium
#BOOTSZ=2
# Small
#BOOTSZ=3

# Clock Speed
# Use 16 MHz external crystal oscillator
F_CPU = 16000000

# Programmer settings
OVERRIDE_AVRDUDE_PROGRAMMER = yes
AVRDUDE_PROGRAMMER = jtag2isp
AVRDUDE_PORT = usb

# Fuse settings
AVRDUDE_FUSES =
# If you wish to override the default fuse settings
# determined in main Makefile, change them here
# and then uncomment OVERRIDE_AVRDUDE_FUSES

#AVRDUDE_FUSES += -U lfuse:w:0xFF:m
#AVRDUDE_FUSES += -U hfuse:w:0xFF:m
#AVRDUDE_FUSES += -U efuse:w:0xFF:m
#AVRDUDE_FUSES += -U lock:w:0xFF:m

# Uncomment to override default fuse configurations
# from main Makefile
#OVERRIDE_AVRDUDE_FUSES = yes

# XBoot settings

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
ENTER_BLINK_WAIT      = 300000

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
LED_PORT_NAME         = B
LED_PIN               = 5
LED_INV               = 1

# UART
# Select BAUD rate and UART number
# For ATMEGA, uart number is usually 0
# UART_U2X will double clock rate for ATMEGA
# Needed for high baud rates
UART_BAUD_RATE        = 115200
UART_NUMBER           = 0
UART_U2X              = yes
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



