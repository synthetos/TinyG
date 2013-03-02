# Hey Emacs, this is a -*- makefile -*-
#
# General-purpose makefile for ATMEL XMEGA
#
# based on the
# WinAVR Sample makefile written by Eric B. Weddington, Jörg Wunsch, et al.
# Released to the Public Domain
# Please read the make user manual!
#
# Additional material for this makefile was submitted by:
#  Tim Henigan
#  Peter Fleury
#  Reiner Patommel
#  Sander Pool
#  Frederik Rouleau
#  Markus Pfaff
#
# Updated for XMEGA by:
#  Alex Forencich
#
# On command line:
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# make coff = Convert ELF to AVR COFF (for use with AVR Studio 3.x or VMLAB).
#
# make extcoff = Convert ELF to AVR Extended COFF (for use with AVR Studio
#                4.07 or greater).
#
# make program = Download the hex file to the device, using avrdude.  Please
#                customize the avrdude settings below first!
#
# make filename.s = Just compile filename.c into the assembler code only
#
# To rebuild project do "make clean" then "make all".
#

# user defined values

# MCU name
# XMEGA
## MCU = atxmega16a4
## MCU = atxmega32a4
## MCU = atxmega64a1
## MCU = atxmega64a3
## MCU = atxmega64a4
## MCU = atxmega128a1
## MCU = atxmega128a3
## MCU = atxmega128a4
## MCU = atxmega192a1
## MCU = atxmega192a3
## MCU = atxmega256a1
## MCU = atxmega256a3
## MCU = atxmega256a3b
## MCU = atxmega16d4
## MCU = atxmega32d4
## MCU = atxmega64d3
## MCU = atxmega64d4
## MCU = atxmega128d3
## MCU = atxmega128d4
## MCU = atxmega192d3
## MCU = atxmega256d3
## MCU = atxmega16a4u
## MCU = atxmega32a4u
## MCU = atxmega64a3u
## MCU = atxmega64a4u
## MCU = atxmega128a3u
## MCU = atxmega128a4u
## MCU = atxmega192a3u
## MCU = atxmega256a3u
## MCU = atxmega256a3bu
## MCU = atxmega64b1
## MCU = atxmega64b3
## MCU = atxmega128b1
## MCU = atxmega128b3
# ATMEGA
## MCU = atmega88
## MCU = atmega88p
## MCU = atmega88pa
## MCU = atmega164
## MCU = atmega168
## MCU = atmega168p
## MCU = atmega169
## MCU = atmega169p
## MCU = atmega169pa
## MCU = atmega324
## MCU = atmega324pa
## MCU = atmega325
## MCU = atmega3250
## MCU = atmega328p
## MCU = atmega329
## MCU = atmega3290
## MCU = atmega64
## MCU = atmega640
## MCU = atmega644
## MCU = atmega644p
## MCU = atmega644pa
## MCU = atmega645
## MCU = atmega6450
## MCU = atmega649
## MCU = atmega649p
## MCU = atmega6490
## MCU = atmega128
## MCU = atmega1280
## MCU = atmega1281
## MCU = atmega1284p
## MCU = atmega2560
## MCU = atmega2561
MCU = atxmega64a3

# Is this a bootloader?
#MAKE_BOOTLOADER=no
MAKE_BOOTLOADER=yes

# Select boot size (ATmega only)
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

# Only program boot section
# (XMega only)
# This will create a target-boot.hex file with the program relocated to
# address 0 and then program the file directly to the boot section.  It
# is faster than programming the entire application section with
# nothing and has the added advantage of leaving the application
# section in tact
# Note: ignored if MAKE_BOOTLOADER is not set
PROG_BOOT_ONLY=yes

# CPU Frequency
F_CPU=2000000
#F_CPU=32000000
#F_CPU=16000000

# Configuration support
-include config.mk

# Is this an xmega?
ifneq ($(findstring atxmega,$(MCU)),)
  XMEGA=yes
endif

# Preprocessor defines
DEFINES = -DF_CPU=$(F_CPU)L

# Output format. (can be srec, ihex, binary)
FORMAT = ihex
#FORMAT = srec

# Target file name (without extension).
TARGET = xboot


# List C source files here. (C dependencies are automatically generated.)
SRC = $(TARGET).c
SRC += flash.c
SRC += eeprom_driver.c
SRC += uart.c
SRC += i2c.c
SRC += fifo.c
SRC += watchdog.c
SRC += api.c
# SRC += ...

# List Assembler source files here.
# Make them always end in a capital .S.  Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
ASRC =
ifeq ($(XMEGA), yes)
# Xmega specific SP driver
ASRC += sp_driver.S
endif
# ASRC += ...

# Optimization level, can be [0, 1, 2, 3, s].
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = s

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
DEBUG = stabs

# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
EXTRAINCDIRS =


# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99

# Place -D or -U options here
#CDEFS = -DBOOTSIZE=$(BOOTSIZE)
CDEFS = 

# Place -I options here
CINCS =


# Compiler flags.
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
COMMON_FLAGS = -g$(DEBUG)
COMMON_FLAGS += $(CDEFS) $(CINCS)
COMMON_FLAGS += -O$(OPT)
COMMON_FLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
COMMON_FLAGS += -ffunction-sections -fdata-sections

ifeq ($(MAKE_BOOTLOADER), yes)
# Good idea for devices with large flash memory
COMMON_FLAGS += -fno-jump-tables
endif

COMMON_FLAGS += -Wall
COMMON_FLAGS += -Wa,-adhlns=$(basename $<).lst
COMMON_FLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
#COMMON_FLAGS += ...

# C Specific flags
CFLAGS = $(COMMON_FLAGS)
CFLAGS += -Wstrict-prototypes
CFLAGS += $(CSTANDARD)
#CFLAGS += ...

# C++ Specific flags
CXXFLAGS = $(COMMON_FLAGS)
#CXXFLAGS += ...



# Assembler flags.
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -ahlms:    create listing
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs



#Additional libraries.

# Minimalistic printf version
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min

# Floating point printf version (requires MATH_LIB = -lm below)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt

PRINTF_LIB =

# Minimalistic scanf version
SCANF_LIB_MIN = -Wl,-u,vfscanf -lscanf_min

# Floating point + %[ scanf version (requires MATH_LIB = -lm below)
SCANF_LIB_FLOAT = -Wl,-u,vfscanf -lscanf_flt

SCANF_LIB =

MATH_LIB = -lm

# External memory options

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# used for variables (.data/.bss) and heap (malloc()).
#EXTMEMOPTS = -Wl,-Tdata=0x801100,--defsym=__heap_end=0x80ffff

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# only used for heap (malloc()).
#EXTMEMOPTS = -Wl,--defsym=__heap_start=0x801100,--defsym=__heap_end=0x80ffff

EXTMEMOPTS =

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += -Wl,--gc-sections
LDFLAGS += $(EXTMEMOPTS)
LDFLAGS += $(PRINTF_LIB) $(SCANF_LIB) $(MATH_LIB)

# Programming support using avrdude. Settings and variables.
ifeq ($(OVERRIDE_AVRDUDE_PROGRAMMER),)
# Programming hardware: alf avr910 avrisp bascom bsd
# dt006 pavr picoweb pony-stk200 sp12 stk200 stk500
#
# Type: avrdude -c ?
# to get a full listing.
#
@AVRDUDE_PROGRAMMER = jtag2pdi
#AVRDUDE_PROGRAMMER = jtag2isp
AVRDUDE_PROGRAMMER = avrispmkii
#AVRDUDE_PROGRAMMER = avr109

# Port
# com1 = serial port. Use lpt1 to connect to parallel port.
# Use usb for usb devices
# For *nix, need device path (/dev/ttyUSBn, /dev/ttySn)
AVRDUDE_PORT = usb
#AVRDUDE_PORT = /dev/ttyUSB0

# BAUD Rate
@AVRDUDE_BAUD = 115200
endif

# Sections to write
AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
#AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TARGET).eep

# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE_COUNTER = -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the device.
#AVRDUDE_NO_VERIFY = -V

# Increase verbosity level.  Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude>
# to submit bug reports.
#AVRDUDE_VERBOSE = -v -v

# Erase behaviour
# Normaly with an XMega, the programmer should automatically erase a 
# page before flashing. 
# It seems that at least the avrdragon_jtag (and probably other types) 
# don't do so(Feb 11)
# In that case, erase the chip before flashing
# Force chip erase
AVRDUDE_CHIP_ERASE = -e
# Disable automatic chip erase
#AVRDUDE_CHIP_ERASE = -D

# Fuses and Lock Bits

ifeq ($(OVERRIDE_AVRDUDE_FUSES),)
ifeq ($(XMEGA), yes)

# XMEGA
AVRDUDE_FUSES =

# See XMega A series datasheet (Atmel doc8077) section 4.16

# Resest Configuration (for fuse byte 2)
# If a custom configuration is needed, please
# override farther down in the fuse byte 2 section
ifeq ($(MAKE_BOOTLOADER), yes)
# Reset (Bootloader)
AVRDUDE_FUSES_RESET_CONFIG = -U fuse2:w:0xBF:m
else
# Reset (Regular)
AVRDUDE_FUSES_RESET_CONFIG = -U fuse2:w:0xFF:m
endif

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
# If a custom configuration is needed, please
# override it here
AVRDUDE_FUSES += $(AVRDUDE_FUSES_RESET_CONFIG)

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

else # xmega

# ATMEGA
AVRDUDE_FUSES =

# Fuses for ATmega devices
MCU_S = $(subst atmega,m,$(MCU))
ifneq ($(filter $(MCU_S), m88 m88p m88pa m168 m168p),)
  ifeq ($(BOOTSZ), 0)
    BOOT_FUSE_NOBL	= -U efuse:w:0x01:m
    BOOT_FUSE_BL	= -U efuse:w:0x00:m
  endif
  ifeq ($(BOOTSZ), 1)
    BOOT_FUSE_NOBL	= -U efuse:w:0x03:m
    BOOT_FUSE_BL	= -U efuse:w:0x02:m
  endif
  ifeq ($(BOOTSZ), 2)
    BOOT_FUSE_NOBL	= -U efuse:w:0x05:m
    BOOT_FUSE_BL	= -U efuse:w:0x04:m
  endif
  ifeq ($(BOOTSZ), 3)
    BOOT_FUSE_NOBL	= -U efuse:w:0x07:m
    BOOT_FUSE_BL	= -U efuse:w:0x06:m
  endif
endif
ifneq ($(filter $(MCU_S), m328p),)
  ifeq ($(BOOTSZ), 0)
    BOOT_FUSE_NOBL	= -U hfuse:w:0xD9:m
    BOOT_FUSE_BL	= -U hfuse:w:0xD8:m
  endif
  ifeq ($(BOOTSZ), 1)
    BOOT_FUSE_NOBL	= -U hfuse:w:0xDB:m
    BOOT_FUSE_BL	= -U hfuse:w:0xDA:m
  endif
  ifeq ($(BOOTSZ), 2)
    BOOT_FUSE_NOBL	= -U hfuse:w:0xDD:m
    BOOT_FUSE_BL	= -U hfuse:w:0xDC:m
  endif
  ifeq ($(BOOTSZ), 3)
    BOOT_FUSE_NOBL	= -U hfuse:w:0xDF:m
    BOOT_FUSE_BL	= -U hfuse:w:0xDE:m
  endif
endif
ifneq ($(filter $(MCU_S), m164 m169 m169p m169pa m324 m324pa m325 m3250 m329 m3290 m64 m640 m644 m644p m644pa m645 m6450 m649 m649p m6490 m128 m1280 m1281 m1284p m2560 m2561),)
  ifeq ($(BOOTSZ), 0)
    BOOT_FUSE_NOBL	= -U hfuse:w:0x99:m
    BOOT_FUSE_BL	= -U hfuse:w:0x98:m
  endif
  ifeq ($(BOOTSZ), 1)
    BOOT_FUSE_NOBL	= -U hfuse:w:0x9B:m
    BOOT_FUSE_BL	= -U hfuse:w:0x9A:m
  endif
  ifeq ($(BOOTSZ), 2)
    BOOT_FUSE_NOBL	= -U hfuse:w:0x9D:m
    BOOT_FUSE_BL	= -U hfuse:w:0x9C:m
  endif
  ifeq ($(BOOTSZ), 3)
    BOOT_FUSE_NOBL	= -U hfuse:w:0x9F:m
    BOOT_FUSE_BL	= -U hfuse:w:0x9E:m
  endif
endif

ifeq ($(MAKE_BOOTLOADER), yes)
# Boot config (Bootloader)
AVRDUDE_FUSES_BOOT_CONFIG = $(BOOT_FUSE_BL)
else
# Boot config (Regular)
AVRDUDE_FUSES_BOOT_CONFIG = $(BOOT_FUSE_NOBL)
endif

#AVRDUDE_FUSES += -U lfuse:w:0xFF:m
#AVRDUDE_FUSES += -U hfuse:w:0xFF:m
#AVRDUDE_FUSES += -U efuse:w:0xFF:m

AVRDUDE_FUSES += $(AVRDUDE_FUSES_BOOT_CONFIG)

endif # xmega
endif # override fuses

ifeq ($(AVRDUDE_FUSES),)
  $(warning AVRDUDE will not change any fuses.  Please double check the fuse configuration to ensure proper startup.)
endif

AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)
ifdef AVRDUDE_BAUD
  AVRDUDE_FLAGS += -b $(AVRDUDE_BAUD)
endif
AVRDUDE_FLAGS += $(AVRDUDE_NO_VERIFY)
AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS += $(AVRDUDE_ERASE_COUNTER)
AVRDUDE_FLAGS += $(AVRDUDE_CHIP_ERASE)

ifeq ($(MAKE_BOOTLOADER), yes)
ifeq ($(XMEGA), yes)
ifeq ($(PROG_BOOT_ONLY), yes)
  BOOT_TARGET=$(TARGET)-boot.hex
  AVRDUDE_WRITE_FLASH = -U boot:w:$(TARGET)-boot.hex
endif
endif
endif

# ---------------------------------------------------------------------------

# Processor definitions
# Sizes in bytes, not works (datasheet generally in words)
# xmega
MCU_S = $(subst atxmega,x,$(MCU))
ifneq ($(filter $(MCU_S), x16a4 x16d4 x16a4u),)
  BOOT_SECTION_START		=0x004000
endif
ifneq ($(filter $(MCU_S), x32a4 x32d4 x32a4u),)
  BOOT_SECTION_START		=0x008000
endif
ifneq ($(filter $(MCU_S), x64a1 x64a3 x64a4 x64d3 x64d4 x64a3u x64a4u x64b1 x64b3),)
  BOOT_SECTION_START		=0x010000
endif
ifneq ($(filter $(MCU_S), x128a1 x128a3 x128a4 x128d3 x128d4 x128a3u x128a4u x128b1 x128b3),)
  BOOT_SECTION_START		=0x020000
endif
ifneq ($(filter $(MCU_S), x192a1 x192a3 x192d3 x192a3u),)
  BOOT_SECTION_START		=0x030000
endif
ifneq ($(filter $(MCU_S), x256a1 x256a3 x256a3b x256d3 x256a3u x256a3bu),)
  BOOT_SECTION_START		=0x040000
endif
# atmega
MCU_S = $(subst atmega,m,$(MCU))
ifneq ($(filter $(MCU_S), m88 m88p m88pa),)
  ifeq ($(BOOTSZ), 0)
    BOOT_SECTION_START		=0x001800
    BOOT_SECTION_SIZE		=0x000800
  endif
  ifeq ($(BOOTSZ), 1)
    BOOT_SECTION_START		=0x001C00
    BOOT_SECTION_SIZE		=0x000400
  endif
  ifeq ($(BOOTSZ), 2)
    BOOT_SECTION_START		=0x001E00
    BOOT_SECTION_SIZE		=0x000200
  endif
  ifeq ($(BOOTSZ), 3)
    BOOT_SECTION_START		=0x001F00
    BOOT_SECTION_SIZE		=0x000100
  endif
endif
ifneq ($(filter $(MCU_S), m164 m168 m168p),)
  ifeq ($(BOOTSZ), 0)
    BOOT_SECTION_START		=0x003800
    BOOT_SECTION_SIZE		=0x000800
  endif
  ifeq ($(BOOTSZ), 1)
    BOOT_SECTION_START		=0x003C00
    BOOT_SECTION_SIZE		=0x000400
  endif
  ifeq ($(BOOTSZ), 2)
    BOOT_SECTION_START		=0x003E00
    BOOT_SECTION_SIZE		=0x000200
  endif
  ifeq ($(BOOTSZ), 3)
    BOOT_SECTION_START		=0x003F00
    BOOT_SECTION_SIZE		=0x000100
  endif
endif
ifneq ($(filter $(MCU_S), m324 m324pa m328p m325 m3250 m329 m3290),)
  ifeq ($(BOOTSZ), 0)
    BOOT_SECTION_START		=0x007000
    BOOT_SECTION_SIZE		=0x001000
  endif
  ifeq ($(BOOTSZ), 1)
    BOOT_SECTION_START		=0x007800
    BOOT_SECTION_SIZE		=0x000800
  endif
  ifeq ($(BOOTSZ), 2)
    BOOT_SECTION_START		=0x007C00
    BOOT_SECTION_SIZE		=0x000400
  endif
  ifeq ($(BOOTSZ), 3)
    BOOT_SECTION_START		=0x007E00
    BOOT_SECTION_SIZE		=0x000200
  endif
endif
ifneq ($(filter $(MCU_S), m64 m640 m644 m644p m644pa m645 m6450 m649 m649p m6490),)
  ifeq ($(BOOTSZ), 0)
    BOOT_SECTION_START		=0x00E000
    BOOT_SECTION_SIZE		=0x002000
  endif
  ifeq ($(BOOTSZ), 1)
    BOOT_SECTION_START		=0x00F000
    BOOT_SECTION_SIZE		=0x001000
  endif
  ifeq ($(BOOTSZ), 2)
    BOOT_SECTION_START		=0x00F800
    BOOT_SECTION_SIZE		=0x000800
  endif
  ifeq ($(BOOTSZ), 3)
    BOOT_SECTION_START		=0x00FC00
    BOOT_SECTION_SIZE		=0x000400
  endif
endif
ifneq ($(filter $(MCU_S), m128 m1280 m1281 m1284p),)
  ifeq ($(BOOTSZ), 0)
    BOOT_SECTION_START		=0x01E000
    BOOT_SECTION_SIZE		=0x002000
  endif
  ifeq ($(BOOTSZ), 1)
    BOOT_SECTION_START		=0x01F000
    BOOT_SECTION_SIZE		=0x001000
  endif
  ifeq ($(BOOTSZ), 2)
    BOOT_SECTION_START		=0x01F800
    BOOT_SECTION_SIZE		=0x000800
  endif
  ifeq ($(BOOTSZ), 3)
    BOOT_SECTION_START		=0x01FC00
    BOOT_SECTION_SIZE		=0x000400
  endif
endif
ifneq ($(filter $(MCU_S), m2560 m2561),)
  ifeq ($(BOOTSZ), 0)
    BOOT_SECTION_START		=0x03E000
    BOOT_SECTION_SIZE		=0x002000
  endif
  ifeq ($(BOOTSZ), 1)
    BOOT_SECTION_START		=0x03F000
    BOOT_SECTION_SIZE		=0x001000
  endif
  ifeq ($(BOOTSZ), 2)
    BOOT_SECTION_START		=0x03F800
    BOOT_SECTION_SIZE		=0x000800
  endif
  ifeq ($(BOOTSZ), 3)
    BOOT_SECTION_START		=0x03FC00
    BOOT_SECTION_SIZE		=0x000400
  endif
endif

ifeq ($(BOOT_SECTION_START),)
  $(error $(MCU) not found in makefile, BOOT_SECTION_START not defined!)
endif

ifeq ($(MAKE_BOOTLOADER), yes)
# BOOT_SECTION_START (=Start of Boot Loader section
# in bytes - not words) as defined above.
LDFLAGS += -Wl,--section-start=.text=$(BOOT_SECTION_START)
endif

ifneq ($(XMEGA), yes)
DEFINES += -DBOOT_SECTION_SIZE=$(BOOT_SECTION_SIZE)
endif

# ---------------------------------------------------------------------------

# Define directories, if needed.
#DIRAVR = c:/winavr
#DIRAVRBIN = $(DIRAVR)/bin
#DIRAVRUTILS = $(DIRAVR)/utils/bin
#DIRINC = .
#DIRLIB = $(DIRAVR)/avr/lib


# Define programs and commands.
#SHELL = "$(DIRAVRUTILS)/sh"
#NM = "$(DIRAVRBIN)/avr-nm"
#CC = "$(DIRAVRBIN)/avr-gcc"
#CXX = "$(DIRAVRBIN)/avr-g++"
#OBJCOPY = "$(DIRAVRBIN)/avr-objcopy"
#OBJDUMP= "$(DIRAVRBIN)/avr-objdump"
#SIZE = "$(DIRAVRBIN)/avr-size"
#AVRDUDE = "$(DIRAVRBIN)/avrdude.sh"
#REMOVE = rm -f
#COPY = cp

# Define programs and commands.
SHELL = sh
CC = avr-gcc
CXX = avr-g++
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
NM = avr-nm
AVRDUDE = avrdude
REMOVE = rm -f
COPY = cp
WINSHELL = cmd


# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before:
MSG_SIZE_AFTER = Size after:
MSG_COFF = Converting to AVR COFF:
MSG_EXTENDED_COFF = Converting to AVR Extended COFF:
MSG_FLASH = Creating load file for Flash:
MSG_BOOT = Creating load file for boot section:
MSG_EEPROM = Creating load file for EEPROM:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:


# Configuration support
ifeq ($(USE_CONFIG_H), yes)
  CONFIG_H = config.h
  DEFINES += -DUSE_CONFIG_H
endif


# Define all object files.
OBJ = $(addsuffix .o,$(basename $(SRC) $(ASRC)))

# Define all listing files.
LST = $(addsuffix .lst,$(basename $(SRC) $(ASRC)))


# Compiler flags to generate dependency files.
### GENDEPFLAGS = -Wp,-M,-MP,-MT,$(*F).o,-MF,.dep/$(@F).d
GENDEPFLAGS = -MD -MP -MF .dep/$(@F).d

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS) $(GENDEPFLAGS) $(DEFINES)
ALL_CXXFLAGS = -mmcu=$(MCU) -I. $(CXXFLAGS) $(GENDEPFLAGS) $(DEFINES)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS) $(DEFINES)





# Default target.
all: begin gccversion sizebefore build sizeafter finished end

build: elf hex eep lss sym

elf: $(TARGET).elf
hex: $(TARGET).hex $(BOOT_TARGET)
eep: $(TARGET).eep
lss: $(TARGET).lss
sym: $(TARGET).sym


# Configuration support
%.conf %.conf.mk: force
	cp $@ config.mk
	$(MAKE)

-include config.h.mk

# Eye candy.
# AVR Studio 3.x does not check make's exit code but relies on
# the following magic strings to be generated by the compile job.
begin:
	@echo
	@echo $(MSG_BEGIN)

finished:
	@echo $(MSG_ERRORS_NONE)

end:
	@echo $(MSG_END)
	@echo


# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex
ELFSIZE = $(SIZE) -x -A $(TARGET).elf
sizebefore:
	@if [ -f $(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); echo; fi

sizeafter:
	@if [ -f $(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi



# Display compiler version information.
gccversion :
	@$(CC) --version



# Program the device.
program: $(TARGET).hex $(TARGET).eep $(BOOT_TARGET)
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM) $(AVRDUDE_USERSIG) $(AVRDUDE_FUSES)




# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
--change-section-address .data-0x800000 \
--change-section-address .bss-0x800000 \
--change-section-address .noinit-0x800000 \
--change-section-address .eeprom-0x810000


coff: $(TARGET).elf
	@echo
	@echo $(MSG_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-avr $< $(TARGET).cof


extcoff: $(TARGET).elf
	@echo
	@echo $(MSG_EXTENDED_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-ext-avr $< $(TARGET).cof



# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

%-boot.hex: %.hex
	@echo
	@echo $(MSG_BOOT) $@
	$(OBJCOPY) -O $(FORMAT) --change-addresses -$(BOOT_SECTION_START) $< $@

%.eep: %.elf
	@echo
	@echo $(MSG_EEPROM) $@
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@



# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJ)
%.elf: $(CONFIG_H) $(OBJ)
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(ALL_CFLAGS) $(OBJ) --output $@ $(LDFLAGS)


# Compile: create object files from C source files.
%.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(ALL_CFLAGS) $< -o $@


# Compile: create object files from C++ source files.
%.o : %.cpp
	@echo
	@echo $(MSG_COMPILING) $<
	$(CXX) -c $(ALL_CXXFLAGS) $< -o $@


# Compile: create assembler files from C source files.
%.s : %.c
	$(CC) -S $(ALL_CFLAGS) $< -o $@


# Compile: create assembler files from C++ source files.
%.s : %.cpp
	$(CXX) -S $(ALL_CXXFLAGS) $< -o $@


# Assemble: create object files from assembler source files.
%.o : %.S
	@echo
	@echo $(MSG_ASSEMBLING) $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $@



# Target: clean project.
clean: begin clean_list finished end

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET)-boot.hex
	$(REMOVE) $(TARGET).eep
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).cof
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).a90
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lnk
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(OBJ)
	$(REMOVE) $(LST)
	$(REMOVE) config.mk config.h
	$(REMOVE) $(addsuffix .s,$(basename $(SRC)))
	$(REMOVE) $(addsuffix .d,$(basename $(SRC)))
	$(REMOVE) .dep/*



# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)


# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex eep lss sym coff extcoff \
clean clean_list program force

