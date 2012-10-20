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
MCU = atxmega256a3
## MCU = atxmega256a3b
## MCU = atxmega64a3

# Is this a bootloader?
#MAKE_BOOTLOADER=no
MAKE_BOOTLOADER=yes

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
#F_CPU=2000000
F_CPU=32000000

# for xboot automated configuration - overrides MCU and F_CPU if present
-include xboot-config.mk

# Preprocessor defines
DEFINES = -DF_CPU=$(F_CPU)L

# Output format. (can be srec, ihex, binary)
FORMAT = ihex
#FORMAT = srec

# Target file name (without extension).
TARGET = xboot


# List C source files here. (C dependencies are automatically generated.)
SRC = $(TARGET).c
SRC += eeprom_driver.c
SRC += uart.c
SRC += i2c.c
SRC += fifo.c
SRC += watchdog.c
# SRC += ...

# List Assembler source files here.
# Make them always end in a capital .S.  Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
ASRC = sp_driver.S
# ASRC += ...

# Optimization level, can be [0, 1, 2, 3, s].
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = s

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
# DEBUG = stabs
DEBUG = dwarf-2

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
CFLAGS = -g$(DEBUG)
CFLAGS += $(CDEFS) $(CINCS)
CFLAGS += -O$(OPT)
CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -Wall -Wstrict-prototypes
CFLAGS += -Wa,-adhlns=$(<:.c=.lst)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
CFLAGS += $(CSTANDARD)



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
LDFLAGS += $(EXTMEMOPTS)
LDFLAGS += $(PRINTF_LIB) $(SCANF_LIB) $(MATH_LIB)

# Programming support using avrdude. Settings and variables.

# Programming hardware: alf avr910 avrisp bascom bsd
# dt006 pavr picoweb pony-stk200 sp12 stk200 stk500
#
# Type: avrdude -c ?
# to get a full listing.
#
AVRDUDE_PROGRAMMER = jtag2pdi
#AVRDUDE_PROGRAMMER = avr109

# Port
# com1 = serial port. Use lpt1 to connect to parallel port.
# Use usb for usb devices
# For *nix, need device path (/dev/ttyUSBn, /dev/ttySn)
AVRDUDE_PORT = usb
#AVRDUDE_PORT = /dev/ttyUSB0

# BAUD Rate
#AVRDUDE_BAUD = 19200

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
#AVRDUDE_CHIP_ERASE = -e
# Disable automatic chip erase
#AVRDUDE_CHIP_ERASE = -D

# Fuses and Lock Bits
# See XMega A series datasheet (Atmel doc8077) section 4.16
AVRDUDE_FUSES =

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

AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)
ifdef AVRDUDE_BAUD
  AVRDUDE_FLAGS += -b $(AVRDUDE_BAUD)
endif
AVRDUDE_FLAGS += $(AVRDUDE_NO_VERIFY)
AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS += $(AVRDUDE_ERASE_COUNTER)
AVRDUDE_FLAGS += $(AVRDUDE_CHIP_ERASE)

ifeq ($(MAKE_BOOTLOADER), yes)
ifeq ($(PROG_BOOT_ONLY), yes)
  BOOT_TARGET=$(TARGET)-boot.hex
  AVRDUDE_WRITE_FLASH = -U boot:w:$(TARGET)-boot.hex
endif
endif

# ---------------------------------------------------------------------------

# Processor definitions
ifeq ($(MCU), atxmega16a4)
  BOOT_SECTION_START		=0x004000
endif
ifeq ($(MCU), atxmega32a4)
  BOOT_SECTION_START		=0x008000
endif
ifeq ($(MCU), atxmega64a1)
  BOOT_SECTION_START		=0x010000
endif
ifeq ($(MCU), atxmega64a3)
  BOOT_SECTION_START		=0x010000
endif
ifeq ($(MCU), atxmega64a4)
  BOOT_SECTION_START		=0x010000
endif
ifeq ($(MCU), atxmega128a1)
  BOOT_SECTION_START		=0x020000
endif
ifeq ($(MCU), atxmega128a3)
  BOOT_SECTION_START		=0x020000
endif
ifeq ($(MCU), atxmega128a4)
  BOOT_SECTION_START		=0x020000
endif
ifeq ($(MCU), atxmega192a1)
  BOOT_SECTION_START		=0x030000
endif
ifeq ($(MCU), atxmega192a3)
  BOOT_SECTION_START		=0x030000
endif
ifeq ($(MCU), atxmega256a1)
  BOOT_SECTION_START		=0x040000
endif
ifeq ($(MCU), atxmega256a3)
  BOOT_SECTION_START		=0x040000
endif
ifeq ($(MCU), atxmega256a3b)
  BOOT_SECTION_START		=0x040000
endif

ifeq ($(MAKE_BOOTLOADER), yes)
# BOOT_SECTION_START (=Start of Boot Loader section
# in bytes - not words) as defined above.
LDFLAGS += -Wl,--section-start=.text=$(BOOT_SECTION_START)
endif

# ---------------------------------------------------------------------------

# Define directories, if needed.
#DIRAVR = c:/winavr
#DIRAVRBIN = $(DIRAVR)/bin
#DIRAVRUTILS = $(DIRAVR)/utils/bin
#DIRINC = .
#DIRLIB = $(DIRAVR)/avr/lib


# Define programs and commands.
#SHELL = $(DIRAVRUTILS)/sh
#NM = $(DIRAVRBIN)/avr-nm
#CC = $(DIRAVRBIN)/avr-gcc
#OBJCOPY = $(DIRAVRBIN)/avr-objcopy
#OBJDUMP= $(DIRAVRBIN)/avr-objdump
#SIZE = $(DIRAVRBIN)/avr-size
#AVRDUDE = $(DIRAVRBIN)/avrdude.sh
#REMOVE = rm -f
#COPY = cp

# Define programs and commands.
SHELL = sh
CC = avr-gcc
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


# Define all object files.
OBJ = $(SRC:.c=.o) $(ASRC:.S=.o)

# Define all listing files.
LST = $(ASRC:.S=.lst) $(SRC:.c=.lst)

# Compiler flags to generate dependency files.
### GENDEPFLAGS = -Wp,-M,-MP,-MT,$(*F).o,-MF,.dep/$(@F).d
GENDEPFLAGS = -MD -MP -MF .dep/$(@F).d

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS) $(GENDEPFLAGS) $(DEFINES)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS) $(DEFINES)



# Default target.
all: begin gccversion sizebefore build sizeafter finished end

build: elf hex eep lss sym

elf: $(TARGET).elf
hex: $(TARGET).hex $(BOOT_TARGET)
eep: $(TARGET).eep
lss: $(TARGET).lss
sym: $(TARGET).sym



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
%.elf: $(OBJ)
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(ALL_CFLAGS) $(OBJ) --output $@ $(LDFLAGS)


# Compile: create object files from C source files.
%.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(ALL_CFLAGS) $< -o $@


# Compile: create assembler files from C source files.
%.s : %.c
	$(CC) -S $(ALL_CFLAGS) $< -o $@


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
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).a90
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lnk
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(OBJ)
	$(REMOVE) $(LST)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRC:.c=.d)
	$(REMOVE) .dep/*



# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)


# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex eep lss sym coff extcoff \
clean clean_list program

