###############################################################################
# Makefile for the project Grbl_test_eeprom
###############################################################################

## General Flags
PROJECT = Grbl_test_eeprom
MCU = atxmega256a3
TARGET = Grbl_test_eeprom.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -Os -std=gnu99 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=Grbl_test_eeprom.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link
OBJECTS = Grbl_test_eeprom.o wiring_serial.o eeprom_driver.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) Grbl_test_eeprom.hex Grbl_test_eeprom.eep Grbl_test_eeprom.lss size

## Compile
Grbl_test_eeprom.o: ../Grbl_test_eeprom.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

wiring_serial.o: ../wiring_serial.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

eeprom_driver.o: ../eeprom_driver.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) Grbl_test_eeprom.elf dep/* Grbl_test_eeprom.hex Grbl_test_eeprom.eep Grbl_test_eeprom.lss Grbl_test_eeprom.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

