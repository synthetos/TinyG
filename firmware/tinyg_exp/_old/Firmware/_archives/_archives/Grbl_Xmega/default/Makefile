###############################################################################
# Makefile for the project Grbl_Xmega
###############################################################################

## General Flags
PROJECT = Grbl_Xmega
MCU = atxmega256a3
TARGET = Grbl_Xmega.elf
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
LDFLAGS +=  -Wl,-Map=Grbl_Xmega.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Libraries
LIBS = -lm 

## Objects that must be built in order to link
OBJECTS = main.o gcode.o config.o wiring_serial.o motion_control.o serial_protocol.o spindle_control.o stepper.o eeprom_driver.o eeprom_xmega.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) Grbl_Xmega.hex Grbl_Xmega.eep Grbl_Xmega.lss size

## Compile
main.o: ../main.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

gcode.o: ../gcode.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

config.o: ../config.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

wiring_serial.o: ../wiring_serial.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

motion_control.o: ../motion_control.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

serial_protocol.o: ../serial_protocol.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

spindle_control.o: ../spindle_control.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

stepper.o: ../stepper.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

eeprom_driver.o: ../eeprom_driver.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

eeprom_xmega.o: ../eeprom_xmega.c
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
	-rm -rf $(OBJECTS) Grbl_Xmega.elf dep/* Grbl_Xmega.hex Grbl_Xmega.eep Grbl_Xmega.lss Grbl_Xmega.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

