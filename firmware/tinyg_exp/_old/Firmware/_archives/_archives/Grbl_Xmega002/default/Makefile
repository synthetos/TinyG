###############################################################################
# Makefile for the project Grbl_Xmega002
###############################################################################

## General Flags
PROJECT = Grbl_Xmega002
MCU = atxmega256a3
TARGET = Grbl_Xmega002.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99   -DF_CPU=32000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=Grbl_Xmega002.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Libraries
LIBS = -lm 

## Objects that must be built in order to link
OBJECTS = eeprom_xmega.o config.o gcode.o wiring_serial.o main.o motion_control.o serial_protocol.o spindle_control.o stepper.o xmega_support.o encoder.o pmic_driver.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) Grbl_Xmega002.hex Grbl_Xmega002.eep Grbl_Xmega002.lss size

## Compile
eeprom_xmega.o: ../eeprom_xmega.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

config.o: ../config.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

gcode.o: ../gcode.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

wiring_serial.o: ../wiring_serial.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

main.o: ../main.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

motion_control.o: ../motion_control.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

serial_protocol.o: ../serial_protocol.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

spindle_control.o: ../spindle_control.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

stepper.o: ../stepper.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xmega_support.o: ../xmega_support.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

encoder.o: ../encoder.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

pmic_driver.o: ../pmic_driver.c
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
	-rm -rf $(OBJECTS) Grbl_Xmega002.elf dep/* Grbl_Xmega002.hex Grbl_Xmega002.eep Grbl_Xmega002.lss Grbl_Xmega002.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

