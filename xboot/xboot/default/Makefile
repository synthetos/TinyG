###############################################################################
# Makefile for the project xboot
###############################################################################

## General Flags
PROJECT = xboot
MCU = atxmega192a3
TARGET = xboot.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99 -fdata-sections  -DUSE_CONFIG_H  -ffunction-sections           -DF_CPU=32000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS += -Wl,-Map=xboot.map,--cref -Wl,--gc-sections -Wl,--section-start=.text=0x030000 -Wl,-Map=xboot.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link
OBJECTS = api.o eeprom_driver.o fifo.o flash.o i2c.o sp_driver.o uart.o watchdog.o xboot.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) xboot.hex xboot.eep xboot.lss size

## Compile
api.o: ../api.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

eeprom_driver.o: ../eeprom_driver.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

fifo.o: ../fifo.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

flash.o: ../flash.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

i2c.o: ../i2c.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

sp_driver.o: ../sp_driver.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

uart.o: ../uart.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

watchdog.o: ../watchdog.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xboot.o: ../xboot.c
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
	-rm -rf $(OBJECTS) xboot.elf dep/* xboot.hex xboot.eep xboot.lss xboot.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

