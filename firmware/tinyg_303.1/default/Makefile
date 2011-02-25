###############################################################################
# Makefile for the project TinyG
###############################################################################

## General Flags
PROJECT = TinyG
MCU = atxmega256a3
TARGET = TinyG.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99                                                                                                                                                                                                        -DF_CPU=32000000UL -O0 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS += -Wl,-u,vfprintf  -lprintf_flt -lm -Wl,-Map=TinyG.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Libraries
LIBS = -lm 

## Objects that must be built in order to link
OBJECTS = main.o config.o encoder.o xmega_init.o spindle.o stepper.o xmega_interrupts.o xio.o xmega_eeprom.o controller.o direct_drive.o motor_queue.o limit_switches.o signals.o xio_usart.o xio_usb.o xio_rs485.o xio_pgm.o network.o system.o canonical_machine.o gcode.o xio_eep.o xmega_rtc.o planner.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) TinyG.hex TinyG.eep TinyG.lss size

## Compile
main.o: ../main.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

config.o: ../config.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

encoder.o: ../encoder.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xmega_init.o: ../xmega_init.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

spindle.o: ../spindle.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

stepper.o: ../stepper.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xmega_interrupts.o: ../xmega_interrupts.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xio.o: ../xio.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xmega_eeprom.o: ../xmega_eeprom.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

controller.o: ../controller.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

direct_drive.o: ../direct_drive.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

motor_queue.o: ../motor_queue.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

limit_switches.o: ../limit_switches.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

signals.o: ../signals.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xio_usart.o: ../xio_usart.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xio_usb.o: ../xio_usb.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xio_rs485.o: ../xio_rs485.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xio_pgm.o: ../xio_pgm.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

network.o: ../network.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

system.o: ../system.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

canonical_machine.o: ../canonical_machine.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

gcode.o: ../gcode.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xio_eep.o: ../xio_eep.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xmega_rtc.o: ../xmega_rtc.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

planner.o: ../planner.c
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
	-rm -rf $(OBJECTS) TinyG.elf dep/* TinyG.hex TinyG.eep TinyG.lss TinyG.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

