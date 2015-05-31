###############################################################################
# Makefile for the project tinyg
###############################################################################

## General Flags
PROJECT = tinyg
MCU = atxmega192a3
TARGET = tinyg.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
<<<<<<< HEAD:firmware/tinyg/default/Makefile
CFLAGS += -gdwarf-2 -std=gnu99 -Wall                         -DF_CPU=32000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
=======
CFLAGS += -gdwarf-2 -std=gnu99 -Wall        -DF_CPU=32000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
>>>>>>> refs/heads/edge:firmware/tinyg/default/Makefile
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS += -Wl,-u,vfprintf  -lprintf_flt  -lm -Wl,-Map=tinyg.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Libraries
LIBS = -lm 

## Objects that must be built in order to link
<<<<<<< HEAD:firmware/tinyg/default/Makefile
OBJECTS = util.o canonical_machine.o config.o controller.o cycle_homing.o gcode_parser.o gpio.o help.o json_parser.o kinematics.o main.o planner.o report.o spindle.o stepper.o system.o test.o xmega_rtc.o xmega_eeprom.o xmega_init.o xmega_interrupts.o xio_usb.o xio.o xio_pgm.o xio_rs485.o xio_usart.o pwm.o plan_line.o plan_arc.o xio_spi.o xio_file.o network.o 
=======
OBJECTS = util.o canonical_machine.o config.o controller.o cycle_homing.o gcode_parser.o gpio.o help.o json_parser.o kinematics.o main.o planner.o report.o spindle.o stepper.o hardware.o test.o xmega_rtc.o xmega_eeprom.o xmega_init.o xmega_interrupts.o xio_usb.o xio_pgm.o xio_rs485.o xio_usart.o pwm.o plan_line.o plan_arc.o xio_spi.o xio_file.o network.o config_app.o text_parser.o switch.o cycle_probing.o xio.o cycle_jogging.o plan_exec.o encoder.o plan_zoid.o persistence.o 
>>>>>>> refs/heads/edge:firmware/tinyg/default/Makefile

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) tinyg.hex tinyg.eep tinyg.lss

## Compile
util.o: ../util.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

canonical_machine.o: ../canonical_machine.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

config.o: ../config.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

controller.o: ../controller.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

cycle_homing.o: ../cycle_homing.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

gcode_parser.o: ../gcode_parser.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

gpio.o: ../gpio.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

help.o: ../help.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

json_parser.o: ../json_parser.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

kinematics.o: ../kinematics.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

main.o: ../main.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

planner.o: ../planner.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

report.o: ../report.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

spindle.o: ../spindle.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

stepper.o: ../stepper.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

hardware.o: ../hardware.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

test.o: ../test.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xmega_rtc.o: ../xmega/xmega_rtc.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xmega_eeprom.o: ../xmega/xmega_eeprom.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xmega_init.o: ../xmega/xmega_init.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xmega_interrupts.o: ../xmega/xmega_interrupts.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xio_usb.o: ../xio/xio_usb.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xio_pgm.o: ../xio/xio_pgm.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xio_rs485.o: ../xio/xio_rs485.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xio_usart.o: ../xio/xio_usart.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

pwm.o: ../pwm.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

plan_line.o: ../plan_line.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

plan_arc.o: ../plan_arc.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xio_spi.o: ../xio/xio_spi.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xio_file.o: ../xio/xio_file.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

network.o: ../network.c
<<<<<<< HEAD:firmware/tinyg/default/Makefile
=======
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

config_app.o: ../config_app.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

text_parser.o: ../text_parser.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

switch.o: ../switch.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

cycle_probing.o: ../cycle_probing.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

xio.o: ../xio.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

cycle_jogging.o: ../cycle_jogging.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

plan_exec.o: ../plan_exec.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

encoder.o: ../encoder.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

plan_zoid.o: ../plan_zoid.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

persistence.o: ../persistence.c
>>>>>>> refs/heads/edge:firmware/tinyg/default/Makefile
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
	-rm -rf $(OBJECTS) tinyg.elf dep/* tinyg.hex tinyg.eep tinyg.lss tinyg.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

