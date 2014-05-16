###############################################################################
# Makefile for the project tinyg
###############################################################################

## General Flags
PROJECT = tinyg
TARGET = tinyg.elf
CC = gcc

CPP = g++

## Options common to compile, link and assembly rules
COMMON = -Wall -Werror -Wno-unused-value -Wno-unused-variable

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -DTINYG_SIMULATOR
CFLAGS += -gdwarf-2 -std=gnu99 -Wall  -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS += -Wl,-u,vfprintf  -lm -Wl,-Map=tinyg.map

## Libraries
LIBS = -lm 

## Objects that must be built in order to link
OBJECTS = util.o canonical_machine.o config.o controller.o cycle_homing.o gcode_parser.o gpio.o help.o json_parser.o kinematics.o main.o planner.o report.o spindle.o hardware.o test.o pwm.o plan_line.o plan_arc.o config_app.o text_parser.o cycle_probing.o cycle_jogging.o plan_exec.o encoder.o sim.o 
## Note: excluded objects are: stepper.o xmega_rtc.o xmega_eeprom.o xmega_init.o xmega_interrupts.o xio_usb.o xio_pgm.o xio_rs485.o xio_usart.o xio_spi.o xio_file.o network.o switch.o xio.o persistence.o
## Note: added objects: sim.o

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) size

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

#stepper.o: ../stepper.c
#	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

hardware.o: ../hardware.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

test.o: ../test.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

#xmega_rtc.o: ../xmega/xmega_rtc.c
#	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

#xmega_eeprom.o: ../xmega/xmega_eeprom.c
#	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

#xmega_init.o: ../xmega/xmega_init.c
#	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

#xmega_interrupts.o: ../xmega/xmega_interrupts.c
#	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

#xio_usb.o: ../xio/xio_usb.c
#	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

#xio_pgm.o: ../xio/xio_pgm.c
#	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

#xio_rs485.o: ../xio/xio_rs485.c
#	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

#xio_usart.o: ../xio/xio_usart.c
#	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

pwm.o: ../pwm.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

plan_line.o: ../plan_line.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

plan_arc.o: ../plan_arc.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

#xio_spi.o: ../xio/xio_spi.c
#	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

#xio_file.o: ../xio/xio_file.c
#	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

#network.o: ../network.c
#	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

config_app.o: ../config_app.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

text_parser.o: ../text_parser.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

#switch.o: ../switch.c
#	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

cycle_probing.o: ../cycle_probing.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

#xio.o: ../xio.c
#	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

cycle_jogging.o: ../cycle_jogging.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

plan_exec.o: ../plan_exec.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

encoder.o: ../encoder.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

sim.o: sim.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

size: ${TARGET}
	@echo
	@size -t ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) tinyg.elf dep/* tinyg.lss tinyg.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)
