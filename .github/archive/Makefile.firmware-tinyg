# Portable Makefile for building TinyG with a plain avr-gcc toolchain
# (macOS: brew tap osx-cross/avr && brew install avr-gcc avrdude)
#
# This is independent of the Atmel Studio project (tinyg.cproj) and the
# auto-generated Makefiles in Debug/ and Release/. Source list mirrors the
# Atmel Studio project: all top-level .c files except system.c, plus xio/
# and xmega/ (except xmega_wdt.c).
#
# Targets:  make            - build build/tinyg.elf and build/tinyg.hex
#           make size       - print section sizes
#           make flash      - flash over avrdude/xboot (set PORT=...)
#           make clean

MCU      = atxmega192a3
F_CPU    = 32000000UL
BUILDDIR = build

SRCS  = $(filter-out system.c,$(wildcard *.c))
SRCS += $(filter-out xio/xio.c,$(wildcard xio/*.c))
SRCS += $(filter-out xmega/xmega_wdt.c,$(wildcard xmega/*.c))

OBJS = $(addprefix $(BUILDDIR)/,$(SRCS:.c=.o))

CC      = avr-gcc
OBJCOPY = avr-objcopy
SIZE    = avr-size

CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -O2 -std=gnu99 -Wall \
          -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums \
          -ffunction-sections -fdata-sections -I. \
          -MMD -MP

LDFLAGS = -mmcu=$(MCU) -Wl,-Map=$(BUILDDIR)/tinyg.map -Wl,-u,vfprintf \
          -Wl,--start-group -lm -lprintf_flt -Wl,--end-group -Wl,--gc-sections

all: $(BUILDDIR)/tinyg.hex size

$(BUILDDIR)/tinyg.elf: $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

$(BUILDDIR)/tinyg.hex: $(BUILDDIR)/tinyg.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

$(BUILDDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

size: $(BUILDDIR)/tinyg.elf
	$(SIZE) $<

# Flash via the xboot bootloader (avr109 protocol). Set PORT to your device,
# e.g. make flash PORT=/dev/tty.usbserial-XXXX
PORT ?= /dev/tty.usbserial*
flash: $(BUILDDIR)/tinyg.hex
	avrdude -p x192a3 -c avr109 -b 115200 -P $(PORT) -U flash:w:$<

clean:
	rm -rf $(BUILDDIR)

-include $(OBJS:.o=.d)

.PHONY: all size flash clean
