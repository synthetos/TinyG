#!/bin/bash

files="README.md
Makefile
config.h.mk
conf/x64a3.conf.mk
conf/x32a4.conf.mk
conf/akafuino32a4.conf.mk
conf/arduino328p.conf.mk
conf/mongoose.conf.mk
xboot.c
xboot.h
flash.c
flash.h
eeprom_driver.c
eeprom_driver.h
protocol.h
sp_driver.S
sp_driver.h
uart.c
uart.h
i2c.c
i2c.h
fifo.c
fifo.h
watchdog.c
watchdog.h
api.c
api.h
xbootapi.c
xbootapi.h"

name=xboot

output=$name-$(date +%Y%m%d).tar.gz

pkg=pkg/$name

mkdir -p $pkg
rm -rf $pkg/*
for f in $files
do
  mkdir -p $pkg/$(dirname $f)
  cp $f $pkg/$f
done

cd pkg
tar -cvzf ../$output $name

